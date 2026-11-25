import cv2
import math
import numpy as np
import torch
import os
from ultralytics import YOLO

# ==================== 配置 ====================
SHAPE_MODEL_PATH = r'D:\2026COMP\shape_grasp_project\shape_grasp_project\runs\5102!.pt'
MATERIAL_MODEL_PATH = r"D:\2026COMP\shape_grasp_project\shape_grasp_project\runs\5141.pt"
CAMERA_ID = 1
CONF_THRESHOLD = 0.25
USE_GPU = True

# 形状默认颜色
COLOR_MAP = {
    'si': (0, 255, 0),     
    'san': (255, 0, 0),    
    'ti': (0, 0, 255),     
    'yuan': (0, 255, 255)  
}

# ==================== 核心：交并比 (IoU) 匹配算法 ====================
def get_iou(boxA, boxB):
    xA = max(boxA[0], boxB[0])
    yA = max(boxA[1], boxB[1])
    xB = min(boxA[2], boxB[2])
    yB = min(boxA[3], boxB[3])

    interArea = max(0, xB - xA) * max(0, yB - yA)
    if interArea == 0: 
        return 0.0

    boxAArea = (boxA[2] - boxA[0]) * (boxA[3] - boxA[1])
    boxBArea = (boxB[2] - boxB[0]) * (boxB[3] - boxB[1])

    return interArea / float(boxAArea + boxBArea - interArea)

# ==================== 核心：边缘锚定角度算法 ====================
def get_robust_angle_and_box(cnt, shape_type):
    rect = cv2.minAreaRect(cnt)
    (cx, cy), (w, h), theta = rect
    box = cv2.boxPoints(rect)
    box = np.int32(box) 
    
    if shape_type == 'san':
        # 1. 提取三角形的 3 个真实顶点
        retval, triangle = cv2.minEnclosingTriangle(cnt)
        if retval and triangle is not None and len(triangle) == 3:
            pts = triangle.reshape(3, 2)
            
            # 2. 找到最左侧的顶点 (X坐标最小)
            left_idx = np.argmin(pts[:, 0])
            p_left = pts[left_idx]
            
            # 找到与最左顶点相连的另外两个顶点
            p1 = pts[(left_idx + 1) % 3]
            p2 = pts[(left_idx + 2) % 3]
            
            # 3. 计算边 p_left -> p1 
            dx1 = p1[0] - p_left[0]
            dy1 = p1[1] - p_left[1] # 保留符号用于判断上下
            angle1 = math.degrees(math.atan2(abs(dx1), abs(dy1))) if (dx1!=0 or dy1!=0) else 90
            
            # 4. 计算边 p_left -> p2 
            dx2 = p2[0] - p_left[0]
            dy2 = p2[1] - p_left[1] # 保留符号用于判断上下
            angle2 = math.degrees(math.atan2(abs(dx2), abs(dy2))) if (dx2!=0 or dy2!=0) else 90
            
            # 5. 取夹角较小的那条边
            if angle1 < angle2:
                best_angle = angle1
                best_edge_pts = (p_left, p1)
                best_dy = dy1
            else:
                best_angle = angle2
                best_edge_pts = (p_left, p2)
                best_dy = dy2
                
            # 6. 【核心逻辑】：判断是上角还是下角
            # 在 OpenCV 中，越往下 y 值越大。所以 dy < 0 说明点在上方（上面的角）
            if best_dy < 0:
                best_angle += 180.0
                
            # 7. 归一化 (除以 360)
            best_angle = best_angle / 360.0
            
            return best_angle, box, best_edge_pts
        else:
            return 0.0, box, None
            
    # 其他形状的算法，统一恢复归一化
    elif shape_type == 'ti':
        M = cv2.moments(cnt)
        if M['m00'] == 0: 
            return 0.0, box, None
        mcx = M['m10'] / M['m00']
        mcy = M['m01'] / M['m00']
        vx = cx - mcx
        vy = cy - mcy
        best_angle = 0
        max_dot = -1
        for i in range(4):
            p1 = box[i]
            p2 = box[(i+1)%4]
            ex = p2[0] - p1[0]
            ey = p2[1] - p1[1]
            length = math.hypot(ex, ey)
            if length > 0:
                dx, dy = ex/length, ey/length
                dot = vx * dx + vy * dy
                if dot > max_dot:
                    max_dot = dot
                    best_angle = math.degrees(math.atan2(ey, ex))
        
        best_angle = (best_angle + 360) % 360
        best_angle = best_angle / 360.0
        return best_angle, box, None
        
    else:
        max_len = 0
        best_angle = 0
        for i in range(4):
            p1 = box[i]
            p2 = box[(i+1)%4]
            length = math.hypot(p2[0]-p1[0], p2[1]-p1[1])
            if length > max_len:
                max_len = length
                best_angle = math.degrees(math.atan2(p2[1]-p1[1], p2[0]-p1[0]))
        
        best_angle = (best_angle + 360) % 360
        best_angle = best_angle / 360.0
        return best_angle, box, None

# ==================== 几何信息提取 ====================
def process_roi_geometry(frame_thresh, x1, y1, x2, y2, shape_type):
    h, w = frame_thresh.shape
    margin = 15
    rx1, ry1 = max(0, int(x1) - margin), max(0, int(y1) - margin)
    rx2, ry2 = min(w, int(x2) + margin), min(h, int(y2) + margin)

    roi_thresh = frame_thresh[ry1:ry2, rx1:rx2]

    kernel = np.ones((5, 5), np.uint8)
    roi_thresh = cv2.morphologyEx(roi_thresh, cv2.MORPH_OPEN, kernel)
    roi_thresh = cv2.morphologyEx(roi_thresh, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(roi_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return 0.0, None, None

    cnt = max(contours, key=cv2.contourArea)
    precise_angle, box_local, best_edge_local = get_robust_angle_and_box(cnt, shape_type)
    
    box_global = box_local + np.array([rx1, ry1])
    
    # 还原“被选中的计算边缘”的全局画面坐标
    best_edge_global = None
    if best_edge_local is not None:
        p1 = best_edge_local[0] + np.array([rx1, ry1])
        p2 = best_edge_local[1] + np.array([rx1, ry1])
        best_edge_global = (tuple(map(int, p1)), tuple(map(int, p2)))

    return precise_angle, box_global, best_edge_global

# ==================== 主程序 ====================
def main():
    if not os.path.exists(SHAPE_MODEL_PATH) or not os.path.exists(MATERIAL_MODEL_PATH):
        print("[ERROR] 找不到模型文件，请检查模型路径！")
        return
        
    device = 'cuda:0' if USE_GPU and torch.cuda.is_available() else 'cpu'
    
    print("[INFO] 正在加载形状模型...")
    shape_model = YOLO(SHAPE_MODEL_PATH)
    print("[INFO] 正在加载材质模型...")
    material_model = YOLO(MATERIAL_MODEL_PATH)
    print("[INFO] 双模型加载成功！启动摄像头...")

    cap = cv2.VideoCapture(CAMERA_ID, cv2.CAP_DSHOW)
    if not cap.isOpened(): 
        print("[ERROR] 无法打开摄像头！")
        return
        
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  
    exposure_value = -5
    cap.set(cv2.CAP_PROP_EXPOSURE, exposure_value)
    print(f"[INFO] 💡 已尝试关闭自动曝光，当前手动曝光值设为: {exposure_value}")

    while True:
        ret, frame = cap.read()
        if not ret: break

        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, frame_thresh = cv2.threshold(frame_gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        shape_results = shape_model(frame, conf=CONF_THRESHOLD, device=device, verbose=False)[0]
        mat_results = material_model(frame, conf=CONF_THRESHOLD, device=device, verbose=False)[0]

        mat_boxes_info = []
        if hasattr(mat_results, 'boxes') and mat_results.boxes is not None:
            m_boxes = mat_results.boxes.xyxy.cpu().numpy()
            m_classes = mat_results.boxes.cls.cpu().numpy()
            for j in range(len(m_boxes)):
                m_cls_name = material_model.names[int(m_classes[j])]
                mat_boxes_info.append({'box': tuple(m_boxes[j]), 'material': m_cls_name})

        boxes_xyxy = []
        classes = []
        confs = []
        if hasattr(shape_results, 'obb') and shape_results.obb is not None:
            boxes_xyxy = shape_results.obb.xyxy.cpu().numpy()
            classes = shape_results.obb.cls.cpu().numpy()
            confs = shape_results.obb.conf.cpu().numpy()
        elif hasattr(shape_results, 'boxes') and shape_results.boxes is not None:
            boxes_xyxy = shape_results.boxes.xyxy.cpu().numpy()
            classes = shape_results.boxes.cls.cpu().numpy()
            confs = shape_results.boxes.conf.cpu().numpy()
            
        shape_names = shape_model.names

        for i in range(len(boxes_xyxy)):
            x1, y1, x2, y2 = boxes_xyxy[i]
            shape_class = shape_names[int(classes[i])]
            conf = confs[i]

            matched_material = "Unknown"
            best_iou = 0
            for mat_info in mat_boxes_info:
                iou = get_iou((x1, y1, x2, y2), mat_info['box'])
                if iou > best_iou and iou > 0.4:
                    best_iou = iou
                    matched_material = mat_info['material']

            if shape_class == 'yuan':
                precise_angle = 0.0
                box_global = None
                best_edge_global = None
            else:
                precise_angle, box_global, best_edge_global = process_roi_geometry(
                    frame_thresh, x1, y1, x2, y2, shape_class
                )
            
            # 恢复到保留 4 位小数（因为是归一化的 0~1）
            display_angle = round(precise_angle, 4)

            if matched_material in ["metal", "plastic"]:
                display_label = f"{matched_material}_{shape_class}"
                box_color = (0, 255, 0) if matched_material == "plastic" else (0, 0, 255)
            else:
                display_label = shape_class
                box_color = COLOR_MAP.get(shape_class, (255, 255, 255))

            # 绘制 YOLO 外接框
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), box_color, 1)
            
            # 绘制内部轮廓多边形
            if box_global is not None:
                cv2.polylines(frame, [box_global], isClosed=True, color=(255, 255, 0), thickness=1)

            # 针对三角形的高亮反馈
            if shape_class == 'san' and best_edge_global is not None:
                # 1. 穿过三角形最左端点画出垂线
                anchor_pt = best_edge_global[0]
                line_x = anchor_pt[0]
                cv2.line(frame, (line_x, int(y1)-10), (line_x, int(y2)+10), (0, 255, 255), 2)
                cv2.putText(frame, "Fixed Vertical", (line_x-20, int(y1)-15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
                
                # 2. 把算出较小角度的那条边，用极其显眼的红色加粗画出！
                cv2.line(frame, best_edge_global[0], best_edge_global[1], (0, 0, 255), 4)

            # 画面显示标签和归一化后的角度 (0.0000 ~ 0.9999)
            label_text = f"{display_label} {conf:.2f} Ang:{display_angle:.4f}"
            cv2.putText(frame, label_text, (int(x1), int(y1)-5), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)

        cv2.imshow("Dual Model Inference - Shape & Material", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'): break
        
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()