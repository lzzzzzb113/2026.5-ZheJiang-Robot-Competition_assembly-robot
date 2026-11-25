import cv2
import math
import numpy as np
import torch
import os
from ultralytics import YOLO

# ==================== 配置 ====================
MODEL_PATH = r"D:\shape_grasp_project\runs\5102!.pt"  # 请修改为实际路径
CAMERA_ID = 0
CONF_THRESHOLD = 0.25
USE_GPU = True

# 传统视觉参数
MIN_AREA_RATIO = 0.08          # 轮廓面积至少占ROI面积的比例
EPSILON_FACTOR = 0.02          # 多边形逼近精度
PARALLEL_ANGLE_THRESH = 10     # 梯形平行边角度差容许度

# 颜色映射 (BGR) - 用于水平检测框
COLOR_MAP = {
    'si': (0, 255, 0),     # 正方形绿色
    'san': (255, 0, 0),    # 三角形蓝色
    'ti': (0, 0, 255),     # 梯形红色
    'yuan': (0, 255, 255)  # 圆形黄色
}
# =================================================

def preprocess_roi(roi):
    """ROI预处理：灰度、高斯模糊、Canny边缘检测"""
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)
    return edges

def get_shape_approx(contour, epsilon_factor=0.02):
    peri = cv2.arcLength(contour, True)
    epsilon = epsilon_factor * peri
    approx = cv2.approxPolyDP(contour, epsilon, True)
    return len(approx), approx

def is_circle(contour, area_threshold_ratio=0.85):
    area = cv2.contourArea(contour)
    perimeter = cv2.arcLength(contour, True)
    if perimeter == 0:
        return False
    circularity = 4 * math.pi * area / (perimeter * perimeter)
    return circularity >= area_threshold_ratio

def angle_between_vectors(v1, v2):
    dot = v1[0]*v2[0] + v1[1]*v2[1]
    n1 = math.hypot(v1[0], v1[1])
    n2 = math.hypot(v2[0], v2[1])
    if n1 == 0 or n2 == 0:
        return 0.0
    cos = dot / (n1 * n2)
    cos = max(-1.0, min(1.0, cos))
    return math.degrees(math.acos(cos))

def get_trapezoid_angle(approx):
    """梯形：短边在上，方向向量 = 短边中点 → 长边中点（竖直向下为0°），返回带符号角度"""
    pts = approx.reshape(4, 2)
    edges = []
    for i in range(4):
        p1 = pts[i]
        p2 = pts[(i+1)%4]
        vec = p2 - p1
        ang = math.degrees(math.atan2(vec[1], vec[0]))
        length = np.linalg.norm(vec)
        edges.append((ang, length, p1, p2))
    # 找平行边对
    parallel = []
    for i in range(2):
        diff = abs(edges[i][0] - edges[i+2][0])
        diff = min(diff, 180 - diff)
        if diff < PARALLEL_ANGLE_THRESH:
            parallel.append((i, i+2))
    if len(parallel) != 1:
        return None
    i1, i2 = parallel[0]
    len1, len2 = edges[i1][1], edges[i2][1]
    short_idx = i1 if len1 < len2 else i2
    long_idx = i2 if len1 < len2 else i1
    p1_s, p2_s = edges[short_idx][2], edges[short_idx][3]
    mid_short = ((p1_s[0]+p2_s[0])/2, (p1_s[1]+p2_s[1])/2)
    p1_l, p2_l = edges[long_idx][2], edges[long_idx][3]
    mid_long = ((p1_l[0]+p2_l[0])/2, (p1_l[1]+p2_l[1])/2)
    direction = (mid_long[0]-mid_short[0], mid_long[1]-mid_short[1])
    vertical_down = (0, 1)
    angle = angle_between_vectors(direction, vertical_down)
    cross = direction[0]*vertical_down[1] - direction[1]*vertical_down[0]
    if cross < 0:
        angle = -angle
    return angle

def get_triangle_angle(approx):
    """三角形：最长边为底，方向向量 = 底边中点 → 对角顶点，与竖直向上（0,-1）的夹角"""
    pts = approx.reshape(3, 2)
    edges = []
    for i in range(3):
        p1 = pts[i]
        p2 = pts[(i+1)%3]
        length = np.linalg.norm(p2 - p1)
        edges.append((length, i, p1, p2))
    longest = max(edges, key=lambda x: x[0])
    _, idx, p1, p2 = longest
    opposite_idx = (idx + 2) % 3
    opposite_pt = pts[opposite_idx]
    mid_base = ((p1[0]+p2[0])/2, (p1[1]+p2[1])/2)
    direction = (opposite_pt[0]-mid_base[0], opposite_pt[1]-mid_base[1])
    vertical_up = (0, -1)
    angle = angle_between_vectors(direction, vertical_up)
    cross = direction[0]*vertical_up[1] - direction[1]*vertical_up[0]
    if cross < 0:
        angle = -angle
    return angle

def get_square_angle(contour):
    """正方形/长方形：最小外接矩形角度，归一化到[-45°,45°]（0°表示一条边水平）"""
    rect = cv2.minAreaRect(contour)
    angle = rect[2]
    if angle > 90:
        angle -= 180
    elif angle < -90:
        angle += 180
    return angle

def is_trapezoid(approx):
    if len(approx) != 4:
        return False
    pts = approx.reshape(4, 2)
    angles = []
    for i in range(4):
        p1 = pts[i]
        p2 = pts[(i+1)%4]
        vec = p2 - p1
        ang = math.degrees(math.atan2(vec[1], vec[0]))
        angles.append(ang)
    parallel_cnt = 0
    for i in range(2):
        diff = abs(angles[i] - angles[i+2])
        diff = min(diff, 180 - diff)
        if diff < PARALLEL_ANGLE_THRESH:
            parallel_cnt += 1
    return parallel_cnt == 1

def compute_angle_from_contour(contour, shape_type):
    """
    根据轮廓和YOLO给出的类别计算对应角度。
    shape_type: 'si', 'san', 'ti', 'yuan'
    """
    if shape_type == 'si':   # 正方形
        return get_square_angle(contour)
    elif shape_type == 'san': # 三角形
        epsilon = EPSILON_FACTOR * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        if len(approx) == 3:
            return get_triangle_angle(approx)
        else:
            return None
    elif shape_type == 'ti':  # 梯形
        epsilon = EPSILON_FACTOR * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        if len(approx) == 4 and is_trapezoid(approx):
            return get_trapezoid_angle(approx)
        else:
            return None
    elif shape_type == 'yuan': # 圆形
        return 0.0
    else:
        return None

def extract_precise_info(roi, yolo_class, offset_x, offset_y, min_area_ratio=0.08):
    """
    在ROI中提取最大轮廓，并计算角度。
    返回 (success, contour_global, angle)
    """
    if roi is None or roi.size == 0:
        return False, None, None
    edges = preprocess_roi(roi)
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return False, None, None
    cnt = max(contours, key=cv2.contourArea)
    roi_area = roi.shape[0] * roi.shape[1]
    if cv2.contourArea(cnt) < min_area_ratio * roi_area:
        return False, None, None
    cnt_global = cnt + np.array([offset_x, offset_y])
    angle = compute_angle_from_contour(cnt, yolo_class)
    if angle is None:
        return False, cnt_global, None
    return True, cnt_global, angle

def draw_detection_box(image, xyxy, color, thickness=2):
    """绘制YOLO检测的水平矩形框"""
    x1, y1, x2, y2 = map(int, xyxy)
    cv2.rectangle(image, (x1, y1), (x2, y2), color, thickness)

def main():
    if not os.path.exists(MODEL_PATH):
        print(f"模型文件不存在: {MODEL_PATH}")
        return
    device = 'cuda:0' if USE_GPU and torch.cuda.is_available() else 'cpu'
    print(f"使用设备: {device}")
    model = YOLO(MODEL_PATH)
    print("模型加载成功")

    cap = cv2.VideoCapture(CAMERA_ID)
    if not cap.isOpened():
        print("无法打开摄像头")
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    print("按 'q' 退出")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        results = model(frame, conf=CONF_THRESHOLD, device=device)[0]
        if results.obb is None:
            cv2.imshow("Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            continue

        boxes_xyxy = results.obb.xyxy.cpu().numpy()
        boxes_xywhr = results.obb.xywhr.cpu().numpy()   # [cx,cy,w,h,angle_rad]
        classes = results.obb.cls.cpu().numpy()
        confs = results.obb.conf.cpu().numpy()
        names = model.names

        for i in range(len(boxes_xyxy)):
            x1, y1, x2, y2 = map(int, boxes_xyxy[i])
            class_id = int(classes[i])
            yolo_class = names[class_id]   # 'si','san','ti','yuan'
            confidence = confs[i]

            # 获取OBB的旋转角度作为备选（弧度转度数）
            obb_angle_rad = boxes_xywhr[i][4]
            fallback_angle = obb_angle_rad * 180.0 / math.pi

            # 尝试传统视觉精确角度
            margin = 15
            roi_x1 = max(0, x1 - margin)
            roi_y1 = max(0, y1 - margin)
            roi_x2 = min(frame.shape[1], x2 + margin)
            roi_y2 = min(frame.shape[0], y2 + margin)
            roi = frame[roi_x1:roi_x2, roi_y1:roi_y2]
            success, cnt_global, precise_angle = False, None, None
            if roi.size > 0:
                success, cnt_global, precise_angle = extract_precise_info(roi, yolo_class, roi_x1, roi_y1, min_area_ratio=0.08)

            # 最终使用的角度：优先使用传统精确角度，否则用OBB角度
            final_angle = precise_angle if (success and precise_angle is not None) else fallback_angle

            # 绘制检测框（水平矩形框，颜色按类别）
            color = COLOR_MAP.get(yolo_class, (255, 255, 255))
            draw_detection_box(frame, boxes_xyxy[i], color, thickness=2)

            # 如果传统视觉成功，绘制绿色精确轮廓
            if success and cnt_global is not None:
                cv2.drawContours(frame, [cnt_global], -1, (0, 255, 0), 2)

            # 显示文本：类别 置信度 角度（若使用OBB角度可加注*）
            if success and precise_angle is not None:
                label = f"{yolo_class} {confidence:.2f} {final_angle:.1f}°"
            else:
                label = f"{yolo_class} {confidence:.2f} {final_angle:.1f}°(OBB)"
            cv2.putText(frame, label, (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # 控制台输出
            cx = (x1 + x2) / 2
            cy = (y1 + y2) / 2
            if success and precise_angle is not None:
                print(f"[{yolo_class}] conf={confidence:.2f}, center=({cx:.1f},{cy:.1f}), angle={final_angle:.1f} (precise)")
            else:
                print(f"[{yolo_class}] conf={confidence:.2f}, center=({cx:.1f},{cy:.1f}), angle={final_angle:.1f} (OBB fallback)")

        cv2.imshow("Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()