import cv2
import math
import torch
import numpy as np
import os
from ultralytics import YOLO

# ==================== 配置区域 ====================
MODEL_PATH = "../runs/5102!.pt"   # 你的YOLO OBB模型路径
CAMERA_ID = 0
CONF_THRESHOLD = 0.25
USE_GPU = True

# 材质分类参数（来源于原始代码，可依光照微调）
PLASTIC_MEAN_THRESH = 170     # 塑料平均灰度阈值
PLASTIC_STD_THRESH = 25       # 塑料标准差阈值
# =================================================

def draw_rotated_box(image, xyxy, angle_deg, color=(0, 255, 0), thickness=2):
    """绘制旋转框"""
    x1, y1, x2, y2 = map(int, xyxy)
    cx = (x1 + x2) // 2
    cy = (y1 + y2) // 2
    w = x2 - x1
    h = y2 - y1
    rect = ((cx, cy), (w, h), angle_deg)
    box_points = cv2.boxPoints(rect)
    box_points = np.int32(box_points)
    cv2.drawContours(image, [box_points], 0, color, thickness)

def classify_triangle_material(roi):
    """
    在ROI中判断三角形材质（塑料/金属）
    返回: (material_label, mean_gray, std_gray)
    """
    if roi is None or roi.size == 0:
        return "Unknown", 0, 0

    # 1. 转HSV并去除绿色背景（假设背景为绿色）
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    lower_green = np.array([35, 30, 20])
    upper_green = np.array([90, 255, 255])
    green_mask = cv2.inRange(hsv, lower_green, upper_green)
    mask = cv2.bitwise_not(green_mask)

    # 形态学处理
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # 2. 查找轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return "Unknown", 0, 0

    # 取最大轮廓并近似为三角形
    cnt = max(contours, key=cv2.contourArea)
    epsilon = 0.04 * cv2.arcLength(cnt, True)
    approx = cv2.approxPolyDP(cnt, epsilon, True)
    if len(approx) != 3:
        return "Unknown", 0, 0

    # 3. 生成三角形掩膜
    tri_mask = np.zeros(roi.shape[:2], dtype=np.uint8)
    cv2.drawContours(tri_mask, [approx], -1, 255, -1)

    # 4. 计算灰度统计
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    mean_gray, std_gray = cv2.meanStdDev(gray, mask=tri_mask)
    mean_gray = mean_gray[0][0]
    std_gray = std_gray[0][0]

    # 5. 分类
    if mean_gray > PLASTIC_MEAN_THRESH and std_gray < PLASTIC_STD_THRESH:
        material = "Plastic"
    else:
        material = "Metal"
    return material, mean_gray, std_gray

def main():
    # 检查模型
    if not os.path.exists(MODEL_PATH):
        print(f"错误：模型文件不存在 {MODEL_PATH}")
        return

    device = 'cuda:0' if USE_GPU and torch.cuda.is_available() else 'cpu'
    print(f"使用设备: {device}")
    model = YOLO(MODEL_PATH)
    model.to(device)
    print("模型加载成功！")
    print(f"模型类别: {model.names}")

    cap = cv2.VideoCapture(CAMERA_ID)
    if not cap.isOpened():
        print("错误：无法打开摄像头")
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    print("按 'q' 键退出预览。")

    color_map = {
        'si': (0, 255, 0),
        'san': (255, 0, 0),
        'ti': (0, 0, 255),
        'yuan': (0, 255, 255)
    }

    while True:
        ret, frame = cap.read()
        if not ret:
            print("无法读取摄像头画面")
            break

        results = model(frame, conf=CONF_THRESHOLD, device=device)[0]

        if results.obb is not None:
            boxes_xyxy = results.obb.xyxy.cpu().numpy()
            boxes_xywhr = results.obb.xywhr.cpu().numpy()
            classes = results.obb.cls.cpu().numpy()
            confs = results.obb.conf.cpu().numpy()

            for i in range(len(boxes_xyxy)):
                x1, y1, x2, y2 = map(int, boxes_xyxy[i])
                class_id = int(classes[i])
                class_name = model.names[class_id]
                confidence = confs[i]

                # 获取OBB角度（默认值）
                angle_rad = boxes_xywhr[i][4]
                angle_deg = angle_rad * 180.0 / math.pi

                # 材质分类（仅对三角形执行）
                material = None
                if class_name == 'san':
                    roi = frame[y1:y2, x1:x2]
                    if roi.size > 0:
                        material, mean_gray, std_gray = classify_triangle_material(roi)
                        if material != "Unknown":
                            # 可根据材质改变框的颜色（例如塑料绿色框，金属红色框）
                            if material == "Plastic":
                                box_color = (0, 255, 0)   # 绿色
                            else:
                                box_color = (0, 0, 255)   # 红色
                        else:
                            box_color = color_map.get(class_name, (255, 0, 0))
                    else:
                        material = "Unknown"
                        box_color = color_map.get(class_name, (255, 0, 0))
                else:
                    box_color = color_map.get(class_name, (0, 255, 0))

                # 绘制旋转框
                draw_rotated_box(frame, boxes_xyxy[i], angle_deg, box_color, thickness=2)

                # 显示类别、置信度和材质
                if material:
                    label_text = f"{class_name} {confidence:.2f}  {material}"
                else:
                    label_text = f"{class_name} {confidence:.2f}"
                cv2.putText(frame, label_text, (x1, y1-5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2)

                # 控制台输出
                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2
                if material:
                    print(f"[{class_name}] conf={confidence:.2f}, center=({cx:.1f},{cy:.1f}), angle={angle_deg:.1f}°, material={material}")
                else:
                    print(f"[{class_name}] conf={confidence:.2f}, center=({cx:.1f},{cy:.1f}), angle={angle_deg:.1f}°")

        cv2.imshow("YOLO + Material Classification", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("程序已退出。")

if __name__ == "__main__":
    main()