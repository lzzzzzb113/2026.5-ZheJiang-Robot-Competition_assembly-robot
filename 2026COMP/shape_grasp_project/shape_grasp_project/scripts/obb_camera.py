import cv2
import math
import torch
import numpy as np
from ultralytics import YOLO

# ==================== 配置区域 ====================
MODEL_PATH = "../runs/5093.pt"   # 训练好的模型路径
CAMERA_ID = 0                                   # 摄像头ID，0为默认
CONF_THRESHOLD = 0.25                           # 置信度阈值
USE_GPU = True                                  # 是否使用GPU推理
# =================================================

def draw_rotated_box(image, xyxy, angle_deg, color=(0, 255, 0), thickness=2):
    """
    在图像上绘制旋转框（通过矩形四个角点）
    参数:
        image: 图像数组
        xyxy: 水平框坐标 [x1, y1, x2, y2]
        angle_deg: 旋转角度（度数）
        color: 框的颜色
        thickness: 线条粗细     
    """
    x1, y1, x2, y2 = map(int, xyxy)
    # 计算中心点
    cx = (x1 + x2) // 2
    cy = (y1 + y2) // 2
    w = x2 - x1
    h = y2 - y1
    # 构建旋转矩形
    rect = ((cx, cy), (w, h), angle_deg)
    box_points = cv2.boxPoints(rect)
    box_points = np.int32(box_points)
    cv2.drawContours(image, [box_points], 0, color, thickness)

def main():
    # 1. 检查模型文件
    import os
    if not os.path.exists(MODEL_PATH):
        print(f"错误：模型文件不存在 {MODEL_PATH}")
        return

    # 2. 加载模型
    device = 'cuda:0' if USE_GPU and torch.cuda.is_available() else 'cpu'
    print(f"使用设备: {device}")
    model = YOLO(MODEL_PATH)
    model.to(device)
    print("模型加载成功！")
    print(f"模型类别: {model.names}")

    # 3. 打开摄像头
    cap = cv2.VideoCapture(CAMERA_ID)
    if not cap.isOpened():
        print("错误：无法打开摄像头")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    print("按 'q' 键退出预览。")

    # 颜色映射（可自行调整）
    color_map = {
        'si': (0, 255, 0),      # 绿色
        'san': (255, 0, 0),     # 蓝色
        'ti': (0, 0, 255),      # 红色
        'yuan': (0, 255, 255)   # 黄色
    }

    while True:
        ret, frame = cap.read()
        if not ret:
            print("无法读取摄像头画面")
            break

        # 4. 推理
        results = model(frame, conf=CONF_THRESHOLD, device=device)[0]

        # 5. 解析OBB结果
        if results.obb is not None:
            # 获取水平框坐标（用于绘制旋转框的近似位置）
            boxes_xyxy = results.obb.xyxy.cpu().numpy()   # [x1,y1,x2,y2]
            # 获取旋转框的中心、宽高、角度（弧度）
            boxes_xywhr = results.obb.xywhr.cpu().numpy() # [cx,cy,w,h,angle_rad]
            classes = results.obb.cls.cpu().numpy()       # 类别ID
            confs = results.obb.conf.cpu().numpy()        # 置信度

            for i in range(len(boxes_xyxy)):
                class_id = int(classes[i])
                class_name = model.names[class_id]
                confidence = confs[i]
                # 中心点坐标（水平框的中心，也是旋转框的中心）
                cx = boxes_xyxy[i][0] + (boxes_xyxy[i][2] - boxes_xyxy[i][0]) / 2
                cy = boxes_xyxy[i][1] + (boxes_xyxy[i][3] - boxes_xyxy[i][1]) / 2
                # 旋转角度（弧度转度数）
                angle_rad = boxes_xywhr[i][4]   # 范围通常在 [-π/2, π/2)
                angle_deg = angle_rad * 180.0 / math.pi
                # 输出到控制台（类别 + 角度）
                print(f"[{class_name}] 置信度:{confidence:.2f}, 中心:({cx:.1f},{cy:.1f}), 角度:{angle_deg:.1f}°")

                # 绘制旋转框（使用水平框和角度）
                color = color_map.get(class_name, (0, 255, 0))
                draw_rotated_box(frame, boxes_xyxy[i], angle_deg, color, thickness=2)
                
                # 显示类别、置信度、角度（图像上）
                label_text = f"{class_name} {confidence:.2f} {angle_deg:.1f}°"
                cv2.putText(frame, label_text, (int(boxes_xyxy[i][0]), int(boxes_xyxy[i][1])-5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # 6. 显示画面
        cv2.imshow("YOLO OBB Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("程序已退出。")

if __name__ == "__main__":
    main()