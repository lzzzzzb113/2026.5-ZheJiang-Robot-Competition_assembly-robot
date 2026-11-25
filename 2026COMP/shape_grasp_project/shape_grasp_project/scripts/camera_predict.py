import cv2
from ultralytics import YOLO
import sys
import os

# ==================== 配置区域 ====================
# 模型路径（请根据你的实际路径调整）
MODEL_PATH = os.path.join('shape_grasp_project', 'runs', '5184.pt')
# 摄像头ID（0为默认摄像头，如果有多个摄像头可改为1,2...）
CAMERA_ID = 1
# 设置摄像头分辨率
FRAME_WIDTH = 1000
FRAME_HEIGHT = 1200
# =================================================

def main():
    # 检查模型文件是否存在
    if not os.path.exists(MODEL_PATH):
        print(f"错误：模型文件不存在 {MODEL_PATH}")
        print("请检查模型路径是否正确。")
        sys.exit(1)

    # 加载模型
    print(f"正在加载模型：{MODEL_PATH}")
    model = YOLO(MODEL_PATH)
    print("模型加载成功！")

    # 打开摄像头
    cap = cv2.VideoCapture(CAMERA_ID)
    if not cap.isOpened():
        print("错误：无法打开摄像头。请检查摄像头连接或CAMERA_ID设置。")
        sys.exit(1)

    # 设置摄像头分辨率
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    print(f"摄像头已打开，分辨率：{FRAME_WIDTH}x{FRAME_HEIGHT}")

    print("按 'q' 退出实时预览。")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("警告：无法读取摄像头画面，可能已断开。")
            break

        # 推理
        results = model(frame)[0]

        # 绘制检测结果
        annotated_frame = results.plot()

        # 获取检测框信息并输出中心点
        if results.boxes is not None:
            boxes = results.boxes.xyxy.cpu().numpy()  # [x1, y1, x2, y2]
            classes = results.boxes.cls.cpu().numpy() # 类别ID
            confs = results.boxes.conf.cpu().numpy()  # 置信度

            for i, box in enumerate(boxes):
                # 计算中心点像素坐标
                x_center = (box[0] + box[2]) / 2
                y_center = (box[1] + box[3]) / 2
                class_name = model.names[int(classes[i])]
                print(f"[{class_name}] 置信度 {confs[i]:.2f} 中心点: ({x_center:.1f}, {y_center:.1f})")

        # 显示画面
        cv2.imshow('Camera Detection', annotated_frame)

        # 按 'q' 退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 释放资源
    cap.release()
    cv2.destroyAllWindows()
    print("程序已退出。")

if __name__ == '__main__':
    main()