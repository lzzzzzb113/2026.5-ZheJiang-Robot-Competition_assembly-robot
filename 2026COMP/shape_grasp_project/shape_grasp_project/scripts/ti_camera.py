import cv2
import numpy as np
from ultralytics import YOLO
import sys
import os

# ==================== 配置区域 ====================
MODEL_PATH = os.path.join('..', 'runs', 'detect', 'train', 'weights', '412.pt')  # 请根据实际路径修改
CAMERA_ID = 0
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
# 梯形顶点检测参数
CANNY_THRESH1 = 50
CANNY_THRESH2 = 150
APPROX_EPSILON_FACTOR = 0.02   # 多边形逼近精度（相对于周长）
# =================================================

def get_trapezoid_vertices(roi):
    """
    在ROI中检测梯形的四个顶点
    返回: 四个点坐标的列表 [(x1,y1), (x2,y2), (x3,y3), (x4,y4)] (相对于ROI的坐标)，若失败返回None
    """
    if roi is None or roi.size == 0:
        return None
    
    # 转为灰度图
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    # 高斯模糊去噪
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    # 边缘检测
    edges = cv2.Canny(blurred, CANNY_THRESH1, CANNY_THRESH2)
    # 膨胀使边缘闭合
    kernel = np.ones((3,3), np.uint8)
    dilated = cv2.dilate(edges, kernel, iterations=2)
    
    # 查找轮廓
    contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    
    # 按面积降序排序，取面积最大的轮廓
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    largest_contour = contours[0]
    
    # 多边形逼近，得到顶点
    epsilon = APPROX_EPSILON_FACTOR * cv2.arcLength(largest_contour, True)
    approx = cv2.approxPolyDP(largest_contour, epsilon, True)
    
    # 如果逼近后的顶点数不是4，可能是梯形但轮廓检测不完美，尝试再次逼近或返回None
    if len(approx) != 4:
        # 尝试不同的epsilon
        for factor in [0.01, 0.03, 0.04, 0.05]:
            epsilon = factor * cv2.arcLength(largest_contour, True)
            approx = cv2.approxPolyDP(largest_contour, epsilon, True)
            if len(approx) == 4:
                break
        else:
            return None
    
    # 将顶点坐标转换为二维点列表
    vertices = approx.reshape(-1, 2).tolist()
    # 按顺时针或逆时针排序（可选，方便观察）
    # 计算中心点，然后按角度排序
    center = np.mean(vertices, axis=0)
    def angle_from_center(p):
        return np.arctan2(p[1]-center[1], p[0]-center[0])
    vertices.sort(key=angle_from_center)
    return vertices

def draw_vertices(frame, vertices, color=(0, 0, 255), radius=5, thickness=-1):
    """在原图上绘制顶点圆圈，并标注坐标"""
    for i, (x, y) in enumerate(vertices):
        cv2.circle(frame, (x, y), radius, color, thickness)
        # 显示坐标文本（略微偏移）
        text = f"{i+1}:({x},{y})"
        cv2.putText(frame, text, (x+10, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

def main():
    # 检查模型文件
    if not os.path.exists(MODEL_PATH):
        print(f"错误：模型文件不存在 {MODEL_PATH}")
        sys.exit(1)

    print(f"正在加载模型：{MODEL_PATH}")
    model = YOLO(MODEL_PATH)
    print("模型加载成功！")
    print("模型类别列表:", model.names)

    cap = cv2.VideoCapture(CAMERA_ID)
    if not cap.isOpened():
        print("错误：无法打开摄像头。")
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    print(f"摄像头已打开，分辨率：{FRAME_WIDTH}x{FRAME_HEIGHT}")
    print("按 'q' 退出实时预览。")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("警告：无法读取摄像头画面，可能已断开。")
            break

        # YOLO推理
        results = model(frame)[0]
        annotated_frame = results.plot()

        if results.boxes is not None:
            boxes = results.boxes.xyxy.cpu().numpy()
            classes = results.boxes.cls.cpu().numpy()
            confs = results.boxes.conf.cpu().numpy()

            for i, box in enumerate(boxes):
                x1, y1, x2, y2 = box.astype(int)
                class_name = model.names[int(classes[i])]
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                print(f"[{class_name}] 置信度 {confs[i]:.2f} 中心点: ({center_x:.1f}, {center_y:.1f})")

                # 如果是梯形，检测四个顶点
                if class_name == 'ti':  # 根据你的模型实际类别名修改
                    # 提取ROI（稍微扩大边界）
                    margin = 10
                    roi_x1 = max(0, x1 - margin)
                    roi_y1 = max(0, y1 - margin)
                    roi_x2 = min(frame.shape[1], x2 + margin)
                    roi_y2 = min(frame.shape[0], y2 + margin)
                    roi = frame[roi_y1:roi_y2, roi_x1:roi_x2]

                    vertices_roi = get_trapezoid_vertices(roi)
                    if vertices_roi is not None:
                        # 将ROI坐标系中的顶点转换到原图坐标系
                        vertices_global = [(x + roi_x1, y + roi_y1) for (x, y) in vertices_roi]
                        print(f"梯形四个顶点坐标（左上角为原点）: {vertices_global}")
                        # 在画面上绘制顶点
                        draw_vertices(annotated_frame, vertices_global, color=(0, 0, 255), radius=6, thickness=2)
                    else:
                        print("梯形顶点检测失败")

        cv2.imshow('Camera Detection', annotated_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("程序已退出。")

if __name__ == '__main__':
    main()