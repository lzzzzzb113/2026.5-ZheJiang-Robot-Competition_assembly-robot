import cv2
import numpy as np
import json
import os

# 形状判断函数（复用）
def is_trapezoid(approx):
    if len(approx) != 4:
        return False
    pts = approx.reshape(-1, 2)
    angles = []
    for i in range(4):
        x1, y1 = pts[i]
        x2, y2 = pts[(i+1)%4]
        dx = x2 - x1
        dy = y2 - y1
        angle = np.arctan2(dy, dx)
        angles.append(angle)
    parallel_count = 0
    if abs(angles[0] - angles[2]) < 0.1 or abs(abs(angles[0]-angles[2]) - np.pi) < 0.1:
        parallel_count += 1
    if abs(angles[1] - angles[3]) < 0.1 or abs(abs(angles[1]-angles[3]) - np.pi) < 0.1:
        parallel_count += 1
    return parallel_count >= 1

def get_shape_hsv_mean(frame, contour):
    mask = np.zeros(frame.shape[:2], dtype=np.uint8)
    cv2.drawContours(mask, [contour], -1, 255, -1)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    pixels = hsv[mask == 255]
    if len(pixels) == 0:
        return None
    return np.mean(pixels, axis=0)

def classify_material(hsv_mean, model):
    """根据HSV均值判断材质，返回材质名称和信心度"""
    # 计算马氏距离（简化：各通道独立标准化后的欧氏距离）
    alu_mean = np.array(model["aluminum"]["mean"])
    alu_std = np.array(model["aluminum"]["std"])
    pe_mean = np.array(model["pe"]["mean"])
    pe_std = np.array(model["pe"]["std"])
    
    # 避免除零
    alu_std = np.maximum(alu_std, 1e-6)
    pe_std = np.maximum(pe_std, 1e-6)
    
    alu_dist = np.sum(((hsv_mean - alu_mean) / alu_std) ** 2)
    pe_dist = np.sum(((hsv_mean - pe_mean) / pe_std) ** 2)
    
    if alu_dist < pe_dist:
        return "aluminum", alu_dist
    else:
        return "pe", pe_dist

def detect_shapes_frame(frame, model):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5,5), 0)
    _, binary = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    kernel = np.ones((3,3), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=1)
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 500:
            continue
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
        vertices = len(approx)
        
        shape = None
        if vertices == 3:
            shape = "triangle"
        elif vertices == 4 and is_trapezoid(approx):
            shape = "trapezoid"
        else:
            continue
        
        # 提取颜色并分类
        hsv_mean = get_shape_hsv_mean(frame, cnt)
        if hsv_mean is None:
            continue
        material, _ = classify_material(hsv_mean, model)
        
        # 根据材质确定大小标签
        if shape == "triangle":
            label = "小三角形" if material == "aluminum" else "大三角形"
        else:  # trapezoid
            label = "小梯形" if material == "aluminum" else "大梯形"
        
        # 绘制轮廓和文字
        color = (0, 255, 0) if shape == "triangle" else (255, 0, 0)
        cv2.drawContours(frame, [cnt], -1, color, 2)
        M = cv2.moments(cnt)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
        else:
            cx, cy = cnt[0][0]
        cv2.putText(frame, label, (cx-40, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
    
    return frame

def main():
    if not os.path.exists("color_model.json"):
        print("未找到 color_model.json，请先运行 calibrate_color.py 进行校准")
        return
    with open("color_model.json", "r") as f:
        model = json.load(f)
    print("已加载颜色模型")
    
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    if not cap.isOpened():
        print("无法打开摄像头")
        return
    
    print("按 'q' 退出实时识别")
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        result = detect_shapes_frame(frame, model)
        cv2.imshow("Real-time Shape & Material Detection", result)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()