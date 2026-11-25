import cv2
import numpy as np
import json
import os

def is_trapezoid(approx):
    # 同校准程序中的定义
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

def detect_shapes_frame(frame, thresholds):
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
        
        label = None
        color = None
        if vertices == 3:
            # 三角形
            if area > thresholds["triangle"]["threshold"]:
                label = "Big Triangle"
            else:
                label = "Small Triangle"
            color = (0, 255, 0)   # 绿色轮廓
        elif vertices == 4 and is_trapezoid(approx):
            if area > thresholds["trapezoid"]["threshold"]:
                label = "Big Trapezoid"
            else:
                label = "Small Trapezoid"
            color = (255, 0, 0)   # 蓝色轮廓
        
        if label:
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
    # 加载阈值文件
    if not os.path.exists("thresholds.json"):
        print("未找到 thresholds.json，请先运行 calibrate.py 进行校准")
        return
    with open("thresholds.json", "r") as f:
        thresholds = json.load(f)
    print("已加载阈值：")
    print(f"  三角形: 小<{thresholds['triangle']['threshold']:.1f} <大")
    print(f"  梯形: 小<{thresholds['trapezoid']['threshold']:.1f} <大")
    
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
        result = detect_shapes_frame(frame, thresholds)
        cv2.imshow("Real-time Shape Detection", result)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()