import cv2
import numpy as np
import json
import os

# 定义形状识别函数（与之前相同，可复用）
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

def detect_shapes_in_frame(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5,5), 0)
    _, binary = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    kernel = np.ones((3,3), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=1)
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    triangles = []  # 存储 (面积, 轮廓, approx)
    trapezoids = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 500:
            continue
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
        vertices = len(approx)
        if vertices == 3:
            triangles.append((area, cnt, approx))
        elif vertices == 4 and is_trapezoid(approx):
            trapezoids.append((area, cnt, approx))
    return triangles, trapezoids

def calibrate():
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    print("=== 校准模式 ===")
    print("请依次将【小三角形】、【大三角形】、【小梯形】、【大梯形】放在摄像头前，并按下空格键记录面积")
    print("按 'q' 退出校准")
    
    triangles_areas = []   # 存放 [小三角形面积, 大三角形面积]
    trapezoids_areas = []
    current_target = "小三角形"
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        display = frame.copy()
        cv2.putText(display, f"请放置: {current_target}", (20, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
        cv2.imshow("Calibration", display)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord(' '):   # 空格键记录当前画面中检测到的形状
            triangles, trapezoids = detect_shapes_in_frame(frame)
            if current_target.startswith("三角形") and triangles:
                # 取面积最大的三角形（假设画面中只有一个目标形状）
                area = triangles[-1][0]   # 按面积排序后的最大（但列表未排序，我们直接取第一个）
                # 为确保，我们按面积排序取最大的（但校准时应只有单个形状）
                triangles_sorted = sorted(triangles, key=lambda x: x[0])
                area = triangles_sorted[-1][0]
                triangles_areas.append(area)
                print(f"记录 {current_target}: 面积 = {area:.1f}")
                # 切换到下一个目标
                if current_target == "小三角形":
                    current_target = "大三角形"
                elif current_target == "大三角形":
                    current_target = "小梯形"
            elif current_target.startswith("梯形") and trapezoids:
                area = sorted(trapezoids, key=lambda x: x[0])[-1][0]
                trapezoids_areas.append(area)
                print(f"记录 {current_target}: 面积 = {area:.1f}")
                if current_target == "小梯形":
                    current_target = "大梯形"
                elif current_target == "大梯形":
                    print("校准完成！")
                    break
            else:
                print(f"未检测到 {current_target}，请调整位置或光照")
    
    cap.release()
    cv2.destroyAllWindows()
    
    if len(triangles_areas) == 2 and len(trapezoids_areas) == 2:
        # 计算面积分界阈值（取两个面积的中值）
        tri_threshold = (triangles_areas[0] + triangles_areas[1]) / 2
        trap_threshold = (trapezoids_areas[0] + trapezoids_areas[1]) / 2
        thresholds = {
            "triangle": {"small": min(triangles_areas), "large": max(triangles_areas), "threshold": tri_threshold},
            "trapezoid": {"small": min(trapezoids_areas), "large": max(trapezoids_areas), "threshold": trap_threshold}
        }
        with open("thresholds.json", "w") as f:
            json.dump(thresholds, f, indent=4)
        print("阈值已保存到 thresholds.json")
        print(f"三角形阈值: {tri_threshold:.1f} (面积大于此值为大三角形)")
        print(f"梯形阈值: {trap_threshold:.1f}")
    else:
        print("校准失败，未完整记录所有形状")

if __name__ == "__main__":
    calibrate()