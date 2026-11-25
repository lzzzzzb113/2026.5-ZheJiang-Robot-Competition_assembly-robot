import cv2
import numpy as np
import json
import os

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

def get_shape_mask(frame, contour):
    """生成轮廓内部的掩膜，并返回掩膜内像素的HSV均值"""
    mask = np.zeros(frame.shape[:2], dtype=np.uint8)
    cv2.drawContours(mask, [contour], -1, 255, -1)  # 填充内部
    # 将原图转为HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # 提取掩膜内的HSV像素
    pixels = hsv[mask == 255]
    if len(pixels) == 0:
        return None
    # 返回均值
    mean_hsv = np.mean(pixels, axis=0)
    return mean_hsv

def calibrate():
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    print("=== 颜色校准模式 ===")
    print("请依次将【铝质（小）三角形】、【PE（大）三角形】放在摄像头前，按空格记录颜色")
    print("完成后按 'q' 退出")
    
    samples = {"aluminum": [], "pe": []}
    current_material = "aluminum"
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        display = frame.copy()
        cv2.putText(display, f"请放置: {current_material} 三角形", (20, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
        cv2.imshow("Calibration", display)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord(' '):
            # 检测当前帧中的三角形
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5,5), 0)
            _, binary = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
            kernel = np.ones((3,3), np.uint8)
            binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=1)
            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            detected = False
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < 500:
                    continue
                peri = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
                if len(approx) == 3:  # 三角形
                    mean_hsv = get_shape_mask(frame, cnt)
                    if mean_hsv is not None:
                        samples[current_material].append(mean_hsv)
                        print(f"记录 {current_material} 颜色: H={mean_hsv[0]:.1f}, S={mean_hsv[1]:.1f}, V={mean_hsv[2]:.1f}")
                        detected = True
                        # 切换材质
                        if current_material == "aluminum":
                            current_material = "pe"
                        else:
                            print("校准完成！")
                            cap.release()
                            cv2.destroyAllWindows()
                            # 计算模型
                            if len(samples["aluminum"])>0 and len(samples["pe"])>0:
                                alu_mean = np.mean(samples["aluminum"], axis=0)
                                alu_std = np.std(samples["aluminum"], axis=0)
                                pe_mean = np.mean(samples["pe"], axis=0)
                                pe_std = np.std(samples["pe"], axis=0)
                                model = {
                                    "aluminum": {"mean": alu_mean.tolist(), "std": alu_std.tolist()},
                                    "pe": {"mean": pe_mean.tolist(), "std": pe_std.tolist()}
                                }
                                with open("color_model.json", "w") as f:
                                    json.dump(model, f, indent=4)
                                print("颜色模型已保存到 color_model.json")
                            return
            if not detected:
                print("未检测到三角形，请调整位置或光照")
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    calibrate()