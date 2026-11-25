import cv2
import os
import numpy as np
from datetime import datetime

# ============================================
# 配置
# ============================================
DATASET_ROOT = "./dataset"           
SAVE_MODE = "train"                  
IMG_SIZE = (640, 480)                

CLASS_NAMES = {
    ord("1"): ("metal", 0),      
    ord("2"): ("plastic", 1),    
}

def create_dirs():
    for split in ["train", "val"]:
        os.makedirs(f"{DATASET_ROOT}/images/{split}", exist_ok=True)
        os.makedirs(f"{DATASET_ROOT}/labels/{split}", exist_ok=True)
    print(f"[INFO] 数据集目录已创建: {DATASET_ROOT}")

# ============================================
# 自动抓取物块并保存图片和标注
# ============================================
def auto_label_sample(frame, class_id, class_name):
    global SAVE_MODE
    h_img, w_img = frame.shape[:2]
    center_x, center_y = w_img / 2, h_img / 2

    # 1. 转灰度图
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # 【核心修复 1】：创建 ROI (感兴趣区域) 遮罩，强行忽略左右两侧的铁轨
    # 假设左右两边各占 20% 的宽度，我们把它涂黑
    roi_mask = np.zeros_like(gray)
    margin_x = int(w_img * 0.20)  # 如果铁轨更宽，可以把 0.20 改大（比如 0.25）
    roi_mask[:, margin_x:w_img-margin_x] = 255
    
    # 把不在中间传送带上的东西全部抹除
    gray_roi = cv2.bitwise_and(gray, gray, mask=roi_mask)

    # 2. 二值化寻找亮色物体
    _, mask = cv2.threshold(gray_roi, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # 3. 形态学去噪
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # 4. 寻找轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # 过滤掉太小的噪点
    valid_contours = [c for c in contours if cv2.contourArea(c) > 1000]
    
    if not valid_contours:
        print("[WARNING] 中间区域未检测到物块！")
        return False

    # 【核心修复 2】：寻找距离画面“正中心”最近的轮廓，而不是最大的！
    best_cnt = None
    min_dist = float('inf')
    
    for c in valid_contours:
        M = cv2.moments(c)
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            # 计算轮廓重心到画面中心的距离的平方
            dist = (cx - center_x)**2 + (cy - center_y)**2
            if dist < min_dist:
                min_dist = dist
                best_cnt = c

    if best_cnt is None:
        return False

    # 5. 获取边界框并稍微外扩一点边距，防止贴得太死
    x, y, w, h = cv2.boundingRect(best_cnt)
    pad = 10
    x = max(0, x - pad)
    y = max(0, y - pad)
    w = min(w_img - x, w + pad * 2)
    h = min(h_img - y, h + pad * 2)

    # 6. 转换为 YOLO 格式 (中心点x, 中心点y, 宽, 高) 且归一化
    x_center = (x + w / 2) / w_img
    y_center = (y + h / 2) / h_img
    box_w = w / w_img
    box_h = h / h_img

    # 7. 保存文件
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    img_filename = f"{class_name}_{timestamp}.jpg"
    label_filename = f"{class_name}_{timestamp}.txt"

    img_path = f"{DATASET_ROOT}/images/{SAVE_MODE}/{img_filename}"
    label_path = f"{DATASET_ROOT}/labels/{SAVE_MODE}/{label_filename}"

    cv2.imwrite(img_path, frame)
    with open(label_path, "w") as f:
        f.write(f"{class_id} {x_center:.6f} {y_center:.6f} {box_w:.6f} {box_h:.6f}\n")

    print(f"[SAVED] {SAVE_MODE}/{img_filename} (中心精准框抓取成功!)")
    
    # 8. 闪烁绿色提示框
    preview = frame.copy()
    cv2.rectangle(preview, (x, y), (x+w, y+h), (0, 255, 0), 3)
    cv2.imshow("Dataset Capture - YOLO Material Classification", preview)
    cv2.waitKey(200)

    return True

# ============================================
# 主循环
# ============================================
def main():
    global SAVE_MODE
    create_dirs()

    cap = cv2.VideoCapture(1)
    if not cap.isOpened():
        print("[ERROR] 无法打开摄像头")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMG_SIZE[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMG_SIZE[1])

    while True:
        ret, frame = cap.read()
        if not ret: break

        display = frame.copy()
        h, w = display.shape[:2]
        
     #设置拍照范围
        margin_x = int(w * 0.20)
        cv2.line(display, (margin_x, 0), (margin_x, h), (0, 0, 255), 1)
        cv2.line(display, (w - margin_x, 0), (w - margin_x, h), (0, 0, 255), 1)
        cv2.putText(display, "Detection Zone", (margin_x + 10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        cv2.putText(display, f"Mode: {SAVE_MODE.upper()} | [1]Metal [2]Plastic", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(display, "[s]witch mode  [q]uit", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        cv2.imshow("Dataset Capture - YOLO Material Classification", display)

        key = cv2.waitKey(1) & 0xFF

        if key in CLASS_NAMES:
            class_name, class_id = CLASS_NAMES[key]
            auto_label_sample(frame, class_id, class_name)
        elif key == ord("s"):
            SAVE_MODE = "val" if SAVE_MODE == "train" else "train"
        elif key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
