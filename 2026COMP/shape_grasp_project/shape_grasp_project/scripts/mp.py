"""
金属（铝本色）vs PE 塑料 实时区分系统
纯传统视觉方法，无需训练，基于多特征加权投票。
原理：
- 金属：高光区域小而亮、纹理对比度高、梯度最大值大、颜色偏冷（低色相、低饱和度）
- 塑料：高光区域弥散且亮度较低、纹理对比度低、梯度平缓、颜色偏暖（较高色相、中等饱和度）
"""

import cv2
import numpy as np

# ---------------------------- 可调参数 ---------------------------------
# 分割参数
BG_REMOVE_THRESH = 40          # 背景差分阈值 (0-255)
MORPH_KERNEL_SIZE = 5          # 形态学核大小
MIN_OBJECT_AREA = 1000         # 最小物体面积（像素）

# 高光特征参数
HIGHLIGHT_FACTOR = 1.8         # 高光阈值 = V均值 + HIGHLIGHT_FACTOR * V标准差
MIN_HL_AREA_RATIO = 0.01       # 高光面积占比低于此值倾向于金属
MAX_HL_AREA_RATIO = 0.15       # 高于此值倾向于塑料
HL_BRIGHTNESS_TH = 200         # 高光平均亮度 > 此值倾向于金属（0-255）

# 纹理特征参数 (基于灰度共生矩阵对比度)
GLCM_DIST = 1                  # 像素距离
GLCM_ANGLE = 0                 # 角度 (0°)
TEXTURE_METAL_TH = 50          # 对比度 > 此值倾向于金属

# 梯度特征参数
GRAD_MAX_METAL_TH = 80         # 梯度最大值 > 此值倾向于金属
GRAD_MEAN_METAL_TH = 20        # 梯度均值 > 此值倾向于金属

# 颜色特征参数（HSV）
HUE_METAL_MAX = 90             # 金属色调通常 < 90（冷色）; 塑料 > 90? 实际需调整
SAT_METAL_MAX = 50             # 金属饱和度通常较低（<50）; 塑料较高

# 投票权重（每个特征对金属的贡献分，塑料相反）
WEIGHT_HL_RATIO = 2            # 高光面积占比（负相关）
WEIGHT_HL_BRIGHT = 2           # 高光亮度（正相关）
WEIGHT_TEXTURE = 3             # 纹理对比度（正相关）
WEIGHT_GRAD_MAX = 2            # 梯度最大值（正相关）
WEIGHT_GRAD_MEAN = 1           # 梯度均值（正相关）
WEIGHT_HUE = 1                 # 色调（负相关：低色调为金属）
WEIGHT_SAT = 1                 # 饱和度（负相关）

# 判定阈值
METAL_VOTE_TH = 5              # 总投票得分 >= 此值判为金属，否则塑料

# ----------------------------------------------------------------------

def segment_object(frame, background, thresh_val=BG_REMOVE_THRESH):
    """
    背景差分法分割物体，返回二值掩码（物体区域为255）
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    bg_gray = cv2.cvtColor(background, cv2.COLOR_BGR2GRAY)
    diff = cv2.absdiff(gray, bg_gray)
    _, mask = cv2.threshold(diff, thresh_val, 255, cv2.THRESH_BINARY)
    # 形态学清理
    kernel = np.ones((MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    # 提取最大连通域
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        max_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(max_contour) < MIN_OBJECT_AREA:
            return np.zeros_like(mask)
        mask = np.zeros_like(mask)
        cv2.drawContours(mask, [max_contour], -1, 255, -1)
    return mask

def compute_features(frame, mask):
    """
    从帧和掩码中提取特征：
    返回字典: { 'hl_ratio', 'hl_brightness', 'texture_contrast', 'grad_max', 'grad_mean', 'hue_mean', 'sat_mean' }
    """
    if np.sum(mask) < 100:
        return None
    # 只取物体区域
    obj = cv2.bitwise_and(frame, frame, mask=mask)
    gray = cv2.cvtColor(obj, cv2.COLOR_BGR2GRAY)
    mask_bool = mask > 0

    # ---------- 1. 高光特征 (V通道) ----------
    hsv = cv2.cvtColor(obj, cv2.COLOR_BGR2HSV)
    v_channel = hsv[:,:,2]
    v_vals = v_channel[mask_bool].astype(np.float32)
    v_mean = np.mean(v_vals)
    v_std = np.std(v_vals)
    hl_th = v_mean + HIGHLIGHT_FACTOR * v_std
    hl_mask = (v_channel > hl_th) & (mask > 0)
    hl_area = np.sum(hl_mask)
    total_area = np.sum(mask_bool)
    hl_ratio = hl_area / total_area if total_area > 0 else 0
    hl_brightness = np.mean(v_channel[hl_mask]) if hl_area > 0 else 0

    # ---------- 2. 纹理特征 (灰度共生矩阵 对比度) ----------
    # 为避免零矩阵，取 gray 中物体区域，并缩放到0-255
    obj_gray = gray[mask_bool].reshape(-1,1).astype(np.uint8)
    if len(obj_gray) < 100:
        texture_contrast = 0
    else:
        # 计算 GLCM (距离1，角度0)
        glcm = cv2.createGLCM(obj_gray, [1], [0], levels=256, symmetric=True, normed=True)
        # OpenCV 的 GLCM 对比度计算需要转换为概率矩阵，用 skimage 简单替代
        # 为不依赖 skimage，手动计算对比度: sum_{i,j} (i-j)^2 * P(i,j)
        # 简化：直接使用局部二值模式方差或灰度共生矩阵，这里用灰度差分统计（快速）
        # 方法：计算相邻像素差绝对值，取均值作为纹理强度
        gray_roi = gray[mask>0].copy()
        # 获取边界周围像素的差分（简化：使用 Sobel 梯度的标准差作为纹理度量）
        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
        grad_mag = np.sqrt(sobelx**2 + sobely**2)
        grad_mag_obj = grad_mag[mask_bool]
        # 纹理对比度用梯度幅值的变异系数（标准差/均值）或者直接用标准差
        if np.mean(grad_mag_obj) > 0:
            texture_contrast = np.std(grad_mag_obj) / np.mean(grad_mag_obj)
        else:
            texture_contrast = 0
        # 放大便于阈值比较
        texture_contrast *= 100

    # ---------- 3. 梯度特征 ----------
    grad_max = np.max(grad_mag_obj) if len(grad_mag_obj) > 0 else 0
    grad_mean = np.mean(grad_mag_obj) if len(grad_mag_obj) > 0 else 0

    # ---------- 4. 颜色特征 ----------
    hue = hsv[:,:,0][mask_bool].astype(np.float32)
    sat = hsv[:,:,1][mask_bool].astype(np.float32)
    hue_mean = np.mean(hue) if len(hue) > 0 else 0
    sat_mean = np.mean(sat) if len(sat) > 0 else 0

    features = {
        'hl_ratio': hl_ratio,
        'hl_brightness': hl_brightness,
        'texture_contrast': texture_contrast,
        'grad_max': grad_max,
        'grad_mean': grad_mean,
        'hue_mean': hue_mean,
        'sat_mean': sat_mean
    }
    return features

def classify_by_vote(features):
    """
    根据特征投票决定金属(True)或塑料(False)
    """
    score = 0  # 金属得分
    # 1. 高光面积占比：越小越金属
    if features['hl_ratio'] < MIN_HL_AREA_RATIO:
        score += WEIGHT_HL_RATIO
    elif features['hl_ratio'] > MAX_HL_AREA_RATIO:
        score -= WEIGHT_HL_RATIO
    # 2. 高光亮度：越大越金属
    if features['hl_brightness'] > HL_BRIGHTNESS_TH:
        score += WEIGHT_HL_BRIGHT
    elif features['hl_brightness'] < HL_BRIGHTNESS_TH - 30:
        score -= WEIGHT_HL_BRIGHT
    # 3. 纹理对比度：越大越金属
    if features['texture_contrast'] > TEXTURE_METAL_TH:
        score += WEIGHT_TEXTURE
    else:
        score -= WEIGHT_TEXTURE
    # 4. 梯度最大值：越大越金属
    if features['grad_max'] > GRAD_MAX_METAL_TH:
        score += WEIGHT_GRAD_MAX
    else:
        score -= WEIGHT_GRAD_MAX
    # 5. 梯度均值：越大越金属
    if features['grad_mean'] > GRAD_MEAN_METAL_TH:
        score += WEIGHT_GRAD_MEAN
    else:
        score -= WEIGHT_GRAD_MEAN
    # 6. 色调：低色调为金属（冷色）
    if features['hue_mean'] < HUE_METAL_MAX:
        score += WEIGHT_HUE
    else:
        score -= WEIGHT_HUE
    # 7. 饱和度：低饱和度为金属
    if features['sat_mean'] < SAT_METAL_MAX:
        score += WEIGHT_SAT
    else:
        score -= WEIGHT_SAT

    is_metal = score >= METAL_VOTE_TH
    return is_metal, score

def draw_result(frame, mask, is_metal, score, features):
    """在图像上绘制结果和特征信息"""
    # 绘制物体轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(frame, contours, -1, (0, 255, 0), 2)
    # 标签和置信度（将得分归一化为0-100%显示）
    label = "METAL" if is_metal else "PLASTIC"
    # 得分范围大约在 -10 ~ 15 之间，映射到 0-100
    conf = min(100, max(0, (score + 10) / 25 * 100))
    text = f"{label} (vote={score:.1f}, conf={conf:.0f}%)"
    cv2.putText(frame, text, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    # 显示关键特征值
    info = f"HLr:{features['hl_ratio']:.2f} HLb:{features['hl_brightness']:.0f} Tex:{features['texture_contrast']:.1f}"
    cv2.putText(frame, info, (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
    info2 = f"Gmax:{features['grad_max']:.0f} Gmean:{features['grad_mean']:.1f} H:{features['hue_mean']:.0f} S:{features['sat_mean']:.0f}"
    cv2.putText(frame, info2, (10, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
    return frame

def main():
    # 打开摄像头
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头")
        return

    print("=== 金属/塑料实时区分系统（纯传统视觉）===")
    print("操作说明：")
    print("  1. 按 'b' 拍摄当前画面作为背景（请确保无物体）")
    print("  2. 将物体放入画面，按 'c' 开始实时识别")
    print("  3. 按 'q' 退出")
    background = None
    run_detection = False

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        display = frame.copy()

        if background is None:
            cv2.putText(display, "Press 'b' to set background (no object)", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        else:
            cv2.putText(display, "Background set. Press 'c' to detect", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        if run_detection and background is not None:
            mask = segment_object(frame, background)
            if np.sum(mask) > MIN_OBJECT_AREA:
                features = compute_features(frame, mask)
                if features:
                    is_metal, score = classify_by_vote(features)
                    display = draw_result(display, mask, is_metal, score, features)
                else:
                    cv2.putText(display, "Feature extraction failed", (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            else:
                cv2.putText(display, "No object detected", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        cv2.imshow("Metal vs Plastic", display)
        key = cv2.waitKey(10) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('b'):
            # 拍摄背景（需要画面中无物体）
            background = frame.copy()
            run_detection = False
            print("背景已设置")
        elif key == ord('c'):
            if background is not None:
                run_detection = not run_detection
                print("识别开关:", "开启" if run_detection else "关闭")
            else:
                print("请先按 'b' 设置背景")

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()