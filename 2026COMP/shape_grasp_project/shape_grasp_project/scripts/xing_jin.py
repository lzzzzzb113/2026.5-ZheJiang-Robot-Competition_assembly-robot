"""
【组合模块】形状检测 + 三角形/梯形材质区分
功能：
  1. YOLO形状模型检测所有物块（san/ti/yuan/si）
  2. 对三角形(san)和梯形(ti)使用材质模型进一步区分 plastic/metal
  3. 标签规则：
      - 金属梯形 -> mt
      - 塑料梯形 -> pt
      - 金属三角形 -> ms
      - 塑料三角形 -> ps
      - 正方形/圆形保持原标签
运行：python shape_material_classifier.py
依赖：runs/5182.pt (形状)  runs/5181m.pt (材质)
"""
import cv2
import torch
import numpy as np
from ultralytics import YOLO
import os

# ==================== 配置 ====================
SHAPE_MODEL_PATH = "shape_grasp_project/runs/5184.pt"
MATERIAL_MODEL_PATH = "shape_grasp_project/runs/5813m.pt"
CAMERA_ID = 1
CONF_THRESHOLD = 0.25
USE_GPU = True
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# 显示颜色
COLOR_MAP = {
    'ps': (255, 0, 255),   # 塑料三角
    'ms': (255, 0, 0),     # 金属三角
    'pt': (0, 255, 255),   # 塑料梯形
    'mt': (0, 165, 255),   # 金属梯形
    'si': (0, 255, 0),     # 正方形
    'yuan': (0, 255, 255), # 圆形 (这里用黄色，与其他不冲突)
    'san': (0, 255, 0),
    'ti': (255, 0, 0)
}

# ==================== 去重工具（同前） ====================
def is_same_object(boxA, boxB, overlap_thresh=0.4):
    xA = max(boxA[0], boxB[0]); yA = max(boxA[1], boxB[1])
    xB = min(boxA[2], boxB[2]); yB = min(boxA[3], boxB[3])
    interArea = max(0, xB - xA) * max(0, yB - yA)
    if interArea == 0: return False
    boxAArea = (boxA[2] - boxA[0]) * (boxA[3] - boxA[1])
    boxBArea = (boxB[2] - boxB[0]) * (boxB[3] - boxB[1])
    iom = interArea / float(min(boxAArea, boxBArea))
    return iom > overlap_thresh

def filter_smallest_boxes(boxes, classes, confs):
    if len(boxes) == 0: return [], [], []
    areas = [(b[2] - b[0]) * (b[3] - b[1]) for b in boxes]
    sorted_idx = np.argsort(areas)
    keep_boxes, keep_classes, keep_confs = [], [], []
    for idx in sorted_idx:
        boxA = boxes[idx]
        if not any(is_same_object(boxA, kb) for kb in keep_boxes):
            keep_boxes.append(boxA)
            keep_classes.append(classes[idx])
            keep_confs.append(confs[idx])
    return keep_boxes, keep_classes, keep_confs

# ==================== 材质识别子函数 ====================
def classify_material(roi_img, material_model, conf_threshold=0.25):
    """
    对裁剪的 ROI 图像进行材质识别，返回最佳材质名称 (plastic/metal) 和置信度。
    若无有效检测，返回 'Unknown', 0.0
    """
    results = material_model(roi_img, conf=conf_threshold, verbose=False)[0]
    best_label = 'Unknown'
    best_conf = 0.0
    if hasattr(results, 'boxes') and results.boxes is not None:
        for j in range(len(results.boxes)):
            conf = float(results.boxes.conf[j].cpu().numpy())
            if conf > best_conf:
                best_conf = conf
                cls_id = int(results.boxes.cls[j].cpu().numpy())
                best_label = material_model.names[cls_id]
    return best_label, best_conf

# ==================== 主程序 ====================
def main():
    if not os.path.exists(SHAPE_MODEL_PATH):
        print(f"[ERROR] 形状模型不存在: {SHAPE_MODEL_PATH}")
        return
    if not os.path.exists(MATERIAL_MODEL_PATH):
        print(f"[ERROR] 材质模型不存在: {MATERIAL_MODEL_PATH}")
        return

    device = 'cuda:0' if USE_GPU and torch.cuda.is_available() else 'cpu'
    print(f"[INFO] 设备: {device}")

    shape_model = YOLO(SHAPE_MODEL_PATH)
    material_model = YOLO(MATERIAL_MODEL_PATH)
    shape_model.to(device)
    material_model.to(device)

    cap = cv2.VideoCapture(CAMERA_ID, cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    print("[INFO] 按 'q' 退出")
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        # 形状检测
        shape_results = shape_model(frame, conf=CONF_THRESHOLD, verbose=False)[0]
        boxes_xyxy = []
        classes = []
        confs = []
        if hasattr(shape_results, 'obb') and shape_results.obb is not None:
            boxes_xyxy = shape_results.obb.xyxy.cpu().numpy()
            classes = shape_results.obb.cls.cpu().numpy()
            confs = shape_results.obb.conf.cpu().numpy()
        elif hasattr(shape_results, 'boxes') and shape_results.boxes is not None:
            boxes_xyxy = shape_results.boxes.xyxy.cpu().numpy()
            classes = shape_results.boxes.cls.cpu().numpy()
            confs = shape_results.boxes.conf.cpu().numpy()
        shape_names = shape_model.names

        # 去重
        boxes_xyxy, classes, confs = filter_smallest_boxes(boxes_xyxy, classes, confs)

        for i in range(len(boxes_xyxy)):
            x1, y1, x2, y2 = map(int, boxes_xyxy[i])
            cls_id = int(classes[i])
            conf = float(confs[i])
            shape_class = shape_names[cls_id]

            # 默认最终标签等于形状类别，颜色使用形状颜色
            final_tag = shape_class
            color = COLOR_MAP.get(shape_class, (255, 255, 255))

            # 如果是三角形或梯形，进一步材质识别
            if shape_class in ['san', 'ti']:
                # 裁剪 ROI（加一点 margin）
                margin = 2
                cx1 = max(0, x1 - margin)
                cy1 = max(0, y1 - margin)
                cx2 = min(frame.shape[1], x2 + margin)
                cy2 = min(frame.shape[0], y2 + margin)
                if cx2 > cx1 and cy2 > cy1:
                    roi = frame[cy1:cy2, cx1:cx2]
                    mat_label, mat_conf = classify_material(roi, material_model, CONF_THRESHOLD)

                    # 组合标签
                    if shape_class == 'san':
                        if mat_label == 'plastic':
                            final_tag = 'ps'
                        elif mat_label == 'metal':
                            final_tag = 'ms'
                    elif shape_class == 'ti':
                        if mat_label == 'plastic':
                            final_tag = 'pt'
                        elif mat_label == 'metal':
                            final_tag = 'mt'

                    color = COLOR_MAP.get(final_tag, (255, 255, 255))

            # 绘制矩形和标签
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            label = f"{final_tag} {conf:.2f}"
            # 如果材质识别成功，可附加材质置信度（可选）
            if shape_class in ['san', 'ti'] and final_tag != shape_class:
                label += f" mat:{mat_conf:.2f}"
            cv2.putText(frame, label, (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        cv2.imshow("Shape + Material Classifier", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()