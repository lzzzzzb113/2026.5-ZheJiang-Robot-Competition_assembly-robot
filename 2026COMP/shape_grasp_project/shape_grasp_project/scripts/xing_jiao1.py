"""
【轮廓标注版】YOLO形状检测 + 轮廓精准绘制 + 极精角度计算
功能：用YOLO检测物块位置，在OTSU二值图中提取物块真实轮廓，
      直接以轮廓标注方式绘制（非矩形框），并计算旋转角度与抓取关键点。
运行：python contour_angle_demo.py
依赖：runs/5182.pt（形状模型）
"""
import cv2
import torch
import numpy as np
import math
import itertools
from ultralytics import YOLO
import os
import sys

# ==================== 配置 ====================
SHAPE_MODEL_PATH = "shape_grasp_project/runs/5184.pt"
CAMERA_ID = 1
CONF_THRESHOLD = 0.25
USE_GPU = True
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# 形状显示颜色
SHAPE_COLOR = {
    'san': (0, 255, 0),
    'ti': (255, 0, 0),
    'yuan': (0, 255, 255),
    'si': (0, 165, 255)
}

# ==================== 工具：检测框去重 ====================
def is_same_object(boxA, boxB, overlap_thresh=0.4):
    xA = max(boxA[0], boxB[0])
    yA = max(boxA[1], boxB[1])
    xB = min(boxA[2], boxB[2])
    yB = min(boxA[3], boxB[3])
    interArea = max(0, xB - xA) * max(0, yB - yA)
    if interArea == 0:
        return False
    boxAArea = (boxA[2] - boxA[0]) * (boxA[3] - boxA[1])
    boxBArea = (boxB[2] - boxB[0]) * (boxB[3] - boxB[1])
    iom = interArea / float(min(boxAArea, boxBArea))
    return iom > overlap_thresh

def filter_smallest_boxes(boxes, classes, confs):
    if len(boxes) == 0:
        return [], [], []
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

# ==================== 轮廓提取（抗边缘干扰） ====================
def extract_clean_contour(binary, x1, y1, x2, y2, shape_type, padding=3):
    """
    在二值图的YOLO检测区域内，通过形态学拓扑剥离找到真实物块轮廓，
    返回全局坐标下的干净轮廓点集 (N,1,2)。
    若失败返回 None。
    """
    h, w = binary.shape
    x1_p = max(0, x1 - padding)
    y1_p = max(0, y1 - padding)
    x2_p = min(w, x2 + padding)
    y2_p = min(h, y2 + padding)
    if x2_p - x1_p < 5 or y2_p - y1_p < 5:
        return None

    roi = binary[y1_p:y2_p, x1_p:x2_p].copy()
    h_roi, w_roi = roi.shape

    # ---- 拓扑剥离：腐蚀 -> 找中心连通域 -> 膨胀 -> 与原图取交 ----
    kernel_size = max(3, int(min(w_roi, h_roi) * 0.08))
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    eroded = cv2.erode(roi, kernel, iterations=1)

    # 选择最靠近ROI中心的连通域（必为物块主体）
    cnts_eroded, _ = cv2.findContours(eroded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best_cnt = None
    if cnts_eroded:
        cx_r, cy_r = w_roi / 2.0, h_roi / 2.0
        min_dist = float('inf')
        for c in cnts_eroded:
            M = cv2.moments(c)
            if M['m00'] == 0:
                continue
            ccx = M['m10'] / M['m00']
            ccy = M['m01'] / M['m00']
            dist = (ccx - cx_r) ** 2 + (ccy - cy_r) ** 2
            if dist < min_dist:
                min_dist = dist
                best_cnt = c

    if best_cnt is None:
        # 退化：直接用原ROI找最大轮廓
        cnts_raw, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if cnts_raw:
            best_cnt = max(cnts_raw, key=cv2.contourArea)
        else:
            return None

    # 构建干净掩膜并膨胀回原始尺寸
    clean_mask = np.zeros_like(roi)
    cv2.drawContours(clean_mask, [best_cnt], -1, 255, -1)
    dilate_kernel = np.ones((kernel_size + 2, kernel_size + 2), np.uint8)
    clean_mask = cv2.dilate(clean_mask, dilate_kernel, iterations=1)
    roi = cv2.bitwise_and(roi, clean_mask)

    # 一刀切断最外圈残留边缘
    cv2.rectangle(roi, (0, 0), (w_roi - 1, h_roi - 1), 0, 1)

    # 提取最终轮廓
    cnts_final, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts_final:
        return None
    final_cnt = max(cnts_final, key=cv2.contourArea)

    # 几何净化：对三角形/梯形做多边形逼近
    if shape_type in ['san', 'ti']:
        epsilon = 0.03 * cv2.arcLength(final_cnt, True)
        approx = cv2.approxPolyDP(final_cnt, epsilon, True)
        if len(approx) >= 3:
            final_cnt = approx

    # 坐标还原到全局
    global_cnt = final_cnt + np.array([[[x1_p, y1_p]]])
    return global_cnt

# ==================== 极精角度计算 ====================
def get_robust_angle_and_box(cnt, shape_type):
    rect = cv2.minAreaRect(cnt)
    (cx, cy), (w, h), theta = rect
    box = cv2.boxPoints(rect).astype(np.int32)

    if shape_type == 'san':
        x, y, bw, bh = cv2.boundingRect(cnt)
        shifted_cnt = cnt - np.array([[[x, y]]])
        mask_shape = (bh + 10, bw + 10)
        orig_mask = np.zeros(mask_shape, dtype=np.uint8)
        cv2.drawContours(orig_mask, [shifted_cnt], -1, 255, -1)

        hull = cv2.convexHull(shifted_cnt)
        epsilon = 0.015 * cv2.arcLength(hull, True)
        approx = cv2.approxPolyDP(hull, epsilon, True)
        pts = approx.reshape(-1, 2)
        best_triangle = None
        best_iou = 0.0
        if len(pts) >= 3:
            if len(pts) > 8: pts = pts[:8]
            for combo in itertools.combinations(pts, 3):
                tri_pts = np.array(combo, dtype=np.int32)
                tri_mask = np.zeros(mask_shape, dtype=np.uint8)
                cv2.drawContours(tri_mask, [tri_pts], -1, 255, -1)
                intersection = cv2.bitwise_and(orig_mask, tri_mask)
                area_inter = cv2.countNonZero(intersection)
                area_union = cv2.countNonZero(orig_mask) + cv2.countNonZero(tri_mask) - area_inter
                if area_union > 0:
                    iou = area_inter / float(area_union)
                    if iou > best_iou:
                        best_iou = iou
                        best_triangle = tri_pts
        if best_triangle is not None:
            triangle = best_triangle + np.array([x, y])
            triangle = np.expand_dims(triangle, axis=1)
        else:
            _, triangle = cv2.minEnclosingTriangle(cnt)
        if triangle is not None and len(triangle) == 3:
            pts = triangle.reshape(3, 2)
            left_idx = np.argmin(pts[:, 0])
            p_left = pts[left_idx]
            p1 = pts[(left_idx + 1) % 3]
            p2 = pts[(left_idx + 2) % 3]
            dx1, dy1 = p1[0] - p_left[0], p1[1] - p_left[1]
            angle1 = math.degrees(math.atan2(abs(dx1), abs(dy1))) if (dx1 or dy1) else 90
            dx2, dy2 = p2[0] - p_left[0], p2[1] - p_left[1]
            angle2 = math.degrees(math.atan2(abs(dx2), abs(dy2))) if (dx2 or dy2) else 90
            if angle1 < angle2:
                best_angle = angle1
                best_edge_pts = (tuple(map(int, p_left)), tuple(map(int, p1)))
                opp_pt = tuple(map(int, p2))
                best_dy = dy1
            else:
                best_angle = angle2
                best_edge_pts = (tuple(map(int, p_left)), tuple(map(int, p2)))
                opp_pt = tuple(map(int, p1))
                best_dy = dy2
            if best_dy < 0:
                best_angle += 180.0
            return best_angle, box, best_edge_pts, opp_pt
        return 0.0, box, None, None

    elif shape_type == 'ti':
        M = cv2.moments(cnt)
        if M['m00'] == 0:
            return 0.0, box, None, None
        mcx = M['m10'] / M['m00']
        mcy = M['m01'] / M['m00']
        vx = cx - mcx
        vy = cy - mcy
        v1x, v1y = box[1][0] - box[0][0], box[1][1] - box[0][1]
        len1 = math.hypot(v1x, v1y)
        dir1x, dir1y = (v1x / len1, v1y / len1) if len1 > 0 else (0, 0)
        v2x, v2y = box[2][0] - box[1][0], box[2][1] - box[1][1]
        len2 = math.hypot(v2x, v2y)
        dir2x, dir2y = (v2x / len2, v2y / len2) if len2 > 0 else (0, 0)
        dot1, dot2 = abs(vx * dir1x + vy * dir1y), abs(vx * dir2x + vy * dir2y)
        if math.hypot(vx, vy) < 1.0:
            sym_dir, sym_len = (dir1x, dir1y, len1) if len1 < len2 else (dir2x, dir2y, len2)
        else:
            sym_dir, sym_len = (dir1x, dir1y, len1) if dot1 > dot2 else (dir2x, dir2y, len2)
            if vx * sym_dir[0] + vy * sym_dir[1] < 0:
                sym_dir = (-sym_dir[0], -sym_dir[1])
        pt1 = (int(cx - sym_dir[0] * sym_len / 2), int(cy - sym_dir[1] * sym_len / 2))
        pt2 = (int(cx + sym_dir[0] * sym_len / 2), int(cy + sym_dir[1] * sym_len / 2))
        best_angle = math.degrees(math.atan2(sym_dir[1], sym_dir[0])) % 360
        return best_angle, box, (pt1, pt2), (int(cx), int(cy))

    else:  # 正方形/圆形
        max_len = 0
        best_angle = 0
        for i in range(4):
            p1 = box[i]
            p2 = box[(i + 1) % 4]
            length = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
            if length > max_len:
                max_len = length
                best_angle = math.degrees(math.atan2(p2[1] - p1[1], p2[0] - p1[0]))
        if best_angle < 0: best_angle += 180
        if best_angle >= 180: best_angle -= 180
        return best_angle, box, None, None

# ==================== 主循环 ====================
def main():
    if not os.path.exists(SHAPE_MODEL_PATH):
        print(f"[ERROR] 模型文件不存在: {SHAPE_MODEL_PATH}")
        return

    device = 'cuda:0' if USE_GPU and torch.cuda.is_available() else 'cpu'
    print(f"[INFO] 设备: {device}")
    model = YOLO(SHAPE_MODEL_PATH)
    model.to(device)

    cap = cv2.VideoCapture(CAMERA_ID, cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    print("[INFO] 轮廓标注+角度演示，按 q 退出")
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        results = model(frame, conf=CONF_THRESHOLD, verbose=False)[0]
        boxes_xyxy = []
        classes = []
        confs = []
        if hasattr(results, 'obb') and results.obb is not None:
            boxes_xyxy = results.obb.xyxy.cpu().numpy()
            classes = results.obb.cls.cpu().numpy()
            confs = results.obb.conf.cpu().numpy()
        elif hasattr(results, 'boxes') and results.boxes is not None:
            boxes_xyxy = results.boxes.xyxy.cpu().numpy()
            classes = results.boxes.cls.cpu().numpy()
            confs = results.boxes.conf.cpu().numpy()

        names = model.names
        boxes_xyxy, classes, confs = filter_smallest_boxes(boxes_xyxy, classes, confs)

        for i in range(len(boxes_xyxy)):
            x1, y1, x2, y2 = map(int, boxes_xyxy[i])
            cls_id = int(classes[i])
            conf = float(confs[i])
            shape_class = names[cls_id]
            color = SHAPE_COLOR.get(shape_class, (255, 255, 255))

            # 提取精确轮廓
            cnt = extract_clean_contour(binary, x1, y1, x2, y2, shape_class)
            if cnt is None or len(cnt) < 3:
                # 降级：仍绘制YOLO矩形
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame, f"{shape_class} {conf:.2f}", (x1, y1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                continue

            # 绘制轮廓（轮廓标注）
            cv2.drawContours(frame, [cnt], -1, color, 2)

            # 计算角度
            if shape_class in ['san', 'ti', 'si']:
                angle, box, edge, opp = get_robust_angle_and_box(cnt, shape_class)
                angle = angle % 360

                # 绘制最小外接矩形（可选，用半透明虚线？这里简化为黄色细线）
                if box is not None:
                    cv2.polylines(frame, [box], True, (0, 255, 255), 1)

                # 绘制角度箭头和对顶点
                if edge:
                    cv2.arrowedLine(frame, edge[0], edge[1], (0, 0, 255), 3, tipLength=0.2)
                if opp:
                    cv2.circle(frame, opp, 5, (0, 255, 255), -1)

                # 三角形额外绘制左锚点垂线
                if shape_class == 'san' and edge:
                    anchor_x = edge[0][0]
                    top_y = max(0, y1 - 20)
                    bot_y = min(FRAME_HEIGHT, y2 + 20)
                    cv2.line(frame, (anchor_x, top_y), (anchor_x, bot_y), (255, 0, 255), 2)

                label = f"{shape_class} {conf:.2f} Ang:{angle:.1f}"
            else:  # 圆形
                label = f"yuan {conf:.2f}"

            # 显示标签（位置取轮廓最上方点）
            cy, _ = cnt[:, 0, 1].min(), cnt[:, 0, 0].min()
            cv2.putText(frame, label, (int(cnt[:, 0, 0].min()), int(cnt[:, 0, 1].min()) - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        cv2.imshow("Contour-based Shape + Angle", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()