"""
【组合模块】形状检测 + 框内精确角度计算
功能：YOLO 形状模型检测 san/ti/yuan/si，对非圆形物块利用传统视觉抗干扰算法
       在精确检测框内计算精准旋转角度、抓取中心点和对顶点。
运行：python shape_and_angle_combined.py
依赖：runs/5182.pt（形状模型，需提前训练好）
"""
import cv2
import torch
import numpy as np
import math
import itertools
from ultralytics import YOLO
import sys
import os

# ==================== 配置区 ====================
SHAPE_MODEL_PATH = "shape_grasp_project/runs/5184.pt"          # 形状模型路径
CAMERA_ID = 1                              # 摄像头ID
CONF_THRESHOLD = 0.25                      # YOLO置信度阈值
USE_GPU = True                             # 是否使用GPU
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# 形状对应的显示颜色
SHAPE_COLOR = {
    'san': (0, 255, 0),    # 绿
    'ti':  (255, 0, 0),    # 蓝
    'yuan':(0, 255, 255),  # 黄
    'si':  (0, 165, 255)   # 橙
}

# ==================== 工具函数：检测框去重 ====================
def is_same_object(boxA, boxB, overlap_thresh=0.4):
    """判断两个框是否属于同一物体（交并比变体IoM）"""
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
    """去重：对于嵌套的检测框，仅保留面积最小的那个"""
    if len(boxes) == 0:
        return [], [], []
    areas = [(b[2] - b[0]) * (b[3] - b[1]) for b in boxes]
    sorted_indices = np.argsort(areas)
    keep_boxes, keep_classes, keep_confs = [], [], []
    for idx in sorted_indices:
        boxA = boxes[idx]
        # 检查是否与已保留的框高度重合
        if not any(is_same_object(boxA, kb) for kb in keep_boxes):
            keep_boxes.append(boxA)
            keep_classes.append(classes[idx])
            keep_confs.append(confs[idx])
    return keep_boxes, keep_classes, keep_confs

# ==================== 极精角度计算（纯几何） ====================
def get_robust_angle_and_box(cnt, shape_type):
    """
    根据轮廓计算旋转角度、最小外接矩形框、最长边端点、对顶点
    对三角形使用 IoU 最优拟合，避免边缘反光干扰
    """
    rect = cv2.minAreaRect(cnt)
    (cx, cy), (w, h), theta = rect
    box = cv2.boxPoints(rect).astype(np.int32)

    if shape_type == 'san':  # 三角形
        x, y, bw, bh = cv2.boundingRect(cnt)
        shifted_cnt = cnt - np.array([[[x, y]]])
        mask_shape = (bh + 10, bw + 10)
        orig_mask = np.zeros(mask_shape, dtype=np.uint8)
        cv2.drawContours(orig_mask, [shifted_cnt], -1, 255, -1)

        # 提取凸包顶点
        hull = cv2.convexHull(shifted_cnt)
        epsilon = 0.015 * cv2.arcLength(hull, True)
        approx = cv2.approxPolyDP(hull, epsilon, True)
        pts = approx.reshape(-1, 2)

        best_triangle = None
        best_iou = 0.0
        if len(pts) >= 3:
            if len(pts) > 8:  # 防止算力爆炸
                pts = pts[:8]
            for combo in itertools.combinations(pts, 3):
                tri_pts = np.array(combo, dtype=np.int32)
                tri_mask = np.zeros(mask_shape, dtype=np.uint8)
                cv2.drawContours(tri_mask, [tri_pts], -1, 255, -1)

                intersection = cv2.bitwise_and(orig_mask, tri_mask)
                area_inter = cv2.countNonZero(intersection)
                area_union = (cv2.countNonZero(orig_mask) +
                              cv2.countNonZero(tri_mask) - area_inter)
                if area_union > 0:
                    iou = area_inter / float(area_union)
                    if iou > best_iou:
                        best_iou = iou
                        best_triangle = tri_pts

        if best_triangle is not None:
            triangle = best_triangle + np.array([x, y])
            triangle = np.expand_dims(triangle, axis=1)
        else:
            retval, triangle = cv2.minEnclosingTriangle(cnt)

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
        else:
            return 0.0, box, None, None

    elif shape_type == 'ti':  # 梯形
        M = cv2.moments(cnt)
        if M['m00'] == 0:
            return 0.0, box, None, None
        mcx = M['m10'] / M['m00']
        mcy = M['m01'] / M['m00']
        vx = cx - mcx
        vy = cy - mcy

        # 选择与形心偏移方向最一致的边
        v1x, v1y = box[1][0] - box[0][0], box[1][1] - box[0][1]
        len1 = math.hypot(v1x, v1y)
        dir1x, dir1y = (v1x / len1, v1y / len1) if len1 > 0 else (0, 0)

        v2x, v2y = box[2][0] - box[1][0], box[2][1] - box[1][1]
        len2 = math.hypot(v2x, v2y)
        dir2x, dir2y = (v2x / len2, v2y / len2) if len2 > 0 else (0, 0)

        dot1 = abs(vx * dir1x + vy * dir1y)
        dot2 = abs(vx * dir2x + vy * dir2y)

        if math.hypot(vx, vy) < 1.0:  # 形心几乎在中心，取短边
            if len1 < len2:
                sym_dir, sym_len = (dir1x, dir1y), len1
            else:
                sym_dir, sym_len = (dir2x, dir2y), len2
        else:
            if dot1 > dot2:
                sym_dir, sym_len = (dir1x, dir1y), len1
            else:
                sym_dir, sym_len = (dir2x, dir2y), len2
            if vx * sym_dir[0] + vy * sym_dir[1] < 0:
                sym_dir = (-sym_dir[0], -sym_dir[1])

        pt1 = (int(cx - sym_dir[0] * sym_len / 2.0),
               int(cy - sym_dir[1] * sym_len / 2.0))
        pt2 = (int(cx + sym_dir[0] * sym_len / 2.0),
               int(cy + sym_dir[1] * sym_len / 2.0))
        best_edge_pts = (pt1, pt2)
        opp_pt = (int(cx), int(cy))
        best_angle = math.degrees(math.atan2(sym_dir[1], sym_dir[0]))
        best_angle = (best_angle + 360) % 360
        return best_angle, box, best_edge_pts, opp_pt

    else:  # 正方形、圆形：取最长边角度
        max_len = 0
        best_angle = 0
        for i in range(4):
            p1 = box[i]
            p2 = box[(i + 1) % 4]
            length = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
            if length > max_len:
                max_len = length
                best_angle = math.degrees(math.atan2(p2[1] - p1[1], p2[0] - p1[0]))
        if best_angle < 0:
            best_angle += 180
        if best_angle >= 180:
            best_angle -= 180
        return best_angle, box, None, None

# ==================== ROI抗边缘干扰几何处理器 ====================
def process_roi_geometry(frame_thresh, x1, y1, x2, y2, shape_type):
    """
    在二值化图像上，裁剪ROI，利用形态学拓扑剥离切断边缘粘连，
    再提取精确轮廓，最终调用 get_robust_angle_and_box 计算角度。
    """
    h_frame, w_frame = frame_thresh.shape

    # 极小外扩（仅为兜底）
    margin = 2
    rx1 = max(0, int(x1) - margin)
    ry1 = max(0, int(y1) - margin)
    rx2 = min(w_frame, int(x2) + margin)
    ry2 = min(h_frame, int(y2) + margin)

    if rx2 - rx1 < 5 or ry2 - ry1 < 5:
        return 0.0, None, None, None

    roi_thresh = frame_thresh[ry1:ry2, rx1:rx2].copy()
    h_roi, w_roi = roi_thresh.shape

    # ---- 形态学拓扑剥离：切断与边缘金属的粘连 ----
    kernel_size = max(3, int(min(w_roi, h_roi) * 0.08))
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    eroded = cv2.erode(roi_thresh, kernel, iterations=1)

    contours_eroded, _ = cv2.findContours(eroded, cv2.RETR_EXTERNAL,
                                          cv2.CHAIN_APPROX_SIMPLE)
    best_cnt = None
    if contours_eroded:
        cx_roi, cy_roi = w_roi / 2.0, h_roi / 2.0
        min_dist = float('inf')
        for c in contours_eroded:
            M = cv2.moments(c)
            if M['m00'] == 0:
                continue
            ccx = M['m10'] / M['m00']
            ccy = M['m01'] / M['m00']
            dist = (ccx - cx_roi) ** 2 + (ccy - cy_roi) ** 2
            if dist < min_dist:
                min_dist = dist
                best_cnt = c

    if best_cnt is not None:
        clean_mask = np.zeros_like(roi_thresh)
        cv2.drawContours(clean_mask, [best_cnt], -1, 255, -1)
        dilate_kernel = np.ones((kernel_size + 2, kernel_size + 2), np.uint8)
        clean_mask = cv2.dilate(clean_mask, dilate_kernel, iterations=1)
        roi_thresh = cv2.bitwise_and(roi_thresh, clean_mask)

    # 一刀切断最外圈残留边缘
    cv2.rectangle(roi_thresh, (0, 0), (w_roi - 1, h_roi - 1), 0, 1)

    contours, _ = cv2.findContours(roi_thresh, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return 0.0, None, None, None

    cnt = max(contours, key=cv2.contourArea)

    # 几何净化（对 san/ti 进一步多边形逼近）
    if shape_type in ['san', 'ti']:
        epsilon = 0.035 * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)
        if len(approx) >= 3:
            cnt = approx

    precise_angle, box_local, best_edge_local, opp_pt_local = \
        get_robust_angle_and_box(cnt, shape_type)

    # 坐标还原到全局图像
    box_global = box_local + np.array([rx1, ry1])

    best_edge_global = None
    opp_pt_global = None
    if best_edge_local is not None:
        p1 = tuple(best_edge_local[0] + np.array([rx1, ry1]))
        p2 = tuple(best_edge_local[1] + np.array([rx1, ry1]))
        best_edge_global = (p1, p2)
    if opp_pt_local is not None:
        opp_pt_global = tuple(opp_pt_local + np.array([rx1, ry1]))

    return precise_angle, box_global, best_edge_global, opp_pt_global

# ==================== 主程序 ====================
def main():
    if not os.path.exists(SHAPE_MODEL_PATH):
        print(f"[ERROR] 模型文件不存在: {SHAPE_MODEL_PATH}")
        return

    device = 'cuda:0' if USE_GPU and torch.cuda.is_available() else 'cpu'
    print(f"[INFO] 使用设备: {device}")
    shape_model = YOLO(SHAPE_MODEL_PATH)
    shape_model.to(device)

    cap = cv2.VideoCapture(CAMERA_ID, cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    print("[INFO] 按 'q' 退出程序")
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        # 生成全图 OTSU 二值图，用于后续角度计算
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, frame_thresh = cv2.threshold(gray, 0, 255,
                                        cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        # YOLO 形状检测
        results = shape_model(frame, conf=CONF_THRESHOLD, verbose=False)[0]
        boxes_xyxy_raw = []
        classes_raw = []
        confs_raw = []

        if hasattr(results, 'obb') and results.obb is not None:
            boxes_xyxy_raw = results.obb.xyxy.cpu().numpy()
            classes_raw = results.obb.cls.cpu().numpy()
            confs_raw = results.obb.conf.cpu().numpy()
        elif hasattr(results, 'boxes') and results.boxes is not None:
            boxes_xyxy_raw = results.boxes.xyxy.cpu().numpy()
            classes_raw = results.boxes.cls.cpu().numpy()
            confs_raw = results.boxes.conf.cpu().numpy()

        shape_names = shape_model.names

        # 去重（保留最小框）
        boxes_xyxy, classes, confs = filter_smallest_boxes(
            boxes_xyxy_raw, classes_raw, confs_raw
        )

        # 逐个处理检测框
        for i in range(len(boxes_xyxy)):
            x1, y1, x2, y2 = map(int, boxes_xyxy[i])
            cls_id = int(classes[i])
            conf = float(confs[i])
            shape_class = shape_names[cls_id]
            color = SHAPE_COLOR.get(shape_class, (255, 255, 255))

            # 绘制检测框
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

            # 圆形和正方形直接标记，三角形和梯形计算角度
            if shape_class in ['san', 'ti']:
                angle_deg, box_global, best_edge_global, opp_pt_global = \
                    process_roi_geometry(frame_thresh, x1, y1, x2, y2, shape_class)
                angle_deg = angle_deg % 360

                # 绘制最小外接矩形（虚线效果）
                if box_global is not None:
                    cv2.polylines(frame, [box_global], True, (0, 255, 255), 2)

                # 绘制角度箭头和关键点
                if best_edge_global is not None:
                    cv2.arrowedLine(frame, best_edge_global[0],
                                    best_edge_global[1], (0, 0, 255), 3,
                                    tipLength=0.2)
                    # 对三角形额外绘制左锚点竖线
                    if shape_class == 'san':
                        anchor_x = best_edge_global[0][0]
                        cv2.line(frame, (anchor_x, y1 - 20),
                                 (anchor_x, y2 + 20), (0, 255, 255), 2)
                        cv2.putText(frame, "Left Anchor",
                                    (anchor_x - 10, y1 - 25),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                                    (0, 255, 255), 1)

                if opp_pt_global is not None:
                    cv2.circle(frame, opp_pt_global, 5, (0, 255, 0), -1)

                # 标签显示形状、角度和置信度
                label = f"{shape_class} {conf:.2f} Ang:{angle_deg:.1f}"
                cv2.putText(frame, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            elif shape_class == 'si':  # 正方形也可计算角度（显示0~180）
                angle_deg, box_global, _, _ = process_roi_geometry(
                    frame_thresh, x1, y1, x2, y2, 'si')
                label = f"si {conf:.2f} Ang:{angle_deg:.1f}"
                cv2.putText(frame, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            else:  # 圆形，角度视为0
                label = f"yuan {conf:.2f}"
                cv2.putText(frame, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        cv2.imshow("Shape Detection + Precision Angle", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()