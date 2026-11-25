"""
【组合模块 V7 (特征融合版)】形状检测 + 材质检测 + 高级串口通讯 + 转盘中断
功能变更：
1. 材质与形状特征融合：金属三角形 -> ms, 塑料三角形 -> ps, 金属梯形 -> mt, 塑料梯形 -> pt。
2. 画面与输出统一：不再单独显示/发送 san 或 ti，直接输出融合后的唯一 Tag。
3. 保持母体严格的输出格式及转盘微距 '7' 的高优中断逻辑。
"""
import cv2
import torch
import numpy as np
import math
import itertools
from ultralytics import YOLO
import sys
import os
import time
import serial
import serial.tools.list_ports
from dataclasses import dataclass
from typing import List, Tuple, Optional

# ==================== 配置区 ====================
SHAPE_MODEL_PATH = "shape_grasp_project/runs/5184.pt"          # 形状模型路径
MATERIAL_MODEL_PATH = "shape_grasp_project/runs/5183m.pt"   # 材质模型路径 (请替换为你的实际路径)
CAMERA_ID = 1                              
CONF_THRESHOLD = 0.25                      
USE_GPU = True                             
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# 形状与融合特征的显示颜色 (可按需自调)
SHAPE_COLOR = {
    'yuan': (0, 255, 255),   # 黄色
    'si':   (0, 165, 255),   # 橙色
    'ms':   (100, 255, 100), # 浅绿 (金属三角)
    'ps':   (0, 200, 0),     # 深绿 (塑料三角)
    'mt':   (100, 100, 255), # 浅蓝 (金属梯形)
    'pt':   (0, 0, 200),     # 深蓝 (塑料梯形)
    'san':  (0, 255, 0),     # 兜底色
    'ti':   (255, 0, 0)      # 兜底色
}

# ==================== 转盘微距定位专属配置 ====================
@dataclass
class ShapeResult:
    shape_type: str          
    color_name: str          
    center: Tuple[float, float]  
    bbox: Tuple[int, int, int, int]  
    contour_area: float      
    dist_to_center: float    

HSV_CONFIG = {
    "red":   {"low":  [0,   100, 100], "high": [10,  255, 255]},
    "red2":  {"low":  [160, 100, 100], "high": [179, 255, 255]}, 
    "blue":  {"low":  [90,  50,  50],  "high": [140, 255, 255]}
}
ALLOWED_COLORS = ["red", "blue"]
COLOR_BGR_MAP = {"red": (0, 0, 255), "blue": (255, 0, 0)}

# ==================== 串口通信配置区 ====================
BAUD_RATE = 115200                         

ASCII_STX = 0x02
ASCII_ETX = 0x03
ASCII_ACK = 0x06
ASCII_SYN = 0x16

class SerialManager:
    def __init__(self, baudrate):
        self.baudrate = baudrate
        self.ser = None
        self.last_reconnect_time = 0
        self.reconnect_interval = 2.0  

    def check_and_connect(self):
        if self.ser is not None and self.ser.is_open: return True
        current_time = time.time()
        if current_time - self.last_reconnect_time < self.reconnect_interval: return False
        self.last_reconnect_time = current_time
        ports = list(serial.tools.list_ports.comports())
        if not ports: return False
        for p in ports:
            try:
                self.ser = serial.Serial(p.device, self.baudrate, timeout=0.02)
                print(f"[INFO] >>> 成功连接到串口: {p.device} <<<")
                return True
            except serial.SerialException:
                continue
        return False

    def check_for_cmd_7(self):
        if not self.check_and_connect(): return False
        if self.ser.in_waiting > 0:
            try:
                data = self.ser.read(self.ser.in_waiting)
                if b'7' in data: return True
            except Exception: pass
        return False

    def send_sync(self, data_str, timeout=0.1):
        if not self.check_and_connect(): return False
        try:
            packet = bytearray()
            packet.append(ASCII_STX)
            packet.extend(data_str.encode('utf-8'))
            packet.append(ASCII_ETX)
            
            self.ser.reset_input_buffer()
            self.ser.write(packet)

            start_t = time.time()
            while time.time() - start_t < timeout:
                if self.ser.in_waiting > 0:
                    if self.ser.read(1) == bytes([ASCII_ACK]): return True
            return False
        except serial.SerialException:
            if self.ser: self.ser.close()
            self.ser = None  
            return False

    def close(self):
        if self.ser is not None and self.ser.is_open:
            self.ser.close()

# ==================== 转盘视觉处理核心函数 ====================
def apply_hsv_range(hsv: np.ndarray, low: List[int], high: List[int]) -> np.ndarray:
    return cv2.inRange(hsv, np.array(low, dtype=np.uint8), np.array(high, dtype=np.uint8))

def apply_color_mask(hsv: np.ndarray, color_name: str, config: dict) -> np.ndarray:
    cfg = config.get(color_name)
    if not cfg: return np.zeros(hsv.shape[:2], dtype=np.uint8)
    mask = apply_hsv_range(hsv, cfg["low"], cfg["high"])
    if color_name + "2" in config:
        mask = cv2.bitwise_or(mask, apply_hsv_range(hsv, config[color_name + "2"]["low"], config[color_name + "2"]["high"]))
    return mask

def compute_center(contour: np.ndarray) -> Tuple[float, float]:
    (cx, cy), radius = cv2.minEnclosingCircle(contour)
    return (float(cx), float(cy))

def find_best_single_slot(image: np.ndarray, min_area: int = 500) -> Optional[ShapeResult]:
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    img_h, img_w = image.shape[:2]
    img_cx, img_cy = img_w / 2.0, img_h / 2.0 
    best_slot = None
    min_distance = float('inf')

    for color_name, cfg in HSV_CONFIG.items():
        if color_name.endswith("2") or color_name not in ALLOWED_COLORS: continue
        mask = apply_color_mask(hsv, color_name, HSV_CONFIG)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            hull = cv2.convexHull(cnt)
            area = cv2.contourArea(hull)
            if area < min_area: continue
            x, y, bw, bh = cv2.boundingRect(hull)
            if x <= 5 or y <= 5 or (x + bw) >= img_w - 5 or (y + bh) >= img_h - 5: continue

            center = compute_center(hull)
            dist_to_cam_center = math.hypot(center[0] - img_cx, center[1] - img_cy)
            if dist_to_cam_center < min_distance:
                min_distance = dist_to_cam_center
                best_slot = ShapeResult("slot", color_name, center, (x, y, bw, bh), area, dist_to_cam_center)
    return best_slot

# ==================== 工具函数与几何处理器 ====================
def is_same_object(boxA, boxB, overlap_thresh=0.4):
    xA, yA = max(boxA[0], boxB[0]), max(boxA[1], boxB[1])
    xB, yB = min(boxA[2], boxB[2]), min(boxA[3], boxB[3])
    interArea = max(0, xB - xA) * max(0, yB - yA)
    if interArea == 0: return False
    boxAArea = (boxA[2] - boxA[0]) * (boxA[3] - boxA[1])
    boxBArea = (boxB[2] - boxB[0]) * (boxB[3] - boxB[1])
    return (interArea / float(min(boxAArea, boxBArea))) > overlap_thresh

def filter_smallest_boxes(boxes, classes, confs):
    if len(boxes) == 0: return [], [], []
    areas = [(b[2] - b[0]) * (b[3] - b[1]) for b in boxes]
    sorted_indices = np.argsort(areas)
    keep_boxes, keep_classes, keep_confs = [], [], []
    for idx in sorted_indices:
        boxA = boxes[idx]
        if not any(is_same_object(boxA, kb) for kb in keep_boxes):
            keep_boxes.append(boxA)
            keep_classes.append(classes[idx])
            keep_confs.append(confs[idx])
    return keep_boxes, keep_classes, keep_confs

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
        approx = cv2.approxPolyDP(hull, 0.015 * cv2.arcLength(hull, True), True)
        pts = approx.reshape(-1, 2)
        best_triangle, best_iou = None, 0.0

        if len(pts) >= 3:
            if len(pts) > 8: pts = pts[:8]
            for combo in itertools.combinations(pts, 3):
                tri_pts = np.array(combo, dtype=np.int32)
                tri_mask = np.zeros(mask_shape, dtype=np.uint8)
                cv2.drawContours(tri_mask, [tri_pts], -1, 255, -1)
                intersection = cv2.bitwise_and(orig_mask, tri_mask)
                area_inter = cv2.countNonZero(intersection)
                area_union = (cv2.countNonZero(orig_mask) + cv2.countNonZero(tri_mask) - area_inter)
                if area_union > 0:
                    iou = area_inter / float(area_union)
                    if iou > best_iou: best_iou, best_triangle = iou, tri_pts

        if best_triangle is not None:
            triangle = best_triangle + np.array([x, y])
            triangle = np.expand_dims(triangle, axis=1)
        else:
            _, triangle = cv2.minEnclosingTriangle(cnt)

        if triangle is not None and len(triangle) == 3:
            pts = triangle.reshape(3, 2)
            left_idx = np.argmin(pts[:, 0])
            p_left = pts[left_idx]
            p1, p2 = pts[(left_idx + 1) % 3], pts[(left_idx + 2) % 3]

            dx1, dy1 = p1[0] - p_left[0], p1[1] - p_left[1]
            angle1 = math.degrees(math.atan2(abs(dx1), abs(dy1))) if (dx1 or dy1) else 90
            dx2, dy2 = p2[0] - p_left[0], p2[1] - p_left[1]
            angle2 = math.degrees(math.atan2(abs(dx2), abs(dy2))) if (dx2 or dy2) else 90

            if angle1 < angle2: best_angle, best_edge_pts, opp_pt, best_dy = angle1, (tuple(map(int, p_left)), tuple(map(int, p1))), tuple(map(int, p2)), dy1
            else: best_angle, best_edge_pts, opp_pt, best_dy = angle2, (tuple(map(int, p_left)), tuple(map(int, p2))), tuple(map(int, p1)), dy2
            if best_dy < 0: best_angle += 180.0
            return best_angle, box, best_edge_pts, opp_pt
        else: return 0.0, box, None, None

    elif shape_type == 'ti': 
        hull = cv2.convexHull(cnt)
        approx = cv2.approxPolyDP(hull, 0.02 * cv2.arcLength(hull, True), True)
        if len(approx) == 4:
            pts = approx.reshape(4, 2)
            edges = []
            for i in range(4):
                p1, p2 = pts[i], pts[(i + 1) % 4]
                vx, vy = p2[0] - p1[0], p2[1] - p1[1]
                edges.append({'p1': p1, 'p2': p2, 'vx': vx, 'vy': vy, 'len': math.hypot(vx, vy)})

            def calc_parallel_score(eA, eB):
                if eA['len'] == 0 or eB['len'] == 0: return 0
                return abs((eA['vx']*eB['vx'] + eA['vy']*eB['vy']) / (eA['len'] * eB['len']))

            score02, score13 = calc_parallel_score(edges[0], edges[2]), calc_parallel_score(edges[1], edges[3])
            base1, base2 = (edges[0], edges[2]) if score02 > score13 else (edges[1], edges[3])
            long_base, short_base = (base1, base2) if base1['len'] > base2['len'] else (base2, base1)

            mid_long = (int((long_base['p1'][0] + long_base['p2'][0]) / 2), int((long_base['p1'][1] + long_base['p2'][1]) / 2))
            mid_short = (int((short_base['p1'][0] + short_base['p2'][0]) / 2), int((short_base['p1'][1] + short_base['p2'][1]) / 2))
            best_angle = (math.degrees(math.atan2(mid_short[1] - mid_long[1], mid_short[0] - mid_long[0])) + 360) % 360
            return best_angle, box, (mid_long, mid_short), None

        M = cv2.moments(cnt)
        if M['m00'] == 0: return 0.0, box, None, None
        mcx, mcy = M['m10'] / M['m00'], M['m01'] / M['m00']
        vx, vy = cx - mcx, cy - mcy
        if math.hypot(vx, vy) < 1.0: return 0.0, box, None, None

        best_angle = (math.degrees(math.atan2(vy, vx)) + 360) % 360
        arrow_len = min(w, h) * 0.8
        pt1 = (int(mcx), int(mcy))
        pt2 = (int(mcx + math.cos(math.radians(best_angle)) * arrow_len), int(mcy + math.sin(math.radians(best_angle)) * arrow_len))
        return best_angle, box, (pt1, pt2), None

    else: 
        max_len, best_angle = 0, 0
        for i in range(4):
            p1, p2 = box[i], box[(i + 1) % 4]
            length = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
            if length > max_len:
                max_len, best_angle = length, math.degrees(math.atan2(p2[1] - p1[1], p2[0] - p1[0]))
        if best_angle < 0: best_angle += 180
        if best_angle >= 180: best_angle -= 180
        return best_angle, box, None, None

def process_roi_geometry(frame_thresh, x1, y1, x2, y2, shape_type):
    h_frame, w_frame = frame_thresh.shape
    margin = 2
    rx1, ry1 = max(0, int(x1) - margin), max(0, int(y1) - margin)
    rx2, ry2 = min(w_frame, int(x2) + margin), min(h_frame, int(y2) + margin)
    if rx2 - rx1 < 5 or ry2 - ry1 < 5: return 0.0, None, None, None
    roi_thresh = frame_thresh[ry1:ry2, rx1:rx2].copy()
    h_roi, w_roi = roi_thresh.shape
    kernel_size = max(3, int(min(w_roi, h_roi) * 0.08))
    eroded = cv2.erode(roi_thresh, np.ones((kernel_size, kernel_size), np.uint8), iterations=1)
    contours_eroded, _ = cv2.findContours(eroded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best_cnt = None
    if contours_eroded:
        cx_roi, cy_roi = w_roi / 2.0, h_roi / 2.0
        min_dist = float('inf')
        for c in contours_eroded:
            M = cv2.moments(c)
            if M['m00'] == 0: continue
            dist = (M['m10'] / M['m00'] - cx_roi) ** 2 + (M['m01'] / M['m00'] - cy_roi) ** 2
            if dist < min_dist: min_dist, best_cnt = dist, c
    if best_cnt is not None:
        clean_mask = np.zeros_like(roi_thresh)
        cv2.drawContours(clean_mask, [best_cnt], -1, 255, -1)
        roi_thresh = cv2.bitwise_and(roi_thresh, cv2.dilate(clean_mask, np.ones((kernel_size + 2, kernel_size + 2), np.uint8), iterations=1))
    cv2.rectangle(roi_thresh, (0, 0), (w_roi - 1, h_roi - 1), 0, 1)
    contours, _ = cv2.findContours(roi_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours: return 0.0, None, None, None
    cnt = max(contours, key=cv2.contourArea)
    if shape_type in ['san', 'ti']:
        approx = cv2.approxPolyDP(cnt, 0.035 * cv2.arcLength(cnt, True), True)
        if len(approx) >= 3: cnt = approx

    precise_angle, box_local, best_edge_local, opp_pt_local = get_robust_angle_and_box(cnt, shape_type)
    box_global = box_local + np.array([rx1, ry1])
    best_edge_global = (tuple(best_edge_local[0] + np.array([rx1, ry1])), tuple(best_edge_local[1] + np.array([rx1, ry1]))) if best_edge_local else None
    opp_pt_global = tuple(opp_pt_local + np.array([rx1, ry1])) if opp_pt_local else None
    return precise_angle, box_global, best_edge_global, opp_pt_global


# ==================== 主程序 ====================
def main():
    if not os.path.exists(SHAPE_MODEL_PATH):
        print(f"[ERROR] 形状模型不存在: {SHAPE_MODEL_PATH}")
        return
    if not os.path.exists(MATERIAL_MODEL_PATH):
        print(f"[WARNING] 材质模型不存在: {MATERIAL_MODEL_PATH}，将跳过材质检测。")

    serial_mgr = SerialManager(BAUD_RATE)
    serial_mgr.check_and_connect()

    device = 'cuda:0' if USE_GPU and torch.cuda.is_available() else 'cpu'
    print(f"[INFO] 使用设备: {device}")
    
    shape_model = YOLO(SHAPE_MODEL_PATH)
    shape_model.to(device)
    
    material_model = None
    if os.path.exists(MATERIAL_MODEL_PATH):
        material_model = YOLO(MATERIAL_MODEL_PATH)
        material_model.to(device)

    cap = cv2.VideoCapture(CAMERA_ID, cv2.CAP_DSHOW)
    if not cap.isOpened(): cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    print("[INFO] 视觉系统运行中...按 'q' 退出")
    
    while True:
        ret, frame = cap.read()
        if not ret: continue
        h_frame, w_frame = frame.shape[:2]

        # =================================================================
        # 【拦截逻辑】：指令 '7' 优先响应转盘
        # =================================================================
        if serial_mgr.check_for_cmd_7():
            print("\n[CMD] 收到指令 '7'，拦截本帧执行【转盘空槽圆心定位】...")
            target = find_best_single_slot(frame, min_area=1000)
            
            if target:
                tcx, tcy = map(int, target.center)
                bx, by, bw, bh = target.bbox
                tag = target.color_name
                
                color = COLOR_BGR_MAP.get(tag, (0, 255, 0))
                cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), color, 3)
                cv2.circle(frame, (tcx, tcy), 6, (0, 0, 255), -1)
                cv2.drawMarker(frame, (tcx, tcy), (0, 0, 255), markerType=cv2.MARKER_TILTED_CROSS, markerSize=30, thickness=3)
                cv2.putText(frame, f"LOCKED: {tag.upper()} SLOT", (bx, max(20, by - 10)), cv2.FONT_HERSHEY_DUPLEX, 0.6, color, 2)
                
                data_str = f"[[{tcx},{tcy},{bw},{bh},{tag}:1.00:0.00]]"
                serial_mgr.send_sync(data_str, timeout=0.1)
            else:
                serial_mgr.send_sync("[]", timeout=0.1)
                cv2.putText(frame, "NO SLOT DETECTED", (20, 40), cv2.FONT_HERSHEY_DUPLEX, 0.8, (0, 0, 255), 2)

            cv2.imshow("Shape Detection + Precision Angle", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'): break
            continue

        # =================================================================
        # 【母体逻辑】：形状 -> 材质融合
        # =================================================================
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, frame_thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        results = shape_model(frame, conf=CONF_THRESHOLD, verbose=False)[0]
        
        boxes_xyxy_raw = results.obb.xyxy.cpu().numpy() if hasattr(results, 'obb') and results.obb is not None else (results.boxes.xyxy.cpu().numpy() if hasattr(results, 'boxes') and results.boxes is not None else [])
        classes_raw = results.obb.cls.cpu().numpy() if hasattr(results, 'obb') and results.obb is not None else (results.boxes.cls.cpu().numpy() if hasattr(results, 'boxes') and results.boxes is not None else [])
        confs_raw = results.obb.conf.cpu().numpy() if hasattr(results, 'obb') and results.obb is not None else (results.boxes.conf.cpu().numpy() if hasattr(results, 'boxes') and results.boxes is not None else [])

        shape_names = shape_model.names
        boxes_xyxy, classes, confs = filter_smallest_boxes(boxes_xyxy_raw, classes_raw, confs_raw)

        for i in range(len(boxes_xyxy)):
            x1, y1, x2, y2 = map(int, boxes_xyxy[i])
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w_frame, x2), min(h_frame, y2)
            
            cls_id = int(classes[i])
            conf = float(confs[i])
            
            original_shape = shape_names[cls_id]
            final_tag = original_shape  # 初始 tag 设为原形状

            # --- 材质推理与 Tag 融合逻辑 ---
            if original_shape in ['san', 'ti'] and material_model is not None and (x2 - x1) > 10 and (y2 - y1) > 10:
                roi = frame[y1:y2, x1:x2].copy()
                mat_results = material_model(roi, conf=CONF_THRESHOLD, verbose=False)[0]
                
                if hasattr(mat_results, 'boxes') and mat_results.boxes is not None and len(mat_results.boxes) > 0:
                    mat_cls_id = int(mat_results.boxes.cls[0].item())
                    # 注意：如果你的材质模型输出类名为 jinshu/suliao，请将下面双引号内的单词替换掉
                    material_name = material_model.names[mat_cls_id].lower() 
                    
                    if original_shape == 'san':
                        final_tag = 'ms' if 'metal' in material_name else 'ps'
                    elif original_shape == 'ti':
                        final_tag = 'mt' if 'metal' in material_name else 'pt'

            # 统一使用融合后的 final_tag 获取显示颜色
            color = SHAPE_COLOR.get(final_tag, (255, 255, 255))
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            
            angle_deg = 0.0

            # 注意：几何算法的底层依然必须通过 original_shape ('san'/'ti') 来区分不同的运算机制
            if original_shape in ['san', 'ti']:
                angle_deg, box_global, best_edge_global, opp_pt_global = \
                    process_roi_geometry(frame_thresh, x1, y1, x2, y2, original_shape)

                angle_deg = angle_deg % 360.0 

                if box_global is not None:
                    cv2.polylines(frame, [box_global], True, (0, 255, 255), 2)
                if best_edge_global is not None:
                    cv2.arrowedLine(frame, best_edge_global[0], best_edge_global[1], (0, 0, 255), 3, tipLength=0.2)
                    if original_shape == 'san':
                        anchor_x = best_edge_global[0][0]
                        cv2.line(frame, (anchor_x, y1 - 20), (anchor_x, y2 + 20), (0, 255, 255), 2)
                if opp_pt_global is not None:
                    cv2.circle(frame, opp_pt_global, 5, (0, 255, 0), -1)

            elif original_shape == 'si':
                angle_deg, box_global, _, _ = process_roi_geometry(frame_thresh, x1, y1, x2, y2, 'si')
                angle_deg = angle_deg % 90.0
                
            # --- 画面显示统一使用最终 Tag (不再单独显示 san/ti) ---
            label = f"{final_tag} {conf:.2f} Ang:{angle_deg:.1f}"
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # --- 严格遵循母体输出逻辑，但 tag 字段变为 ms/ps/mt/pt/yuan/si ---
            best = {
                'x': int((x1 + x2) / 2),
                'y': int((y1 + y2) / 2),
                'w': int(x2 - x1),
                'h': int(y2 - y1),
                'tag': final_tag,
                'conf': conf,
                'angle_norm': angle_deg
            }

            data_str = f"[{best['x']},{best['y']},{best['w']},{best['h']},{best['tag']}:{best['conf']:.4f}:{best['angle_norm']:.4f}]"
            success = serial_mgr.send_sync(data_str, timeout=0.1)
            
            status_text = "Serial: OK" if success else "Serial: ERR / Wait"
            status_color = (0, 255, 0) if success else (0, 0, 255)
            cv2.putText(frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)

        cv2.imshow("Shape Detection + Precision Angle", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    serial_mgr.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()