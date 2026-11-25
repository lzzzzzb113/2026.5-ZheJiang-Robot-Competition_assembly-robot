"""
【组合模块 V11 (主被动结合版 + 完整角度还原版 + 增强型串口架构)】
核心更新：
1. 继承 he15 所有视觉特性：视野广播、局部二值化、纯CV材质提取、精准角度还原。
2. 融入 serial_comf 强大的串口底座：支持 USB 串口自动寻找、SYN-ACK 握手协议防掉线防丢包。
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
import threading

# ==================== 配置区 ====================
SHAPE_MODEL_PATH = "shape_grasp_project/runs/5184.pt"          
MATERIAL_MODEL_PATH = "shape_grasp_project/runs/5183m.pt"   
CAMERA_ID = 1                              
CONF_THRESHOLD = 0.25                      
USE_GPU = True                             
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

CMD_MAP = {
    ord('1'): 'ps',   
    ord('2'): 'ms',   
    ord('3'): 'pt',   
    ord('4'): 'mt',   
    ord('5'): 'si',   
    ord('6'): 'yuan', 
}

SHAPE_COLOR = {
    'yuan': (0, 255, 255), 'si': (0, 165, 255), 
    'ms': (100, 255, 100), 'ps': (0, 200, 0),     
    'mt': (100, 100, 255), 'pt': (0, 0, 200),     
    'san': (0, 255, 0),    'ti': (255, 0, 0)      
}

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

# ==================== 增强型串口通信配置区 (融合 serial_comf) ====================
BAUD_RATE = 9600                        
ASCII_STX = 0x02
ASCII_ETX = 0x03
ASCII_ACK = 0x06
ASCII_SYN = 0x16

def auto_detect_serial_port(keyword="USB", silent=False):
    """自动寻找并匹配包含特定关键字的串口设备"""
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        if not silent: print("[ERROR] 未检测到任何可用串口！")
        return None

    if keyword:
        for port in ports:
            if keyword.lower() in port.description.lower():
                if not silent: print(f"[INFO] 自动匹配到串口: {port.device}")
                return port.device

    selected_port = ports[0].device
    if not silent: print(f"[INFO] 自动选择首个有效串口: {selected_port}")
    return selected_port

class SerialManager:
    def __init__(self, baudrate):
        self.baudrate = baudrate
        self.ser = None
        self.last_reconnect_time = 0
        self.reconnect_interval = 2.0  
        self.pending_cmd = None  

    def check_and_connect(self):
        if self.ser is not None and self.ser.is_open: return True
        if time.time() - self.last_reconnect_time < self.reconnect_interval: return False
        self.last_reconnect_time = time.time()
        
        # 使用 serial_comf 的自动匹配机制
        port_to_use = auto_detect_serial_port(keyword="USB", silent=True)
        if not port_to_use:
            ports = list(serial.tools.list_ports.comports())
            if ports: port_to_use = ports[0].device
            
        if port_to_use:
            try:
                self.ser = serial.Serial(port_to_use, self.baudrate, timeout=0.02)
                print(f"\n[INFO] >>> 成功建立底层通信连接: {port_to_use} @ {self.baudrate} <<<")
                return True
            except serial.SerialException: 
                pass
        return False

    def get_latest_command(self):
        if not self.check_and_connect(): return None
        if self.pending_cmd:
            cmd = self.pending_cmd
            self.pending_cmd = None
            return cmd
            
        if self.ser.in_waiting > 0:
            try:
                data = self.ser.read(self.ser.in_waiting)
                # 融合 serial_comf 握手协议: 自动回复 SYN 握手包
                for byte in data:
                    if byte == ASCII_SYN:
                        self.ser.write(bytes([ASCII_ACK]))
                        self.ser.flush()
                        print("[INFO] 底层握手成功 (SYN->ACK)")

                # 提取最新有效指令
                for byte in reversed(data):
                    b_char = bytes([byte])
                    if b_char in [b'1', b'2', b'3', b'4', b'5', b'6', b'7']:
                        return b_char
            except Exception: pass
        return None

    def send_sync(self, data_str, timeout=0.1):
        """精准抓取用的同步发送（等待ACK）"""
        if not self.check_and_connect(): return False
        try:
            # 发送前检查并处理残余握手
            if self.ser.in_waiting > 0:
                data = self.ser.read(self.ser.in_waiting)
                for byte in data:
                    if byte == ASCII_SYN: 
                        self.ser.write(bytes([ASCII_ACK]))
                        self.ser.flush()
                for byte in reversed(data):
                    b_char = bytes([byte])
                    if b_char in [b'1', b'2', b'3', b'4', b'5', b'6', b'7']:
                        self.pending_cmd = b_char
                        break

            packet = bytearray()
            packet.append(ASCII_STX)
            packet.extend(data_str.encode('utf-8'))
            packet.append(ASCII_ETX)
            self.ser.write(packet)
            self.ser.flush() # 强制刷新缓冲

            start_t = time.time()
            while time.time() - start_t < timeout:
                if self.ser.in_waiting > 0:
                    reply = self.ser.read(1)
                    if reply == bytes([ASCII_ACK]): return True
                    elif reply in [b'1', b'2', b'3', b'4', b'5', b'6', b'7']:  
                        self.pending_cmd = reply
            return False
        except serial.SerialException:
            if self.ser: self.ser.close()
            self.ser = None
            return False

    def send_broadcast(self, data_str):
        """心跳广播专用：不阻塞，不等ACK"""
        if not self.check_and_connect(): return
        try:
            packet = bytearray()
            packet.append(ASCII_STX)
            packet.extend(data_str.encode('utf-8'))
            packet.append(ASCII_ETX)
            self.ser.write(packet)
            self.ser.flush()
        except serial.SerialException:
            if self.ser: self.ser.close()
            self.ser = None

    def close(self):
        if self.ser is not None and self.ser.is_open:
            self.ser.close()

# ==================== 几何处理核心函数 (完全还原 he9.py / he15.py) ====================
def apply_hsv_range(hsv, low, high): return cv2.inRange(hsv, np.array(low, dtype=np.uint8), np.array(high, dtype=np.uint8))

def apply_color_mask(hsv, color_name, config): 
    cfg = config.get(color_name) 
    if not cfg: return np.zeros(hsv.shape[:2], dtype=np.uint8)
    mask = apply_hsv_range(hsv, cfg["low"], cfg["high"])
    if color_name + "2" in config: 
        mask = cv2.bitwise_or(mask, apply_hsv_range(hsv, config[color_name + "2"]["low"], config[color_name + "2"]["high"]))
    return mask

def compute_center(contour): 
    (cx, cy), r = cv2.minEnclosingCircle(contour)
    return float(cx), float(cy)

def find_best_single_slot(image, min_area=500):
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
                min_distance, best_slot = dist_to_cam_center, ShapeResult("slot", color_name, center, (x, y, bw, bh), area, dist_to_cam_center)
    return best_slot

def detect_material_cv(roi_bgr, roi_thresh):
    """
    全新过滤算法：基于二值化掩码剔除背景干扰，精准提取表面材质特征
    """
    if roi_bgr.shape[:2] != roi_thresh.shape or roi_thresh.size == 0:
        return 'plastic'
        
    area = cv2.countNonZero(roi_thresh)
    if area < 50: return 'plastic'

    hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
    v = hsv[:, :, 2]
    
    valid_v = v[roi_thresh > 127]
    if len(valid_v) == 0: return 'plastic'

    v_std = np.std(valid_v)
    high_light_count = np.sum(valid_v > 235)
    highlight_ratio = high_light_count / area

    gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
    masked_gray = cv2.bitwise_and(gray, gray, mask=roi_thresh)
    edges = cv2.Canny(masked_gray, 80, 200)
    eroded_mask = cv2.erode(roi_thresh, np.ones((5, 5), np.uint8))
    internal_edges = cv2.bitwise_and(edges, edges, mask=eroded_mask)
    edge_ratio = cv2.countNonZero(internal_edges) / area

    metal_score = 0
    if v_std > 30.0: metal_score += 1             
    if highlight_ratio > 0.015: metal_score += 1  
    if edge_ratio > 0.02: metal_score += 1        

    return 'metal' if metal_score >= 2 else 'plastic'

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

            if angle1 < angle2: 
                best_angle, best_edge_pts, opp_pt, best_dy = angle1, (tuple(map(int, p_left)), tuple(map(int, p1))), tuple(map(int, p2)), dy1
            else: 
                best_angle, best_edge_pts, opp_pt, best_dy = angle2, (tuple(map(int, p_left)), tuple(map(int, p2))), tuple(map(int, p1)), dy2
            if best_dy < 0: best_angle += 180.0
            return best_angle, box, best_edge_pts, opp_pt
        else: 
            return 0.0, box, None, None

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

def process_roi_geometry(gray_frame, x1, y1, x2, y2, shape_type):
    h_frame, w_frame = gray_frame.shape
    margin = 2
    rx1, ry1 = max(0, int(x1) - margin), max(0, int(y1) - margin)
    rx2, ry2 = min(w_frame, int(x2) + margin), min(h_frame, int(y2) + margin)
    if rx2 - rx1 < 5 or ry2 - ry1 < 5: return 0.0, None, None, None

    roi_gray = gray_frame[ry1:ry2, rx1:rx2].copy()
    
    thresh_val, _ = cv2.threshold(roi_gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    
    final_thresh_val = max(thresh_val, 160)
    _, roi_thresh = cv2.threshold(roi_gray, final_thresh_val, 255, cv2.THRESH_BINARY)

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
    if not os.path.exists(SHAPE_MODEL_PATH): return print(f"[ERROR] 形状模型不存在: {SHAPE_MODEL_PATH}")

    serial_mgr = SerialManager(BAUD_RATE)
    serial_mgr.check_and_connect()

    device = 'cuda:0' if USE_GPU and torch.cuda.is_available() else 'cpu'
    print(f"[INFO] 使用设备: {device}")
    
    shape_model = YOLO(SHAPE_MODEL_PATH)
    shape_model.to(device)
    material_model = YOLO(MATERIAL_MODEL_PATH).to(device) if os.path.exists(MATERIAL_MODEL_PATH) else None

    cap = cv2.VideoCapture(CAMERA_ID, cv2.CAP_DSHOW)
    if not cap.isOpened(): cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    print("[INFO] 视觉系统运行中...正在向单片机广播视野...按 'q' 退出")
    
    last_status = "Broadcasting View..."
    last_color = (255, 255, 0)
    last_broadcast_time = 0

    while True:
        ret, frame = cap.read()
        if not ret: continue
        h_frame, w_frame = frame.shape[:2]

        cmd = serial_mgr.get_latest_command()

        # ================= 转盘特例 =================
        if cmd == b'7':
            target = find_best_single_slot(frame, min_area=1000)
            if target:
                tcx, tcy = map(int, target.center)
                bx, by, bw, bh = target.bbox
                data_str = f"[[{tcx},{tcy},{bw},{bh},{target.color_name}:1.00:0.00]]"
                success = serial_mgr.send_sync(data_str, timeout=0.1)
                last_status, last_color = (f"Sent SLOT -> OK", (0, 255, 0)) if success else ("Send SLOT -> ERR", (0, 0, 255))
            else:
                serial_mgr.send_sync("[]", timeout=0.1)
                last_status, last_color = ("No SLOT found -> []", (0, 0, 255))
            cv2.putText(frame, last_status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, last_color, 2)
            cv2.imshow("Shape Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'): break
            continue

        # ================= 常规推理 =================
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, frame_thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        results = shape_model(frame, conf=CONF_THRESHOLD, verbose=False)[0]
        
        boxes_xyxy_raw = results.obb.xyxy.cpu().numpy() if hasattr(results, 'obb') and results.obb is not None else (results.boxes.xyxy.cpu().numpy() if hasattr(results, 'boxes') and results.boxes is not None else [])
        classes_raw = results.obb.cls.cpu().numpy() if hasattr(results, 'obb') and results.obb is not None else (results.boxes.cls.cpu().numpy() if hasattr(results, 'boxes') and results.boxes is not None else [])
        confs_raw = results.obb.conf.cpu().numpy() if hasattr(results, 'obb') and results.obb is not None else (results.boxes.conf.cpu().numpy() if hasattr(results, 'boxes') and results.boxes is not None else [])

        shape_names = shape_model.names
        boxes_xyxy, classes, confs = filter_smallest_boxes(boxes_xyxy_raw, classes_raw, confs_raw)

        requested_target_found = False
        current_view_objects = []  

        for i in range(len(boxes_xyxy)):
            x1, y1, x2, y2 = map(int, boxes_xyxy[i])
            x1, y1, x2, y2 = max(0, x1), max(0, y1), min(w_frame, x2), min(h_frame, y2)
            
            cls_id, conf = int(classes[i]), float(confs[i])
            original_shape = shape_names[cls_id]
            final_tag = original_shape  

            # --- 材质推理 ---
            if original_shape in ['san', 'ti'] and (x2 - x1) > 10 and (y2 - y1) > 10:
                roi_bgr = frame[y1:y2, x1:x2].copy()
                roi_thresh = frame_thresh[y1:y2, x1:x2].copy()
                
                final_material = detect_material_cv(roi_bgr, roi_thresh)
                
                if material_model is not None:
                    mat_results = material_model(roi_bgr, conf=CONF_THRESHOLD, verbose=False)[0]
                    if hasattr(mat_results, 'boxes') and mat_results.boxes is not None and len(mat_results.boxes) > 0:
                        mat_conf = float(mat_results.boxes.conf[0].item())
                        if mat_conf > 0.85:
                            cls_idx = int(mat_results.boxes.cls[0].item())
                            material_name_raw = material_model.names[cls_idx].lower()
                            final_material = 'metal' if 'metal' in material_name_raw else 'plastic'

                if original_shape == 'san': 
                    final_tag = 'ms' if final_material == 'metal' else 'ps'
                elif original_shape == 'ti': 
                    final_tag = 'mt' if final_material == 'metal' else 'pt'

            cx, cy = int((x1+x2)/2), int((y1+y2)/2)
            w, h = int(x2-x1), int(y2-y1)

            # --- UI绘制与角度计算 ---
            color = SHAPE_COLOR.get(final_tag, (255, 255, 255))
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            angle_deg = 0.0

            if original_shape in ['san', 'ti']:
                angle_deg, box_global, best_edge_global, opp_pt_global = process_roi_geometry(gray,x1, y1, x2, y2, original_shape)
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
                angle_deg, box_global, _, _ = process_roi_geometry(gray, x1,y1, x2, y2, 'si')
                angle_deg = angle_deg % 90.0
                
            angle_norm = angle_deg / 360.0

            label = f"{final_tag} {conf:.2f} Ang:{angle_norm:.4f}"
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            broadcast_item = f"{final_tag}:{cx}:{cy}"
            current_view_objects.append(broadcast_item)

            # 【精准响应模式】
            if cmd is not None and cmd in CMD_MAP and not requested_target_found:
                target_tag = CMD_MAP[cmd]
                if final_tag == target_tag:
                    data_str = f"[{cx},{cy},{w},{h},{final_tag}:{conf:.4f}:{angle_norm:.4f}]"
                    print(f"\n[INFO] 命中请求目标 {target_tag}，正发送: {data_str}")
                    success = serial_mgr.send_sync(data_str, timeout=0.1)
                    last_status = f"CMD:{cmd.decode()} -> {final_tag} OK" if success else f"CMD:{cmd.decode()} -> ERR"
                    last_color = (0, 255, 0) if success else (0, 0, 255)
                    requested_target_found = True

        if cmd is not None and cmd in CMD_MAP and not requested_target_found:
            serial_mgr.send_sync("[]", timeout=0.1)
            last_status = f"CMD:{cmd.decode()} -> Not Found -> []"
            last_color = (0, 0, 255)

        # ================= 主动心跳广播 =================
        if cmd is None:
            current_time = time.time()
            if current_time - last_broadcast_time > 0.1:  # 10Hz 频率
                if current_view_objects:
                    broadcast_str = "V:" + ",".join(current_view_objects)
                else:
                    broadcast_str = "V:null"
                
                serial_mgr.send_broadcast(broadcast_str)
                last_broadcast_time = current_time
                last_status = f"Broadcasting... ({len(current_view_objects)} items)"
                last_color = (255, 255, 0) 

        cv2.putText(frame, last_status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, last_color, 2)
        cv2.imshow("Shape Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'): break

    cap.release()
    serial_mgr.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()