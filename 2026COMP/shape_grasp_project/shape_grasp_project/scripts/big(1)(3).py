"""
【统一融合版】转盘微距定位 + 形状角度极精识别 + 串口通讯
功能：
  1. 转盘槽位定位：HSV 提取红/蓝色槽位，锁定距离画面中心最近的槽，响应 MCU 指令 '7'
  2. YOLO 形状检测 + 传统视觉角度计算：
     - 检测 san/ti/yuan/si 四种形状
     - 对 san/ti 提取精准旋转角度（抗边缘干扰）
     - 角度限制：统一除以 360 进行归一化 (输出 0.0000 ~ 1.0000 范围)
  3. 串口自动重连，支持指令 '7'，并自动连续推送形状识别数据
运行：python unified_vision.py
依赖：runs/5182.pt（形状模型）
"""
import cv2
import serial
import serial.tools.list_ports
import threading
import time
import math
import numpy as np
import itertools
import sys
import os
import torch
from ultralytics import YOLO
from dataclasses import dataclass
from typing import List, Tuple, Optional

SHAPE_MODEL_PATH = "shape_grasp_project/runs/5184.pt"          
CAMERA_ID = 1
SERIAL_PORT = "AUTO"
BAUDRATE = 115200
CONF_THRESHOLD = 0.25
USE_GPU = True
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# 协议字节
ASCII_STX = 0x02
ASCII_ETX = 0x03
ASCII_ACK = 0x06
ASCII_SYN = 0x16

# 形状颜色（YOLO 部分）
SHAPE_COLOR = {
    'san': (0, 255, 0),    # 绿
    'ti':  (255, 0, 0),    # 蓝
    'yuan':(0, 255, 255),  # 黄
    'si':  (0, 165, 255)   # 橙
}

# 转盘颜色
HSV_CONFIG = {
    "red":   {"low": [0, 100, 100], "high": [10, 255, 255]},
    "red2":  {"low": [160, 100, 100], "high": [179, 255, 255]},
    "blue":  {"low": [90, 50, 50], "high": [140, 255, 255]}
}
ALLOWED_COLORS = ["red", "blue"]
COLOR_BGR_MAP = {"red": (0, 0, 255), "blue": (255, 0, 0)}

# 编码处理
if sys.stdout.encoding and sys.stdout.encoding.upper() != 'UTF-8':
    try: sys.stdout.reconfigure(encoding='utf-8')
    except: pass

# ==================== 转盘数据结构 ====================
@dataclass
class SlotTarget:
    color_name: str
    center: Tuple[float, float]
    bbox: Tuple[int, int, int, int]
    area: float
    dist: float

# ==================== 工具函数：串口自动检测 ====================
def auto_detect_serial_port(keyword=None, silent=False):
    ports = serial.tools.list_ports.comports()
    if not ports: return None
    if keyword:
        for p in ports:
            if keyword.lower() in p.description.lower():
                if not silent: print(f"[INFO] 自动匹配串口: {p.device}")
                return p.device
    selected = ports[0].device
    if not silent: print(f"[INFO] 自动选择串口: {selected}")
    return selected

# ==================== 转盘定位核心函数 ====================
def apply_hsv_range(hsv, low, high):
    return cv2.inRange(hsv, np.array(low), np.array(high))

def apply_color_mask(hsv, color_name):
    cfg = HSV_CONFIG.get(color_name)
    if not cfg: return np.zeros(hsv.shape[:2], dtype=np.uint8)
    mask = apply_hsv_range(hsv, cfg["low"], cfg["high"])
    if color_name + "2" in HSV_CONFIG:
        mask2 = apply_hsv_range(hsv, HSV_CONFIG[color_name+"2"]["low"], HSV_CONFIG[color_name+"2"]["high"])
        mask = cv2.bitwise_or(mask, mask2)
    return mask

def find_best_single_slot(image, min_area=500):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    img_h, img_w = image.shape[:2]
    img_cx, img_cy = img_w/2.0, img_h/2.0
    best = None
    min_dist = float('inf')

    for color_name in ALLOWED_COLORS:
        mask = apply_color_mask(hsv, color_name)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((15,15), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            hull = cv2.convexHull(cnt)
            area = cv2.contourArea(hull)
            if area < min_area: continue
            x, y, w, h = cv2.boundingRect(hull)
            if x <= 5 or y <= 5 or (x+w) >= img_w-5 or (y+h) >= img_h-5:
                continue
            (cx, cy), _ = cv2.minEnclosingCircle(hull)
            dist = math.hypot(cx - img_cx, cy - img_cy)
            if dist < min_dist:
                min_dist = dist
                best = SlotTarget(color_name, (cx, cy), (x, y, w, h), area, dist)
    return best

# ==================== YOLO 去重工具 ====================
def is_same_object(boxA, boxB, overlap_thresh=0.4):
    xA, yA = max(boxA[0], boxB[0]), max(boxA[1], boxB[1])
    xB, yB = min(boxA[2], boxB[2]), min(boxA[3], boxB[3])
    interArea = max(0, xB-xA) * max(0, yB-yA)
    if interArea == 0: return False
    areaA = (boxA[2]-boxA[0])*(boxA[3]-boxA[1])
    areaB = (boxB[2]-boxB[0])*(boxB[3]-boxB[1])
    return interArea / float(min(areaA, areaB)) > overlap_thresh

def filter_smallest_boxes(boxes, classes, confs):
    if len(boxes) == 0: return [], [], []
    areas = [(b[2]-b[0])*(b[3]-b[1]) for b in boxes]
    sorted_idx = np.argsort(areas)
    keep_boxes, keep_cls, keep_conf = [], [], []
    for idx in sorted_idx:
        boxA = boxes[idx]
        if not any(is_same_object(boxA, kb) for kb in keep_boxes):
            keep_boxes.append(boxA)
            keep_cls.append(classes[idx])
            keep_conf.append(confs[idx])
    return keep_boxes, keep_cls, keep_conf

# ==================== 角度计算核心 ====================
def get_robust_angle_and_box(cnt, shape_type):
    rect = cv2.minAreaRect(cnt)
    (cx, cy), (w, h), theta = rect
    box = cv2.boxPoints(rect).astype(np.int32)

    if shape_type == 'san':
        x, y, bw, bh = cv2.boundingRect(cnt)
        shifted = cnt - np.array([[[x, y]]])
        mask_shape = (bh+10, bw+10)
        orig_mask = np.zeros(mask_shape, dtype=np.uint8)
        cv2.drawContours(orig_mask, [shifted], -1, 255, -1)
        hull = cv2.convexHull(shifted)
        epsilon = 0.015 * cv2.arcLength(hull, True)
        approx = cv2.approxPolyDP(hull, epsilon, True)
        pts = approx.reshape(-1, 2)
        best_tri = None; best_iou = 0.0
        if len(pts) >= 3:
            if len(pts) > 8: pts = pts[:8]
            for combo in itertools.combinations(pts, 3):
                tri_pts = np.array(combo, dtype=np.int32)
                tri_mask = np.zeros(mask_shape, dtype=np.uint8)
                cv2.drawContours(tri_mask, [tri_pts], -1, 255, -1)
                inter = cv2.bitwise_and(orig_mask, tri_mask)
                area_inter = cv2.countNonZero(inter)
                area_union = cv2.countNonZero(orig_mask) + cv2.countNonZero(tri_mask) - area_inter
                if area_union > 0:
                    iou = area_inter / float(area_union)
                    if iou > best_iou:
                        best_iou = iou; best_tri = tri_pts
        if best_tri is not None:
            triangle = best_tri + np.array([x, y])
            triangle = np.expand_dims(triangle, axis=1)
        else:
            _, triangle = cv2.minEnclosingTriangle(cnt)
        if triangle is not None and len(triangle) == 3:
            pts = triangle.reshape(3,2)
            left_idx = np.argmin(pts[:,0])
            p_left = pts[left_idx]
            p1 = pts[(left_idx+1)%3]; p2 = pts[(left_idx+2)%3]
            dx1, dy1 = p1[0]-p_left[0], p1[1]-p_left[1]
            angle1 = math.degrees(math.atan2(abs(dx1), abs(dy1))) if (dx1 or dy1) else 90
            dx2, dy2 = p2[0]-p_left[0], p2[1]-p_left[1]
            angle2 = math.degrees(math.atan2(abs(dx2), abs(dy2))) if (dx2 or dy2) else 90
            if angle1 < angle2:
                angle = angle1; edge = (tuple(map(int, p_left)), tuple(map(int, p1)))
                opp = tuple(map(int, p2)); dy = dy1
            else:
                angle = angle2; edge = (tuple(map(int, p_left)), tuple(map(int, p2)))
                opp = tuple(map(int, p1)); dy = dy2
            if dy < 0: angle += 180.0
            return angle, box, edge, opp
        return 0.0, box, None, None

    elif shape_type == 'ti':
        hull = cv2.convexHull(cnt)
        epsilon = 0.02 * cv2.arcLength(hull, True)
        approx = cv2.approxPolyDP(hull, epsilon, True)
        if len(approx) == 4:
            pts = approx.reshape(4,2)
            edges = []
            for i in range(4):
                p1, p2 = pts[i], pts[(i+1)%4]
                vx, vy = p2[0]-p1[0], p2[1]-p1[1]
                length = math.hypot(vx, vy)
                edges.append({'p1':p1, 'p2':p2, 'vx':vx, 'vy':vy, 'len':length})
            def parallel_score(eA, eB):
                if eA['len']==0 or eB['len']==0: return 0
                return abs((eA['vx']*eB['vx']+eA['vy']*eB['vy'])/(eA['len']*eB['len']))
            score02 = parallel_score(edges[0], edges[2])
            score13 = parallel_score(edges[1], edges[3])
            if score02 > score13:
                base1, base2 = edges[0], edges[2]
            else:
                base1, base2 = edges[1], edges[3]
            if base1['len'] > base2['len']:
                long_b, short_b = base1, base2
            else:
                long_b, short_b = base2, base1
            mid_long = (int((long_b['p1'][0]+long_b['p2'][0])/2), int((long_b['p1'][1]+long_b['p2'][1])/2))
            mid_short = (int((short_b['p1'][0]+short_b['p2'][0])/2), int((short_b['p1'][1]+short_b['p2'][1])/2))
            dx = mid_short[0] - mid_long[0]
            dy = mid_short[1] - mid_long[1]
            angle = math.degrees(math.atan2(dy, dx)) % 360
            return angle, box, (mid_long, mid_short), None
        M = cv2.moments(cnt)
        if M['m00'] == 0: return 0.0, box, None, None
        mcx, mcy = M['m10']/M['m00'], M['m01']/M['m00']
        vx, vy = cx - mcx, cy - mcy
        if math.hypot(vx, vy) < 1.0: return 0.0, box, None, None
        angle = math.degrees(math.atan2(vy, vx)) % 360
        arrow_len = min(w, h) * 0.8
        pt1 = (int(mcx), int(mcy))
        pt2 = (int(mcx + math.cos(math.radians(angle))*arrow_len), int(mcy + math.sin(math.radians(angle))*arrow_len))
        return angle, box, (pt1, pt2), None

    else:
        max_len = 0; angle = 0
        for i in range(4):
            p1, p2 = box[i], box[(i+1)%4]
            length = math.hypot(p2[0]-p1[0], p2[1]-p1[1])
            if length > max_len:
                max_len = length
                angle = math.degrees(math.atan2(p2[1]-p1[1], p2[0]-p1[0]))
        if angle < 0: angle += 180
        if angle >= 180: angle -= 180
        return angle, box, None, None

# ==================== 统一角度归一化算法 ====================
def normalize_shape_angle(angle, shape_type):
    """
    统一处理所有形状的角度限制，并全部除以 360 实现归一化 (结果为 0.0~1.0)
    """
    if shape_type == 'san':
        val = angle % 360.0
    elif shape_type == 'ti':
        val = angle % 360.0
    elif shape_type == 'si':
        val = angle % 90.0
    elif shape_type == 'yuan':
        val = 0.0
    else:
        val = angle % 360.0
        
    # 核心修改：统一除以 360.0，使其落在 0~1 的区间
    return val / 360.0

def process_roi_geometry(frame_thresh, x1, y1, x2, y2, shape_type):
    h_frame, w_frame = frame_thresh.shape
    margin = 2
    rx1, ry1 = max(0, x1-margin), max(0, y1-margin)
    rx2, ry2 = min(w_frame, x2+margin), min(h_frame, y2+margin)
    if rx2-rx1 < 5 or ry2-ry1 < 5: return 0.0, None, None, None

    roi = frame_thresh[ry1:ry2, rx1:rx2].copy()
    h_roi, w_roi = roi.shape

    kernel_size = max(3, int(min(w_roi, h_roi)*0.08))
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    eroded = cv2.erode(roi, kernel, iterations=1)
    cnts_eroded, _ = cv2.findContours(eroded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best_cnt = None
    if cnts_eroded:
        cx_r, cy_r = w_roi/2.0, h_roi/2.0
        min_dist = float('inf')
        for c in cnts_eroded:
            M = cv2.moments(c)
            if M['m00'] == 0: continue
            ccx, ccy = M['m10']/M['m00'], M['m01']/M['m00']
            dist = (ccx-cx_r)**2 + (ccy-cy_r)**2
            if dist < min_dist:
                min_dist = dist; best_cnt = c
    if best_cnt is not None:
        clean_mask = np.zeros_like(roi)
        cv2.drawContours(clean_mask, [best_cnt], -1, 255, -1)
        dilate_kernel = np.ones((kernel_size+2, kernel_size+2), np.uint8)
        clean_mask = cv2.dilate(clean_mask, dilate_kernel, iterations=1)
        roi = cv2.bitwise_and(roi, clean_mask)

    cv2.rectangle(roi, (0,0), (w_roi-1, h_roi-1), 0, 1)
    contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours: return 0.0, None, None, None
    cnt = max(contours, key=cv2.contourArea)

    if shape_type in ['san', 'ti']:
        epsilon = 0.035 * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)
        if len(approx) >= 3: cnt = approx

    angle, box_local, edge_local, opp_local = get_robust_angle_and_box(cnt, shape_type)

    box_global = box_local + np.array([rx1, ry1])
    edge_global = None; opp_global = None
    if edge_local:
        p1 = tuple(edge_local[0] + np.array([rx1, ry1]))
        p2 = tuple(edge_local[1] + np.array([rx1, ry1]))
        edge_global = (p1, p2)
    if opp_local:
        opp_global = tuple(opp_local + np.array([rx1, ry1]))
    return angle, box_global, edge_global, opp_global

# ==================== 统一主系统 ====================
class UnifiedServer:
    def __init__(self):
        print("[INFO] 初始化统一视觉引擎（转盘 + 形状角度归一化）...")
        if not os.path.exists(SHAPE_MODEL_PATH):
            print(f"[ERROR] 形状模型缺失: {SHAPE_MODEL_PATH}")
            exit(1)

        device = 'cuda:0' if USE_GPU and torch.cuda.is_available() else 'cpu'
        print(f"[INFO] YOLO 设备: {device}")
        self.shape_model = YOLO(SHAPE_MODEL_PATH)
        self.shape_model.to(device)

        self.cap = cv2.VideoCapture(CAMERA_ID, cv2.CAP_DSHOW)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

        self.running = True
        self.latest_frame = None
        self.camera_ok = False
        self.frame_lock = threading.Lock()

        self.slot_target = None
        self.slot_lock = threading.Lock()
        self.slot_last_reply_time = 0

        self.shape_results = []
        self.shape_lock = threading.Lock()

        self.ser = None
        if not self.connect_serial(silent=False):
            print("[WARNING] 串口连接失败，后台自动重连...")

    def connect_serial(self, silent=False):
        if self.ser is not None:
            try: self.ser.close()
            except: pass
        port = SERIAL_PORT if str(SERIAL_PORT).upper() != "AUTO" else auto_detect_serial_port("USB", silent)
        if port:
            try:
                self.ser = serial.Serial(port, BAUDRATE, timeout=0.01)
                if not silent: print(f"[INFO] 串口已连接: {port}")
                return True
            except: return False
        return False

    def capture_loop(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                with self.frame_lock:
                    self.latest_frame = frame.copy()
                self.camera_ok = True
            else:
                self.camera_ok = False
            time.sleep(0.005)

    def turntable_loop(self):
        while self.running:
            with self.frame_lock:
                if self.latest_frame is None or not self.camera_ok:
                    time.sleep(0.01); continue
                frame = self.latest_frame.copy()
            target = find_best_single_slot(frame, min_area=1000)
            with self.slot_lock:
                self.slot_target = target
            time.sleep(0.03)

    def shape_angle_loop(self):
        while self.running:
            with self.frame_lock:
                if self.latest_frame is None or not self.camera_ok:
                    time.sleep(0.01); continue
                frame = self.latest_frame.copy()

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

            results = self.shape_model(frame, conf=CONF_THRESHOLD, verbose=False)[0]
            boxes_raw, cls_raw, conf_raw = [], [], []
            if hasattr(results, 'obb') and results.obb is not None:
                boxes_raw = results.obb.xyxy.cpu().numpy()
                cls_raw = results.obb.cls.cpu().numpy()
                conf_raw = results.obb.conf.cpu().numpy()
            elif hasattr(results, 'boxes') and results.boxes is not None:
                boxes_raw = results.boxes.xyxy.cpu().numpy()
                cls_raw = results.boxes.cls.cpu().numpy()
                conf_raw = results.boxes.conf.cpu().numpy()
            names = self.shape_model.names

            boxes, cls, conf = filter_smallest_boxes(boxes_raw, cls_raw, conf_raw)

            detections = []
            for i in range(len(boxes)):
                x1, y1, x2, y2 = map(int, boxes[i])
                cls_id = int(cls[i])
                conf_val = float(conf[i])
                shape_class = names[cls_id]

                item = {
                    'bbox': (x1, y1, x2, y2),
                    'shape_class': shape_class,
                    'conf': conf_val,
                    'angle': 0.0,
                    'box_global': None,
                    'edge': None,
                    'opp': None
                }

                if shape_class in ['san', 'ti']:
                    angle_raw, box_global, edge, opp = process_roi_geometry(thresh, x1, y1, x2, y2, shape_class)
                    item['angle'] = normalize_shape_angle(angle_raw, shape_class)
                    item['box_global'] = box_global
                    item['edge'] = edge
                    item['opp'] = opp
                elif shape_class == 'si':
                    angle_raw, box_global, _, _ = process_roi_geometry(thresh, x1, y1, x2, y2, 'si')
                    item['angle'] = normalize_shape_angle(angle_raw, shape_class)
                    item['box_global'] = box_global
                else:  
                    item['angle'] = normalize_shape_angle(0.0, shape_class)
                    
                detections.append(item)

            with self.shape_lock:
                self.shape_results = detections
            time.sleep(0.03)

    def handle_request_turntable(self):
        if not self.camera_ok:
            return bytes([ASCII_STX]) + b"[]" + bytes([ASCII_ETX])
        with self.slot_lock:
            target = self.slot_target
        if target is None:
            return bytes([ASCII_STX]) + b"[]" + bytes([ASCII_ETX])
        tcx, tcy = map(int, target.center)
        bx, by, bw, bh = target.bbox
        tag = target.color_name
        self.slot_last_reply_time = time.time()
        data_str = f"[[{tcx},{tcy},{bw},{bh},{tag}:1.00:0.00]]"
        return bytes([ASCII_STX]) + data_str.encode() + bytes([ASCII_ETX])

    def serial_loop(self):
        last_send_time = 0
        while self.running:
            if self.ser is None or not self.ser.is_open:
                self.connect_serial(silent=True)
                time.sleep(1)
                continue
            try:
                # 优先处理下位机指令
                if self.ser.in_waiting:
                    byte = self.ser.read(1)
                    req_code = byte[0]
                    if req_code == ASCII_SYN:
                        self.ser.write(bytes([ASCII_ACK]))
                        continue
                    if req_code == ord('7'):
                        print("[POLL] 收到指令 '7'")
                        response = self.handle_request_turntable()
                        self.ser.write(response)
                        self.ser.flush()
                        print("[REPLY] 转盘数据已发送")

                # 定时主动推送形状角度数据
                now = time.time()
                if now - last_send_time >= 0.03:  
                    last_send_time = now
                    
                    with self.shape_lock:
                        detections = self.shape_results.copy()
                        
                    if detections:
                        best_det = max(detections, key=lambda d: d['conf'])
                        x1, y1, x2, y2 = best_det['bbox']
                        
                        best = {
                            'x': int((x1 + x2) / 2),
                            'y': int((y1 + y2) / 2),
                            'w': int(x2 - x1),
                            'h': int(y2 - y1),
                            'tag': best_det['shape_class'],
                            'conf': float(best_det['conf']),
                            'angle_norm': float(best_det['angle'])
                        }
                        
                        # 格式化输出 (angle_norm 已经除以了360，变成 0.xxxx 的小数)
                        data_str = f"[{best['x']},{best['y']},{best['w']},{best['h']},{best['tag']}:{best['conf']:.4f}:{best['angle_norm']:.4f}]"
                        
                        packet = bytes([ASCII_STX]) + data_str.encode() + bytes([ASCII_ETX])
                        self.ser.write(packet)
                        self.ser.flush()
                        
                time.sleep(0.005)
            except Exception as e:
                if self.ser:
                    try: self.ser.close()
                    except: pass
                self.ser = None
                time.sleep(1)

    def run(self):
        threading.Thread(target=self.capture_loop, daemon=True).start()
        threading.Thread(target=self.turntable_loop, daemon=True).start()
        threading.Thread(target=self.shape_angle_loop, daemon=True).start()
        threading.Thread(target=self.serial_loop, daemon=True).start()

        win_name = "Unified Vision: Slot + Shape & Angle"
        cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)

        print("[INFO] 系统运行中，按 'q' 退出")
        while self.running:
            with self.frame_lock:
                frame_disp = self.latest_frame.copy() if self.latest_frame is not None else None
            if frame_disp is None:
                cv2.waitKey(1); continue

            h, w = frame_disp.shape[:2]
            img_cx, img_cy = w//2, h//2

            # ---- 绘制转盘 ----
            with self.slot_lock:
                slot = self.slot_target
            cv2.drawMarker(frame_disp, (img_cx, img_cy), (200,200,200), cv2.MARKER_CROSS, 40, 1)
            if slot is not None:
                color = COLOR_BGR_MAP.get(slot.color_name, (0,255,0))
                x, y, bw, bh = slot.bbox
                cv2.rectangle(frame_disp, (x,y), (x+bw, y+bh), color, 3)
                tcx, tcy = map(int, slot.center)
                cv2.circle(frame_disp, (tcx, tcy), 6, (0,0,255), -1)
                cv2.line(frame_disp, (img_cx, img_cy), (tcx, tcy), (0,255,255), 2)
                cv2.putText(frame_disp, f"SLOT {slot.color_name.upper()}", (x, max(y-10,20)),
                            cv2.FONT_HERSHEY_DUPLEX, 0.6, color, 2)
                if time.time() - self.slot_last_reply_time < 1.5:
                    cv2.putText(frame_disp, "[MCU REQUESTED]", (x, max(y-35,45)),
                                cv2.FONT_HERSHEY_DUPLEX, 0.6, (0,0,255), 2)

            # ---- 绘制形状角度 ----
            with self.shape_lock:
                detections = self.shape_results.copy()
            for d in detections:
                x1, y1, x2, y2 = d['bbox']
                shape_class = d['shape_class']
                conf = d['conf']
                angle = d['angle']
                color = SHAPE_COLOR.get(shape_class, (255,255,255))
                cv2.rectangle(frame_disp, (x1,y1), (x2,y2), color, 2)

                if d['box_global'] is not None:
                    cv2.polylines(frame_disp, [d['box_global']], True, (0,255,255), 1)
                if d['edge'] is not None:
                    cv2.arrowedLine(frame_disp, d['edge'][0], d['edge'][1], (0,0,255), 2, tipLength=0.2)
                if d['opp'] is not None:
                    cv2.circle(frame_disp, d['opp'], 5, (0,255,0), -1)

                # 界面显示也修改为保留 3 位小数，方便观察归一化后的数据
                label = f"{shape_class} {conf:.2f} Ang:{angle:.3f}"
                cv2.putText(frame_disp, label, (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            cv2.imshow(win_name, frame_disp)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.running = False
                break

        self.cap.release()
        if self.ser and self.ser.is_open:
            self.ser.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    server = UnifiedServer()
    server.run()