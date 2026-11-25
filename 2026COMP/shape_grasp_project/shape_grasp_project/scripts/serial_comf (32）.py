import cv2
import serial
import serial.tools.list_ports
import threading
import time
import math
import numpy as np
import sys
import torch
import os
from ultralytics import YOLO
from dataclasses import dataclass
from typing import List, Tuple, Optional
# 模型路径 (YOLO使用)
SHAPE_MODEL_PATH = "shape_grasp_project/runs/5184.pt"      
MATERIAL_MODEL_PATH = "shape_grasp_project/runs/5183m.pt"        

CAMERA_ID = 1

# 串口配置 (共享配置，波特率取高者 115200 保证转盘响应)
SERIAL_PORT = "AUTO" 
BAUDRATE = 9600

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
CONF_THRESHOLD = 0.25
USE_GPU = True

# 通讯协议字节
ASCII_STX = 0x02
ASCII_ETX = 0x03
ASCII_ACK = 0x06
ASCII_SYN = 0x16
# 修复控制台UTF-8编码问题
if sys.stdout.encoding and sys.stdout.encoding.upper() != 'UTF-8':
    try:
        sys.stdout.reconfigure(encoding='utf-8')
    except Exception:
        pass
    
@dataclass
class ShapeResult:
    shape_type: str          
    color_name: str          
    center: Tuple[float, float]  
    bbox: Tuple[int, int, int, int]  
    contour_area: float      
    dist_to_center: float    

# 转盘使用的 HSV 颜色配置
HSV_CONFIG = {
    "red":   {"low":  [0,   100, 100], "high": [10,  255, 255]},
    "red2":  {"low":  [160, 100, 100], "high": [179, 255, 255]}, 
    "blue":  {"low":  [90,  50,  50],  "high": [140, 255, 255]}
}
ALLOWED_COLORS = ["red", "blue"]
COLOR_BGR_MAP = {
    "red":  (0, 0, 255), "blue": (255, 0, 0),
}

# YOLO使用的指令与颜色映射
CMD_MAP = {
    ord('1'): 'ps',   ord('2'): 'ms',   ord('3'): 'pt',   
    ord('4'): 'mt',   ord('5'): 'si',   ord('6'): 'yuan', 
}
COLOR_MAP = {
    'si': (0, 255, 0),    'san': (255, 0, 0),   'ti': (0, 0, 255),
    'yuan': (0, 255, 255),'pt': (0, 255, 255),  'mt': (0, 165, 255), 
    'ps': (255, 0, 255),  'ms': (255, 0, 0)   
}

# =====================================================================
# 【3. 纯算法函数区】 (全盘保留，不改动一行代码)
# =====================================================================
def auto_detect_serial_port(keyword=None, silent=False):
    ports = serial.tools.list_ports.comports()
    if not ports:
        if not silent: print("[ERROR] 未检测到任何可用串口！")
        return None
    if keyword:
        for port in ports:
            if keyword.lower() in port.description.lower():
                if not silent: print(f"[INFO] 自动匹配到串口: {port.device}")
                return port.device
    selected_port = ports[0].device
    if not silent: print(f"[INFO] 自动选择串口: {selected_port}")
    return selected_port

def apply_hsv_range(hsv: np.ndarray, low: List[int], high: List[int]) -> np.ndarray:
    return cv2.inRange(hsv, np.array(low, dtype=np.uint8), np.array(high, dtype=np.uint8))

def apply_color_mask(hsv: np.ndarray, color_name: str, config: dict) -> np.ndarray:
    cfg = config.get(color_name)
    if not cfg: return np.zeros(hsv.shape[:2], dtype=np.uint8)
    
    mask = apply_hsv_range(hsv, cfg["low"], cfg["high"])
    if color_name + "2" in config:
        mask2 = apply_hsv_range(hsv, config[color_name + "2"]["low"], config[color_name + "2"]["high"])
        mask = cv2.bitwise_or(mask, mask2)
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
        if color_name.endswith("2"): continue
        if color_name not in ALLOWED_COLORS: continue
        mask = apply_color_mask(hsv, color_name, HSV_CONFIG)
        kernel_open = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)
        kernel_close = np.ones((15, 15), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            hull = cv2.convexHull(cnt)
            area = cv2.contourArea(hull)
            if area < min_area: continue
            x, y, bw, bh = cv2.boundingRect(hull)
            margin = 5
            if x <= margin or y <= margin or (x + bw) >= img_w - margin or (y + bh) >= img_h - margin:
                continue
            center = compute_center(hull)
            dist_to_cam_center = math.hypot(center[0] - img_cx, center[1] - img_cy)
            if dist_to_cam_center < min_distance:
                min_distance = dist_to_cam_center
                best_slot = ShapeResult(
                    shape_type="slot",
                    color_name=color_name,
                    center=center, bbox=(x, y, bw, bh),
                    contour_area=area, dist_to_center=dist_to_cam_center
                )
    return best_slot

def get_iou(boxA, boxB):
    xA = max(boxA[0], boxB[0])
    yA = max(boxA[1], boxB[1])
    xB = min(boxA[2], boxB[2])
    yB = min(boxA[3], boxB[3])
    interArea = max(0, xB - xA) * max(0, yB - yA)
    if interArea == 0: return 0.0
    boxAArea = (boxA[2] - boxA[0]) * (boxA[3] - boxA[1])
    boxBArea = (boxB[2] - boxB[0]) * (boxB[3] - boxB[1])
    return interArea / float(boxAArea + boxBArea - interArea)

def is_same_object(boxA, boxB, overlap_thresh=0.4):
    xA = max(boxA[0], boxB[0])
    yA = max(boxA[1], boxB[1])
    xB = min(boxA[2], boxB[2])
    yB = min(boxA[3], boxB[3])
    interArea = max(0, xB - xA) * max(0, yB - yA)
    if interArea == 0: return False
    boxAArea = (boxA[2] - boxA[0]) * (boxA[3] - boxA[1])
    boxBArea = (boxB[2] - boxB[0]) * (boxB[3] - boxB[1])
    iom = interArea / float(min(boxAArea, boxBArea))
    return iom > overlap_thresh

def filter_smallest_boxes(boxes, classes, confs):
    if len(boxes) == 0: return [], [], []
    areas = []
    for box in boxes:
        w = max(0, box[2] - box[0])
        h = max(0, box[3] - box[1])
        areas.append(w * h)
    sorted_indices = np.argsort(areas)
    keep_boxes, keep_classes, keep_confs = [], [], []
    for idx in sorted_indices:
        boxA = boxes[idx]
        overlap = False
        for keep_box in keep_boxes:
            if is_same_object(boxA, keep_box):
                overlap = True
                break
        if not overlap:
            keep_boxes.append(boxA)
            keep_classes.append(classes[idx])
            keep_confs.append(confs[idx])
    return keep_boxes, keep_classes, keep_confs

def get_robust_angle_and_box(cnt, shape_type):
    rect = cv2.minAreaRect(cnt)
    (cx, cy), (w, h), theta = rect
    box = cv2.boxPoints(rect)
    box = np.int32(box)
    
    if shape_type == 'san':
        retval, triangle = cv2.minEnclosingTriangle(cnt)
        if retval and triangle is not None and len(triangle) == 3:
            pts = triangle.reshape(3, 2)
            left_idx = np.argmin(pts[:, 0])
            p_left = pts[left_idx]
            p1 = pts[(left_idx + 1) % 3]
            p2 = pts[(left_idx + 2) % 3]
            dx1 = p1[0] - p_left[0]
            dy1 = p1[1] - p_left[1] 
            angle1 = math.degrees(math.atan2(abs(dx1), abs(dy1))) if (dx1!=0 or dy1!=0) else 90
            dx2 = p2[0] - p_left[0]
            dy2 = p2[1] - p_left[1]  
            angle2 = math.degrees(math.atan2(abs(dx2), abs(dy2))) if (dx2!=0 or dy2!=0) else 90
            if angle1 < angle2:
                best_angle = angle1
                best_edge_pts = (p_left, p1)
                opp_pt = p2
                best_dy = dy1
            else:
                best_angle = angle2
                best_edge_pts = (p_left, p2)
                opp_pt = p1
                best_dy = dy2
            if best_dy < 0: best_angle += 180.0
            return best_angle, box, best_edge_pts, opp_pt
        else: return 0.0, box, None, None
        
    elif shape_type == 'ti':
        # --- 【核爆级升级】：数学级平面对边识别算法，杜绝将斜边误认为平行边 ---
        approx = None
        # 使用动态逼近精度，强制剥离锯齿，逼出一个完美的 4 边形
        for eps_multiplier in np.arange(0.01, 0.15, 0.005):
            eps = eps_multiplier * cv2.arcLength(cnt, True)
            temp_approx = cv2.approxPolyDP(cnt, eps, True)
            if len(temp_approx) == 4:
                approx = temp_approx
                break
                
        if approx is not None:
            pts = approx.reshape(4, 2)
            edges = []
            
            # 记录4条边的属性 (起点，终点，长度，角度)
            for i in range(4):
                p1, p2 = pts[i], pts[(i+1)%4]
                dx, dy = p2[0] - p1[0], p2[1] - p1[1]
                length = max(1.0, math.hypot(dx, dy))
                # 角度映射到 0~180 度进行平行判定
                angle = math.degrees(math.atan2(dy, dx)) % 180
                edges.append({'p1': p1, 'p2': p2, 'len': length, 'ang': angle})
                
            # 计算互相平行的对边（梯形唯一的特征：必定有一组对边互相平行）
            # 对边组合 A: (0, 2)
            diff02 = abs(edges[0]['ang'] - edges[2]['ang'])
            if diff02 > 90: diff02 = 180 - diff02
            
            # 对边组合 B: (1, 3)
            diff13 = abs(edges[1]['ang'] - edges[3]['ang'])
            if diff13 > 90: diff13 = 180 - diff13
            
            # 角度差越小，说明越接近平行！找出真正的平行边
            if diff02 < diff13:
                e_a, e_b = edges[0], edges[2]
            else:
                e_a, e_b = edges[1], edges[3]
                
            # 区分这条平行边，谁是长边，谁是短边
            if e_a['len'] > e_b['len']:
                long_e, short_e = e_a, e_b
            else:
                long_e, short_e = e_b, e_a
                
            # 分别算出长边和短边的中点
            mid_long = ((long_e['p1'][0] + long_e['p2'][0]) / 2.0, (long_e['p1'][1] + long_e['p2'][1]) / 2.0)
            mid_short = ((short_e['p1'][0] + short_e['p2'][0]) / 2.0, (short_e['p1'][1] + short_e['p2'][1]) / 2.0)
            
            # 将这两个中点打包，UI在画箭头时会自动从 best_edge_pts[0] 画到 [1]，即【长边中点 指向 短边中点】！
            best_edge_pts = (mid_long, mid_short)
            
            # 抓取中心点 (完美落在长短边中点连线的中央)
            opp_pt = ((mid_long[0] + mid_short[0]) / 2.0, (mid_long[1] + mid_short[1]) / 2.0)
            
            # 计算夹爪的旋转角度，使其与中点连线完全一致
            dx = mid_short[0] - mid_long[0]
            dy = mid_short[1] - mid_long[1]
            best_angle = math.degrees(math.atan2(dy, dx))
            best_angle = (best_angle + 360) % 360
            
            return best_angle, box, best_edge_pts, opp_pt

        # --- 若图形破损太严重无法算出四边形，保留最初始的外接质心法兜底 ---
        M = cv2.moments(cnt)
        if M['m00'] == 0: return 0.0, box, None, None
        mcx = M['m10'] / M['m00']
        mcy = M['m01'] / M['m00']
        vx = cx - mcx
        vy = cy - mcy
        v1x, v1y = box[1][0] - box[0][0], box[1][1] - box[0][1]
        len1 = math.hypot(v1x, v1y)
        dir1x, dir1y = (v1x/len1, v1y/len1) if len1 > 0 else (0,0)
        v2x, v2y = box[2][0] - box[1][0], box[2][1] - box[1][1]
        len2 = math.hypot(v2x, v2y)
        dir2x, dir2y = (v2x/len2, v2y/len2) if len2 > 0 else (0,0)
        dot1 = abs(vx * dir1x + vy * dir1y)
        dot2 = abs(vx * dir2x + vy * dir2y)
        if math.hypot(vx, vy) < 1.0:
            if len1 < len2: sym_dir, sym_len = (dir1x, dir1y), len1
            else: sym_dir, sym_len = (dir2x, dir2y), len2
        else:
            if dot1 > dot2: sym_dir, sym_len = (dir1x, dir1y), len1
            else: sym_dir, sym_len = (dir2x, dir2y), len2
            if vx * sym_dir[0] + vy * sym_dir[1] < 0:
                sym_dir = (-sym_dir[0], -sym_dir[1])
        pt1 = (cx - sym_dir[0] * sym_len / 2.0, cy - sym_dir[1] * sym_len / 2.0)
        pt2 = (cx + sym_dir[0] * sym_len / 2.0, cy + sym_dir[1] * sym_len / 2.0)
        best_edge_pts = (pt1, pt2)
        opp_pt = (cx, cy)
        best_angle = math.degrees(math.atan2(sym_dir[1], sym_dir[0]))
        best_angle = (best_angle + 360) % 360
        return best_angle, box, best_edge_pts, opp_pt
        
    else:
        max_len = 0
        best_angle = 0
        for i in range(4):
            p1 = box[i]
            p2 = box[(i+1)%4]
            length = math.hypot(p2[0]-p1[0], p2[1]-p1[1])
            if length > max_len:
                max_len = length
                best_angle = math.degrees(math.atan2(p2[1]-p1[1], p2[0]-p1[0]))
        if best_angle < 0: best_angle += 180
        if best_angle >= 180: best_angle -= 180
        return best_angle, box, None, None




# =====================================================================
# 【4. 核心系统类：共享硬件，拆分双算法，合并串口逻辑】
# =====================================================================
class UnifiedSystemServer:
    def __init__(self):
        print("[INFO] 初始化双引擎视觉定位系统 (转盘微距 + YOLO 物件) ...")
        
        if not os.path.exists(SHAPE_MODEL_PATH) or not os.path.exists(MATERIAL_MODEL_PATH):
            print("[ERROR] 找不到模型文件，请检查模型路径！")
            exit(1)
            
        device = 'cuda:0' if USE_GPU and torch.cuda.is_available() else 'cpu'
        print(f"[INFO] YOLO使用设备: {device}")
        self.shape_model = YOLO(SHAPE_MODEL_PATH)
        self.material_model = YOLO(MATERIAL_MODEL_PATH)

        # 唯一共享的摄像头
        self.cap = cv2.VideoCapture(CAMERA_ID, cv2.CAP_DSHOW)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
            
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, -5)

        self.running = True
        self.latest_frame = None
        self.camera_ok = False
        self.frame_lock = threading.Lock()
        
        # --- 转盘数据隔离 ---
        self.turntable_target = None
        self.turntable_lock = threading.Lock()
        self.turntable_last_cmd = ""
        self.turntable_last_reply_time = 0
        
        # --- YOLO数据隔离 ---
        self.yolo_results = []
        self.yolo_lock = threading.Lock()
        self.yolo_requested_target = None
        self.yolo_last_cmd = ""
        self.yolo_last_reply_time = 0
        
        # 唯一共享的串口
        self.ser = None
        if not self.connect_serial(silent=False):
            print("[WARNING] 初始串口连接失败，系统将在后台自动尝试循环重连...")

    def connect_serial(self, silent=False):
        if self.ser is not None:
            try: self.ser.close()
            except: pass
        port_to_use = SERIAL_PORT
        if str(SERIAL_PORT).upper() == "AUTO":
            port_to_use = auto_detect_serial_port(keyword="USB", silent=silent)
        if port_to_use:
            try:
                self.ser = serial.Serial(port_to_use, BAUDRATE, timeout=0.01)
                if not silent or (silent and self.ser.is_open):
                    print(f"\n[INFO] >>> 成功建立串口连接: {port_to_use} @ {BAUDRATE} <<<")
                return True
            except Exception as e:
                if not silent: print(f"[ERROR] 无法打开串口 {port_to_use}: {e}")
                return False
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
                time.sleep(0.01)

    # ---------------- 隔离算法区 1：转盘检测 ----------------
    def detection_loop_turntable(self):
        while self.running:
            with self.frame_lock:
                if self.latest_frame is None or not self.camera_ok:
                    time.sleep(0.01)
                    continue
                frame = self.latest_frame.copy()

            target = find_best_single_slot(frame, min_area=1000)

            with self.turntable_lock:
                self.turntable_target = target
            time.sleep(0.02) 

    # ---------------- 隔离算法区 2：YOLO检测 ----------------
    def detection_loop_yolo(self):
        while self.running:
            with self.frame_lock:
                if self.latest_frame is None or not self.camera_ok:
                    time.sleep(0.01)
                    continue
                frame = self.latest_frame.copy()

            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            _, frame_thresh = cv2.threshold(frame_gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            shape_results = self.shape_model(frame, conf=CONF_THRESHOLD, verbose=False)[0]
            mat_results = self.material_model(frame, conf=CONF_THRESHOLD, verbose=False)[0]

            mat_boxes_info = []
            if getattr(mat_results, 'boxes', None) is not None and len(mat_results.boxes) > 0:
                m_boxes = mat_results.boxes.xyxy.cpu().numpy()
                m_classes = mat_results.boxes.cls.cpu().numpy()
                for j in range(len(m_boxes)):
                    m_cls_name = self.material_model.names[int(m_classes[j])]
                    mat_boxes_info.append({'box': tuple(m_boxes[j]), 'material': m_cls_name})

            boxes_xyxy_raw = []
            classes_raw = []
            confs_raw = []
            if getattr(shape_results, 'obb', None) is not None and len(shape_results.obb) > 0:
                boxes_xyxy_raw = shape_results.obb.xyxy.cpu().numpy()
                classes_raw = shape_results.obb.cls.cpu().numpy()
                confs_raw = shape_results.obb.conf.cpu().numpy()
            elif getattr(shape_results, 'boxes', None) is not None and len(shape_results.boxes) > 0:
                boxes_xyxy_raw = shape_results.boxes.xyxy.cpu().numpy()
                classes_raw = shape_results.boxes.cls.cpu().numpy()
                confs_raw = shape_results.boxes.conf.cpu().numpy()

            shape_names = self.shape_model.names
            boxes_xyxy, classes, confs = filter_smallest_boxes(boxes_xyxy_raw, classes_raw, confs_raw)

            all_detections = []
            for i in range(len(boxes_xyxy)):
                x1, y1, x2, y2 = boxes_xyxy[i]
                class_id = int(float(classes[i])) 
                shape_class = shape_names[class_id]
                conf_val = float(confs[i])
                matched_material = "Unknown"
                best_iou = 0
                for mat_info in mat_boxes_info:
                    iou = get_iou((x1, y1, x2, y2), mat_info['box'])
                    if iou > best_iou and iou > 0.4:
                        best_iou = iou
                        matched_material = mat_info['material']
                final_tag = shape_class
                if shape_class == 'ti':   
                    if matched_material == 'plastic': final_tag = 'pt'
                    elif matched_material == 'metal': final_tag = 'mt'
                elif shape_class == 'san': 
                    if matched_material == 'plastic': final_tag = 'ps'
                    elif matched_material == 'metal': final_tag = 'ms'
                best_edge_global = None
                box_global = None
                opp_pt_global = None
                if shape_class == 'yuan': angle_deg = 0.0
                else:
                    angle_deg, box_global, best_edge_global, opp_pt_global = process_roi_geometry(frame_thresh, x1, y1, x2, y2, shape_class)
                    angle_deg = angle_deg % 360.0
                angle_norm = round(angle_deg / 360.0, 4)
                area = (x2 - x1) * (y2 - y1)
                all_detections.append({
                    "raw_box": (x1, y1, x2, y2), "shape_class": shape_class, "final_tag": final_tag,
                    "conf": conf_val, "area": area, "angle_norm": angle_norm,
                    "best_edge_global": best_edge_global, "opp_pt_global": opp_pt_global, "box_global": box_global
                })

            new_results = []
            for det in all_detections:
                x1, y1, x2, y2 = det['raw_box']
                shape_class = det['shape_class']
                w_orig = x2 - x1; h_orig = y2 - y1
                p_proj_global = None; new_center = None
                if shape_class == 'san' and det['best_edge_global'] is not None and det['opp_pt_global'] is not None:
                    P0 = np.array(det['best_edge_global'][0], dtype=float)
                    P1 = np.array(det['best_edge_global'][1], dtype=float)
                    P2 = np.array(det['opp_pt_global'], dtype=float)
                    v = P1 - P0; w = P2 - P0; v_sq = np.dot(v, v)
                    if v_sq > 0:
                        t = np.dot(w, v) / v_sq; P_proj = P0 + t * v
                    else: P_proj = P0
                    C_new = (P2 + P_proj) / 2.0; cx, cy = C_new[0], C_new[1]
                    p_proj_global = (int(P_proj[0]), int(P_proj[1])); new_center = (int(cx), int(cy))
                elif shape_class == 'ti' and det['opp_pt_global'] is not None:
                    cx, cy = det['opp_pt_global']; new_center = (int(cx), int(cy))
                else:
                    cx = x1 + w_orig / 2.0; cy = y1 + h_orig / 2.0

                if shape_class in ['san', 'ti']:
                    side = max(w_orig, h_orig)
                    x_final = int(cx - side / 2.0); y_final = int(cy - side / 2.0)
                    w_final = h_final = int(side)
                else:
                    x_final = int(x1); y_final = int(y1); w_final = int(w_orig); h_final = int(h_orig)

                new_results.append({
                    "x": x_final, "y": y_final, "w": w_final, "h": h_final,
                    "angle_norm": det['angle_norm'], "tag": det['final_tag'], "conf": det['conf'],
                    "area": det['area'], "shape_class": shape_class,
                    "best_edge_global": det['best_edge_global'], "opp_pt_global": det['opp_pt_global'],
                    "p_proj_global": p_proj_global, "new_center": new_center, "box_global": det['box_global']
                })
            with self.yolo_lock:
                self.yolo_results = new_results
            time.sleep(0.03)

    # ---------------- 请求处理与封包逻辑保留 ----------------
    def encode_detections(self):
        if not self.yolo_results: return "" 
        parts = []
        for d in self.yolo_results:
            parts.append(f"{d['x']},{d['y']},{d['w']},{d['h']},{d['tag']}:{d['conf']:.4f}:{d['angle_norm']:.4f}")
        return "][".join(parts)

    def handle_request_turntable(self, req_code):
        if not self.camera_ok or self.latest_frame is None:
            return bytes([ASCII_STX]) + b"[]" + bytes([ASCII_ETX])
        if req_code == ord('7'):
            self.turntable_last_cmd = '7'
            self.turntable_last_reply_time = time.time()
            with self.turntable_lock: target = self.turntable_target
            if target is None: return bytes([ASCII_STX]) + b"[]" + bytes([ASCII_ETX])
            bx, by, bw, bh = target.bbox
            tag = target.color_name
            tcx, tcy = map(int, target.center)
            data_str = f"[[{tcx},{tcy},{bw},{bh},{tag}:1.00:0.00]]"
            return bytes([ASCII_STX]) + data_str.encode('utf-8') + bytes([ASCII_ETX])
        return bytes([ASCII_STX]) + b"[]" + bytes([ASCII_ETX])

    def handle_request_yolo(self, req_code):
        if not self.camera_ok or self.latest_frame is None:
            return bytes([ASCII_STX]) + b"[]" + bytes([ASCII_ETX])
        if req_code == 0x46:
            self.yolo_requested_target = "ALL (0x46)"
            self.yolo_last_cmd = "ALL"
            self.yolo_last_reply_time = time.time()
            encoded_str = self.encode_detections()
            data_str = "[" + encoded_str + "]" if encoded_str else "[]"
            return bytes([ASCII_STX]) + "[".encode('utf-8') + data_str.encode('utf-8')+ "]".encode('utf-8') + bytes([ASCII_ETX])
        
        target_tag = CMD_MAP.get(req_code, None)
        self.yolo_requested_target = target_tag 
        if target_tag is None: return bytes([ASCII_STX]) + b"[]" + bytes([ASCII_ETX])
        self.yolo_last_cmd = target_tag
        with self.yolo_lock: matches = [d for d in self.yolo_results if d['tag'] == target_tag]
        if not matches: return bytes([ASCII_STX]) + b"[]" + bytes([ASCII_ETX])
        
        matches.sort(key=lambda x: x['area'])
        best = matches[0]
        data_str = f"[{best['x']},{best['y']},{best['w']},{best['h']},{best['tag']}:{best['conf']:.4f}:{best['angle_norm']:.4f}]"
        self.yolo_last_reply_time = time.time()
        return bytes([ASCII_STX]) + "[".encode('utf-8') + data_str.encode('utf-8')+ "]".encode('utf-8') + bytes([ASCII_ETX])

    # ---------------- 关键修改区：融合分发的串口引擎 ----------------
    def serial_loop(self):
        retry_interval = 2.0  
        send_interval = 0.1  
        last_push_time = time.time()
        
        while self.running:
            if self.ser is None or not self.ser.is_open:
                if self.connect_serial(silent=True): pass
                else:
                    time.sleep(retry_interval)
                    continue
            
            try:
                if self.ser.in_waiting:
                    byte = self.ser.read(1)
                    
                    if byte[0] == ASCII_SYN:
                        self.ser.write(bytes([ASCII_ACK]))
                        continue
                        
                    if byte[0] == ord('7'):
                        print(f"\n[POLL] 收到指令 '7' -> 转盘微距极精定位请求")
                        response = self.handle_request_turntable(byte[0])
                        self.ser.write(response)
                        self.ser.flush()
                        last_push_time = time.time() 
                        continue

                    if byte[0] == ASCII_STX:
                        data = bytearray()
                        while True:
                            b = self.ser.read(1)
                            if not b: break
                            data.append(b[0])
                            if b[0] == ASCII_ETX: break
                            
                        if len(data) >= 1 and data[-1] == ASCII_ETX:
                            req_code = data[0] if len(data) > 0 else 0
                            req_name = "ALL SHAPES (0x46)" if req_code == 0x46 else CMD_MAP.get(req_code, f"Unknown(0x{req_code:02X})")
                            print(f"[POLL] 收到包头指令: 意图 -> {req_name}")
                            
                            response = self.handle_request_yolo(req_code)
                            self.ser.write(response)
                            self.ser.flush()
                            last_push_time = time.time() 
                            continue

                current_time = time.time()
                if current_time - last_push_time >= send_interval:
                    if not self.camera_ok or self.latest_frame is None:
                        packet = bytes([ASCII_STX]) + b"[]" + bytes([ASCII_ETX])
                    else:
                        encoded_str = self.encode_detections()
                        data_str = "[" + encoded_str + "]" if encoded_str else "[]"
                        packet = bytes([ASCII_STX]) +  "[".encode('utf-8') + data_str.encode('utf-8')+ "]".encode('utf-8') + bytes([ASCII_ETX])
                    
                    self.ser.write(packet)
                    self.ser.flush()
                    last_push_time = current_time
                else:
                    time.sleep(0.005) 
                    
            except (serial.SerialException, OSError) as e:
                print(f"\n[WARNING] 串口连接断开，异常: {e}")
                if self.ser:
                    try: self.ser.close()
                    except: pass
                self.ser = None
                time.sleep(1)
            except Exception as e:
                time.sleep(1)

    # ---------------- 整体UI绘制与进程挂载 ----------------
    def start(self):
        t_cap = threading.Thread(target=self.capture_loop, daemon=True)
        t_cap.start()
        
        t_det_turn = threading.Thread(target=self.detection_loop_turntable, daemon=True)
        t_det_turn.start()
        
        t_det_yolo = threading.Thread(target=self.detection_loop_yolo, daemon=True)
        t_det_yolo.start()
        
        t_serial = threading.Thread(target=self.serial_loop, daemon=True)
        t_serial.start()

        win_name = "Unified Double-Engine Vision Server"
        cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)

        print("[INFO] 系统已全面启动，后台算法与双通讯链路运转中。按 'q' 键退出。")

        while self.running:
            with self.frame_lock:
                frame_disp = self.latest_frame.copy() if self.latest_frame is not None else None
            
            if frame_disp is None:
                cv2.waitKey(1)
                continue
                
            h, w = frame_disp.shape[:2]
            img_cx, img_cy = int(w / 2), int(h / 2)

            with self.turntable_lock:
                target = self.turntable_target
                
            cv2.drawMarker(frame_disp, (img_cx, img_cy), (200, 200, 200), markerType=cv2.MARKER_CROSS, markerSize=w, thickness=1)
            cv2.circle(frame_disp, (img_cx, img_cy), 3, (255, 255, 255), -1)

            if target is not None:
                color = COLOR_BGR_MAP.get(target.color_name, (0, 255, 0))
                tcx, tcy = map(int, target.center)
                bx, by, bw, bh = target.bbox
                cv2.rectangle(frame_disp, (bx, by), (bx + bw, by + bh), color, 3)
                cv2.circle(frame_disp, (tcx, tcy), 6, (0, 0, 255), -1)
                cv2.drawMarker(frame_disp, (tcx, tcy), (0, 0, 255), markerType=cv2.MARKER_TILTED_CROSS, markerSize=30, thickness=3)
                cv2.line(frame_disp, (img_cx, img_cy), (tcx, tcy), (0, 255, 255), 2, lineType=cv2.LINE_AA)
                label = f"LOCKED: {target.color_name.upper()} SLOT"
                cv2.putText(frame_disp, label, (bx, max(20, by - 10)), cv2.FONT_HERSHEY_DUPLEX, 0.6, color, 2)
                
                if time.time() - self.turntable_last_reply_time < 1.5:
                    cv2.putText(frame_disp, "[🎯 MCU REQUESTED '7']", (bx, max(45, by - 35)), cv2.FONT_HERSHEY_DUPLEX, 0.7, (0, 0, 255), 2)
                    dx = tcx - img_cx
                    dy = tcy - img_cy
                    cv2.putText(frame_disp, f"Error: X {dx:+03d} | Y {dy:+03d}", (20, 40), cv2.FONT_HERSHEY_DUPLEX, 0.8, (0, 255, 255), 2)
            else:
                cv2.putText(frame_disp, "NO SLOT LOCKED", (img_cx - 120, img_cy - 20), cv2.FONT_HERSHEY_DUPLEX, 0.8, (0, 0, 255), 2)
            
            req_text_turn = "7 (Center Slot)" if time.time() - self.turntable_last_reply_time < 1.5 else "Waiting..."
            cv2.putText(frame_disp, f"Turn Trigger: {req_text_turn}", (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            with self.yolo_lock:
                current_detections = self.yolo_results.copy()

            target_highlight = None
            if self.yolo_requested_target and self.yolo_requested_target != "ALL (0x46)":
                matches = [d for d in current_detections if d['tag'] == self.yolo_requested_target]
                if matches:
                    matches.sort(key=lambda x: x['area'])
                    target_highlight = matches[0]

            for d in current_detections:
                x, y, det_w, det_h = d['x'], d['y'], d['w'], d['h']
                angle_norm = d['angle_norm']
                tag = d['tag']
                conf_val = d.get('conf', 0.0)
                shape_class = d.get('shape_class', '')
                best_edge_global = d.get('best_edge_global', None)
                opp_pt_global = d.get('opp_pt_global', None)
                p_proj_global = d.get('p_proj_global', None)
                new_center = d.get('new_center', None)
                box_global = d.get('box_global', None)
                
                color = COLOR_MAP.get(tag, (255,255,255))
                cv2.rectangle(frame_disp, (x, y), (x+det_w, y+det_h), color, 2)
                
                if box_global is not None:
                    cv2.polylines(frame_disp, [box_global], isClosed=True, color=(255, 255, 0), thickness=1)

                if shape_class == 'ti' and best_edge_global is not None:
                    cv2.arrowedLine(frame_disp, best_edge_global[0], best_edge_global[1], (0, 0, 255), 4, tipLength=0.2)
                    if new_center is not None:
                        cv2.circle(frame_disp, new_center, 5, (0, 255, 255), -1)
                        cv2.putText(frame_disp, "Midpoint(0deg->)", (new_center[0] + 10, new_center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 2)
                        
                elif shape_class == 'san' and best_edge_global is not None:
                    anchor_pt = best_edge_global[0]
                    line_x = anchor_pt[0]
                    y_top = max(0, y - 20)
                    y_bot = min(FRAME_HEIGHT, y + det_h + 20)
                    cv2.line(frame_disp, (line_x, y_top), (line_x, y_bot), (0, 255, 255), 2)
                    cv2.putText(frame_disp, "Left Anchor", (line_x - 10, y_top - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
                    cv2.line(frame_disp, best_edge_global[0], best_edge_global[1], (0, 0, 255), 4)
                    
                    if p_proj_global is not None and opp_pt_global is not None:
                        cv2.line(frame_disp, opp_pt_global, p_proj_global, (255, 0, 255), 2)
                    if new_center is not None:
                        cv2.circle(frame_disp, new_center, 5, (0, 255, 255), -1)
                        cv2.putText(frame_disp, "Grasp Center", (new_center[0] + 10, new_center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 2)

                label = f"{tag} {conf_val:.2f} Ang:{angle_norm:.4f}"
                cv2.putText(frame_disp, label, (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                
                if target_highlight and d == target_highlight and d['tag'] == self.yolo_last_cmd and time.time() - self.yolo_last_reply_time < 1.5:
                    cv2.putText(frame_disp, "[🎯 MIN TARGET SENT]", (x, max(15, y - 25)), cv2.FONT_HERSHEY_DUPLEX, 0.7, (0, 0, 255), 2)
                            
            req_text_yolo = self.yolo_requested_target if self.yolo_requested_target else "Waiting for Request..."
            cv2.putText(frame_disp, f"YOLO Trigger: {req_text_yolo}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            if not self.camera_ok:
                cv2.putText(frame_disp, "CAMERA ERROR!", (FRAME_WIDTH//2 - 150, FRAME_HEIGHT//2), cv2.FONT_HERSHEY_DUPLEX, 1.5, (0, 0, 255), 3)

            cv2.imshow(win_name, frame_disp)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.running = False
                break

        self.cap.release()
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    server = UnifiedSystemServer()
    server.start()