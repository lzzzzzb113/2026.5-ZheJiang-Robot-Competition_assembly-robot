import cv2
import serial
import serial.tools.list_ports
import threading
import time
import math
import numpy as np
import torch
import os
import itertools  # 新增：用于三角形的组合搜索
from ultralytics import YOLO

# ==================== 配置 ====================
# 【模型路径】
SHAPE_MODEL_PATH = "shape_grasp_project/runs/5184.pt"      
MATERIAL_MODEL_PATH = "shape_grasp_project/runs/5183m.pt"  
CAMERA_ID = 1

# 【串口配置】
SERIAL_PORT = "AUTO" 
BAUDRATE = 9600

# 串口稳定性配置：识别核心逻辑不变，只增强收发可靠性。
# 单片机采用“发送指令 -> 等待回复”的轮询方式时，建议关闭主动推送，避免串口缓冲区里混入全量数据包。
ENABLE_ACTIVE_PUSH = False
ACTIVE_PUSH_INTERVAL = 0.1
SERIAL_TIMEOUT = 0.05
SERIAL_WRITE_TIMEOUT = 0.2
SERIAL_FRAME_TIMEOUT = 0.25
SERIAL_INTER_BYTE_SLEEP = 0.001
RESPONSE_GUARD_TIME = 0.20

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
CONF_THRESHOLD = 0.25
USE_GPU = True

# 严格保留的同步指令
ASCII_STX = 0x02
ASCII_ETX = 0x03
ASCII_ACK = 0x06
ASCII_SYN = 0x16

COLOR_MAP = {
    # 颜色改为与第一个代码 SHAPE_COLOR 一致
    'yuan': (0, 255, 255),
    'si': (0, 165, 255),
    'ms': (100, 255, 100),
    'ps': (0, 200, 0),
    'mt': (100, 100, 255),
    'pt': (0, 0, 200),
    'san': (0, 255, 0),
    'ti': (255, 0, 0)
}

# ============================================
# 【单片机指令映射表 (ASCII码)】
# 单片机发送STX + 下列字符 + ETX，系统将返回对应物体中面积最小的一个
CMD_MAP = {
    ord('1'): 'ps',   
    ord('2'): 'ms',   
    ord('3'): 'pt',   
    ord('4'): 'mt',   
    ord('5'): 'si',   
    ord('6'): 'yuan', 
}
# ============================================

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

# ==================== 融合 Big Ultra 的几何计算逻辑 ====================
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
        
        # 使用穷举法寻找最贴合的三角形组合
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
                    if iou > best_iou: best_iou, best_triangle = iou, tri_pts
                    
        if best_triangle is not None:
            triangle = best_triangle + np.array([x, y])
            triangle = np.expand_dims(triangle, axis=1)
        else:
            _, triangle = cv2.minEnclosingTriangle(cnt)
            
        if triangle is not None and triangle.size == 6:
            pts = triangle.reshape(3, 2)
            left_idx = np.argmin(pts[:, 0])
            p_left = pts[left_idx]
            p1, p2 = pts[(left_idx + 1) % 3], pts[(left_idx + 2) % 3]
            
            dx1, dy1 = p1[0] - p_left[0], p1[1] - p_left[1]
            angle1 = math.degrees(math.atan2(abs(dx1), abs(dy1))) if (dx1 or dy1) else 90
            dx2, dy2 = p2[0] - p_left[0], p2[1] - p_left[1]
            angle2 = math.degrees(math.atan2(abs(dx2), abs(dy2))) if (dx2 or dy2) else 90
            
            if angle1 < angle2:
                best_angle, best_edge_pts, opp_pt = angle1, (tuple(map(int, p_left)), tuple(map(int, p1))), tuple(map(int, p2))
                best_dy = dy1
            else:
                best_angle, best_edge_pts, opp_pt = angle2, (tuple(map(int, p_left)), tuple(map(int, p2))), tuple(map(int, p1))
                best_dy = dy2
                
            if best_dy < 0: best_angle += 180.0
            return best_angle, box, best_edge_pts, opp_pt
        else:
            return 0.0, box, None, None

    elif shape_type == 'ti':
        hull = cv2.convexHull(cnt)
        peri = cv2.arcLength(hull, True)
        trap_pts = None

        # 动态 eps 强制逼近四边形
        for eps_rate in [0.01, 0.015, 0.02, 0.025, 0.03, 0.04, 0.05, 0.06, 0.08]:
            approx = cv2.approxPolyDP(hull, eps_rate * peri, True)
            if len(approx) == 4:
                trap_pts = approx.reshape(4, 2).astype(np.float32)
                break

        if trap_pts is None:
            trap_pts = cv2.boxPoints(rect).astype(np.float32)

        center = np.mean(trap_pts, axis=0)
        trap_pts = np.array(
            sorted(trap_pts, key=lambda p: math.atan2(p[1] - center[1], p[0] - center[0])),
            dtype=np.float32
        )

        def edge_angle_local(p1, p2):
            ang = math.degrees(math.atan2(p2[1] - p1[1], p2[0] - p1[0]))
            return ang % 180.0

        def angle_diff_180_local(a, b):
            d = abs(a - b) % 180.0
            return min(d, 180.0 - d)

        def midpoint_local(p1, p2):
            return (int(round((p1[0] + p2[0]) / 2.0)), int(round((p1[1] + p2[1]) / 2.0)))

        def edge_length_local(p1, p2):
            return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

        edges = [
            (trap_pts[0], trap_pts[1]),
            (trap_pts[1], trap_pts[2]),
            (trap_pts[2], trap_pts[3]),
            (trap_pts[3], trap_pts[0]),
        ]

        pair_02 = (edges[0], edges[2])
        pair_13 = (edges[1], edges[3])

        def pair_score(pair):
            e1, e2 = pair
            a1 = edge_angle_local(e1[0], e1[1])
            a2 = edge_angle_local(e2[0], e2[1])
            parallel_error = angle_diff_180_local(a1, a2)
            m1 = midpoint_local(e1[0], e1[1])
            m2 = midpoint_local(e2[0], e2[1])
            vertical_sep = abs(m1[1] - m2[1])
            return parallel_error, vertical_sep

        err02, sep02 = pair_score(pair_02)
        err13, sep13 = pair_score(pair_13)

        if abs(err02 - err13) < 8:
            base_pair = pair_02 if sep02 >= sep13 else pair_13
        else:
            base_pair = pair_02 if err02 < err13 else pair_13

        base1, base2 = base_pair
        mid1 = midpoint_local(base1[0], base1[1])
        mid2 = midpoint_local(base2[0], base2[1])
        len1 = edge_length_local(base1[0], base1[1])
        len2 = edge_length_local(base2[0], base2[1])

        # 取较长平行边作为主轴基准
        if len1 >= len2:
            start_pt, end_pt = mid1, mid2
        else:
            start_pt, end_pt = mid2, mid1

        best_edge_pts = (start_pt, end_pt)
        opp_pt = end_pt
        best_angle = math.degrees(math.atan2(end_pt[1] - start_pt[1], end_pt[0] - start_pt[0]))
        trap_box = trap_pts.astype(np.int32)
        return best_angle, trap_box, best_edge_pts, opp_pt

    # 四边形兜底逻辑
    max_len, best_angle = 0, 0
    for i in range(4):
        p1, p2 = box[i], box[(i + 1) % 4]
        length = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        if length > max_len:
            max_len, best_angle = length, math.degrees(math.atan2(p2[1] - p1[1], p2[0] - p1[0]))
    
    if best_angle < 0: best_angle += 180
    if best_angle >= 180: best_angle -= 180
    return best_angle, box, None, None
# =========================================================================

def process_roi_geometry(gray_frame, x1, y1, x2, y2, shape_type):
    """移植自第一个代码：在 YOLO 原始框内重新提取轮廓，返回真实多边形、角度边和标记点。"""
    rx1, ry1 = max(0, int(x1)), max(0, int(y1))
    rx2, ry2 = int(x2), int(y2)
    roi_gray = gray_frame[ry1:ry2, rx1:rx2]
    
    if roi_gray.size == 0:
        return 0.0, None, None, None
        
    _, roi_thresh = cv2.threshold(roi_gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    contours, _ = cv2.findContours(roi_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        return 0.0, None, None, None
        
    cnt = max(contours, key=cv2.contourArea)
    if len(cnt) < 3:
        return 0.0, None, None, None
        
    precise_angle, box_local, best_edge_local, opp_pt_local = get_robust_angle_and_box(cnt, shape_type)
    
    box_global = np.int32(box_local + np.array([rx1, ry1])) if box_local is not None else None
    
    if best_edge_local is not None:
        best_edge_global = (
            tuple(map(int, np.array(best_edge_local[0]) + [rx1, ry1])),
            tuple(map(int, np.array(best_edge_local[1]) + [rx1, ry1]))
        )
    else:
        best_edge_global = None
        
    opp_pt_global = tuple(map(int, np.array(opp_pt_local) + [rx1, ry1])) if opp_pt_local else None
    
    return precise_angle, box_global, best_edge_global, opp_pt_global

class YOLOSerialServer:
    def __init__(self):
        if not os.path.exists(SHAPE_MODEL_PATH) or not os.path.exists(MATERIAL_MODEL_PATH):
            print("[ERROR] 找不到模型文件，请检查模型路径！")
            exit(1)
            
        device = 'cuda:0' if USE_GPU and torch.cuda.is_available() else 'cpu'
        print(f"[INFO] 使用设备: {device}")
        self.shape_model = YOLO(SHAPE_MODEL_PATH)
        self.material_model = YOLO(MATERIAL_MODEL_PATH)

        self.cap = cv2.VideoCapture(CAMERA_ID, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, -4)

        self.running = True
        self.latest_frame = None
        self.camera_ok = False
        self.frame_lock = threading.Lock()
        
        self.detection_results = []
        self.results_lock = threading.Lock()
        
        # 记录指令信息用于 UI 渲染
        self.requested_target = None
        self.last_cmd = ""
        self.last_reply_time = 0
        
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
                # timeout 原来是 0.01，单片机发送稍慢时容易读不到 ETX，导致整包被丢弃。
                self.ser = serial.Serial(
                    port_to_use,
                    BAUDRATE,
                    timeout=SERIAL_TIMEOUT,
                    write_timeout=SERIAL_WRITE_TIMEOUT,
                    inter_byte_timeout=SERIAL_TIMEOUT
                )
                try:
                    self.ser.reset_input_buffer()
                    self.ser.reset_output_buffer()
                except Exception:
                    pass
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

    def detection_loop(self):
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

            new_results = []
            h_frame, w_frame = frame.shape[:2]
            for i in range(len(boxes_xyxy)):
                x1, y1, x2, y2 = map(int, boxes_xyxy[i])
                x1, y1 = max(0, x1), max(0, y1)
                x2, y2 = min(w_frame, x2), min(h_frame, y2)
                if x2 <= x1 or y2 <= y1:
                    continue

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
                material_lower = str(matched_material).lower()
                if shape_class == 'ti':
                    if 'plastic' in material_lower:
                        final_tag = 'pt'
                    elif 'metal' in material_lower:
                        final_tag = 'mt'
                elif shape_class == 'san':
                    if 'plastic' in material_lower:
                        final_tag = 'ps'
                    elif 'metal' in material_lower:
                        final_tag = 'ms'

                angle_deg = 0.0
                box_global = None
                best_edge_global = None
                opp_pt_global = None

                # 三角形、梯形：使用第一个代码的识别方框、角度边和标记点。
                if shape_class in ['san', 'ti']:
                    angle_deg, box_global, best_edge_global, opp_pt_global = process_roi_geometry(
                        frame_gray, x1, y1, x2, y2, shape_class
                    )
                    angle_deg = angle_deg % 360.0
                elif shape_class == 'si':
                    angle_deg, box_global, _, _ = process_roi_geometry(frame_gray, x1, y1, x2, y2, 'si')
                    angle_deg = angle_deg % 90.0

                angle_norm = angle_deg / 360.0
                w_orig = int(x2 - x1)
                h_orig = int(y2 - y1)
                area = w_orig * h_orig

                new_results.append({
                    "x": int(x1), "y": int(y1), "w": w_orig, "h": h_orig,
                    "angle_norm": angle_norm,
                    "tag": final_tag,
                    "conf": conf_val,
                    "area": area,
                    "shape_class": shape_class,
                    "best_edge_global": best_edge_global,
                    "opp_pt_global": opp_pt_global,
                    "p_proj_global": None,
                    "new_center": None,
                    "box_global": box_global
                })

            with self.results_lock:
                self.detection_results = new_results

            time.sleep(0.03)

    def encode_detections(self):
        """0x46使用：将当前所有物块打包成字符串"""
        if not self.detection_results: return "" 
        parts = []
        for d in self.detection_results:
            parts.append(f"{d['x']},{d['y']},{d['w']},{d['h']},{d['tag']}:{d['conf']:.4f}:{d['angle_norm']:.4f}")
        encoded = "][".join(parts)
        return encoded

    def build_packet(self, payload_str="[]", wrap_outer=False):
        """统一打包串口数据。

        wrap_outer=True 时，保持 serial_comf（20） 这版代码的发送格式：
        STX + "[" + payload + "]" + ETX。
        注意：payload 本身已经带 []，所以单个目标会发成 [[x,y,w,h,tag:conf:angle]]。
        wrap_outer=False 时用于无目标/异常，发送 STX + [] + ETX。
        """
        if not payload_str:
            payload_str = "[]"
        payload = str(payload_str)
        if wrap_outer:
            payload = "[" + payload + "]"
        return bytes([ASCII_STX]) + payload.encode('utf-8') + bytes([ASCII_ETX])

    def read_request_payload(self):
        """读取 STX 后面的 payload，直到 ETX。避免因为串口分字节到达导致半包丢失。"""
        data = bytearray()
        deadline = time.time() + SERIAL_FRAME_TIMEOUT
        while self.running and time.time() < deadline:
            b = self.ser.read(1)
            if b:
                data.append(b[0])
                if b[0] == ASCII_ETX:
                    return data
            else:
                time.sleep(SERIAL_INTER_BYTE_SLEEP)
        return data

    def safe_write_packet(self, packet, log_prefix="[SERIAL]"):
        """串口写入统一出口：写完 flush，失败时抛出异常交给外层重连。"""
        if self.ser is None or not self.ser.is_open:
            raise serial.SerialException("serial port is not open")
        self.ser.write(packet)
        self.ser.flush()
        print(f"{log_prefix} 发送: {packet[:80]}{'...' if len(packet) > 80 else ''}")

    def handle_request(self, req_code):
        # 1. 摄像头断线检查 (返回空列表而不直接发 0x46，避免扰乱通信协议)
        if not self.camera_ok or self.latest_frame is None:
            print("[ERROR] 摄像头异常，返回空数据")
            return self.build_packet("[]")

        # 2. 0x46 获取全部数据
        if req_code == 0x46:
            self.requested_target = "ALL (0x46)"
            self.last_cmd = "ALL"
            self.last_reply_time = time.time()
            
            encoded_str = self.encode_detections()
            data_str = "[" + encoded_str + "]" if encoded_str else "[]"
            return self.build_packet(data_str, wrap_outer=True)

        # 3. 指定图形拉取 (单片机发送 1~6 字符)
        target_tag = CMD_MAP.get(req_code, None)
        self.requested_target = target_tag 

        if target_tag is None:
            print(f"[WARNING] 未知请求码: 0x{req_code:02X}，返回空数据")
            return self.build_packet("[]")

        self.last_cmd = target_tag

        with self.results_lock:
            matches = [d for d in self.detection_results if d['tag'] == target_tag]

        if not matches:
            print(f"[POLL] 当前没有检测到目标 {target_tag}，返回空数据")
            return self.build_packet("[]")

        # 【核心逻辑】：同类物体，找面积最小的！
        # area 越小，排序越靠前，所以 matches[0] 即为最小的
        matches.sort(key=lambda x: x['area'])
        best = matches[0]

        data_str = f"[{best['x']},{best['y']},{best['w']},{best['h']},{best['tag']}:{best['conf']:.4f}:{best['angle_norm']:.4f}]"
        self.last_reply_time = time.time()
        
        return self.build_packet(data_str, wrap_outer=True)

    def serial_loop(self):
        retry_interval = 2.0
        send_interval = ACTIVE_PUSH_INTERVAL
        last_push_time = time.time()
        
        while self.running:
            if self.ser is None or not self.ser.is_open:
                if self.connect_serial(silent=True): pass
                else:
                    time.sleep(retry_interval)
                    continue
            
            try:
                # 【严格保留 STX/ETX 包头包尾解析规则和 SYN/ACK 握手协议】
                if self.ser.in_waiting:
                    byte = self.ser.read(1)
                    if not byte:
                        time.sleep(0.002)
                        continue
                    
                    if byte[0] == ASCII_SYN:
                        self.safe_write_packet(bytes([ASCII_ACK]), log_prefix="[HANDSHAKE]")
                        print("Handshake OK (SYN->ACK)")
                        last_push_time = time.time()
                        continue
                        
                    if byte[0] == ASCII_STX:
                        data = self.read_request_payload()
                            
                        if len(data) >= 1 and data[-1] == ASCII_ETX:
                            # 提取单片机发来的有效请求码，ETX 不参与解析。
                            payload = data[:-1]
                            if not payload:
                                print("[WARNING] 收到空请求包，已忽略")
                                continue

                            req_code = payload[0]
                            req_name = "ALL SHAPES (0x46)" if req_code == 0x46 else CMD_MAP.get(req_code, f"Unknown(0x{req_code:02X})")
                            print(f"[POLL] 收到请求指令: '{chr(req_code) if 32 <= req_code <= 126 else hex(req_code)}' -> 意图: {req_name}")
                            
                            response = self.handle_request(req_code)
                            self.safe_write_packet(response, log_prefix="[POLL]")
                            last_push_time = time.time()
                            continue
                        else:
                            print(f"[WARNING] 收到不完整串口包，已丢弃: {bytes(data)}")
                            continue

                    # 非 SYN / 非 STX 的字节可能是上一次残留数据，丢弃即可，避免污染下一包。
                    print(f"[WARNING] 丢弃无效串口字节: 0x{byte[0]:02X}")

                # 主动推送默认关闭，避免单片机轮询时读到旧的全量包而不是刚请求的目标包。
                # 如确实需要电脑端 10Hz 主动输出，将 ENABLE_ACTIVE_PUSH 改为 True。
                current_time = time.time()
                if ENABLE_ACTIVE_PUSH and current_time - last_push_time >= send_interval:
                    # 刚刚回复过单片机时留一个保护时间，避免主动包紧跟着回复包进入 MCU 接收缓冲。
                    if current_time - self.last_reply_time < RESPONSE_GUARD_TIME:
                        time.sleep(0.005)
                        continue

                    if not self.camera_ok or self.latest_frame is None:
                        packet = self.build_packet("[]")
                    else:
                        encoded_str = self.encode_detections()
                        data_str = "[" + encoded_str + "]" if encoded_str else "[]"
                        packet = self.build_packet(data_str, wrap_outer=True)
                    
                    self.safe_write_packet(packet, log_prefix="[PUSH]")
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
                print(f"[WARNING] 串口循环异常: {e}")
                time.sleep(0.1)

    def start(self):
        t_cap = threading.Thread(target=self.capture_loop, daemon=True)
        t_cap.start()
        t_det = threading.Thread(target=self.detection_loop, daemon=True)
        t_det.start()
        t_serial = threading.Thread(target=self.serial_loop, daemon=True)
        t_serial.start()

        print("Server started. Press 'q' to quit.")
        while self.running:
            with self.frame_lock:
                if self.latest_frame is not None:
                    frame_disp = self.latest_frame.copy()
                else:
                    frame_disp = None
            if frame_disp is None:
                cv2.waitKey(1)
                continue

            with self.results_lock:
                current_detections = self.detection_results.copy()

            target_highlight = None
            if self.requested_target and self.requested_target != "ALL (0x46)":
                matches = [d for d in current_detections if d['tag'] == self.requested_target]
                if matches:
                    # 同样为了 UI 高亮一致性，标出那个面积最小的！
                    matches.sort(key=lambda x: x['area'])
                    target_highlight = matches[0]

            for d in current_detections:
                x, y, w, h = d['x'], d['y'], d['w'], d['h']
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
                
                cv2.rectangle(frame_disp, (x, y), (x+w, y+h), color, 2)
                
                # 三角形/梯形显示逻辑改为第一个代码的样式：
                # 识别方框=YOLO原始框；真实轮廓=黄色多边形；角度=红色箭头；标记点=绿色圆点。
                if shape_class in ['san', 'ti']:
                    if box_global is not None:
                        cv2.polylines(frame_disp, [np.int32(box_global)], True, (0, 255, 255), 2)
                    if best_edge_global is not None:
                        cv2.arrowedLine(frame_disp, best_edge_global[0], best_edge_global[1], (0, 0, 255), 3, tipLength=0.2)
                        if shape_class == 'san':
                            anchor_x = int(best_edge_global[0][0])
                            cv2.line(
                                frame_disp,
                                (anchor_x, max(0, y - 20)),
                                (anchor_x, min(FRAME_HEIGHT, y + h + 20)),
                                (0, 255, 255),
                                2
                            )
                    if opp_pt_global is not None:
                        cv2.circle(frame_disp, opp_pt_global, 5, (0, 255, 0), -1)
                elif box_global is not None:
                    cv2.polylines(frame_disp, [np.int32(box_global)], isClosed=True, color=(255, 255, 0), thickness=1)

                label = f"{tag} {conf_val:.2f} Ang:{angle_norm:.4f}"
                cv2.putText(frame_disp, label, (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                
                # 如果这个物体是单片机刚刚请求的最小面积物块，打上红标
                if target_highlight and d == target_highlight and d['tag'] == self.last_cmd and time.time() - self.last_reply_time < 1.5:
                    cv2.putText(frame_disp, "[TARGET SENT]", (x, max(15, y - 25)), cv2.FONT_HERSHEY_DUPLEX, 0.7, (0, 0, 255), 2)
                            
            # 屏幕左上角显示单片机指令状态
            req_text = self.requested_target if self.requested_target else "Waiting for Request..."
            cv2.putText(frame_disp, f"MCU Last Trigger: {req_text}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            if not self.camera_ok:
                cv2.putText(frame_disp, "CAMERA ERROR!", (FRAME_WIDTH//2 - 150, FRAME_HEIGHT//2), cv2.FONT_HERSHEY_DUPLEX, 1.5, (0, 0, 255), 3)

            cv2.imshow("YOLO Serial Server - Ultimate Vision", frame_disp)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.running = False
                break

        self.cap.release()
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    server = YOLOSerialServer()
    server.start()