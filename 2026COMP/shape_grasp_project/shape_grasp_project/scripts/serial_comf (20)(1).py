import cv2
import serial
import serial.tools.list_ports
import threading
import time
import math
import numpy as np
import torch
import os
from ultralytics import YOLO

# ==================== 配置 ====================
# 【模型路径】
SHAPE_MODEL_PATH = "shape_grasp_project/runs/5184.pt"      
MATERIAL_MODEL_PATH = "shape_grasp_project/runs/5183m.pt"   
CAMERA_ID = 1

# 【串口配置】
SERIAL_PORT = "AUTO" 
BAUDRATE = 9600

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
    'si': (0, 255, 0),
    'san': (255, 0, 0),
    'ti': (0, 0, 255),
    'yuan': (0, 255, 255),
    'pt': (0, 255, 255),   
    'mt': (0, 165, 255), 
    'ps': (255, 0, 255), 
    'ms': (255, 0, 0)   
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
                
            if best_dy < 0:
                best_angle += 180.0
                
            return best_angle, box, best_edge_pts, opp_pt
        else:
            return 0.0, box, None, None

    elif shape_type == 'ti':
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

def process_roi_geometry(frame_thresh, x1, y1, x2, y2, shape_type):
    h, w = frame_thresh.shape
    margin = 15
    rx1, ry1 = max(0, int(x1)-margin), max(0, int(y1)-margin)
    rx2, ry2 = min(w, int(x2)+margin), min(h, int(y2)+margin)

    roi_thresh = frame_thresh[ry1:ry2, rx1:rx2]
    kernel = np.ones((5,5), np.uint8)
    roi_thresh = cv2.morphologyEx(roi_thresh, cv2.MORPH_OPEN, kernel)
    roi_thresh = cv2.morphologyEx(roi_thresh, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(roi_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours: return 0.0, None, None, None
    
    cnt = max(contours, key=cv2.contourArea)
    precise_angle, box_local, best_edge_local, opp_pt_local = get_robust_angle_and_box(cnt, shape_type)
    
    box_global = box_local + np.array([rx1, ry1])
    
    best_edge_global = None
    opp_pt_global = None
    
    if best_edge_local is not None:
        p1 = best_edge_local[0] + np.array([rx1, ry1])
        p2 = best_edge_local[1] + np.array([rx1, ry1])
        best_edge_global = (tuple(map(int, p1)), tuple(map(int, p2)))
        
    if opp_pt_local is not None:
        opp_pt_global = tuple(map(int, opp_pt_local + np.array([rx1, ry1])))
        
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
        self.cap.set(cv2.CAP_PROP_EXPOSURE, -5)

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
                
                if shape_class == 'yuan':
                    angle_deg = 0.0
                else:
                    angle_deg, box_global, best_edge_global, opp_pt_global = process_roi_geometry(frame_thresh, x1, y1, x2, y2, shape_class)
                    angle_deg = angle_deg % 360.0

                angle_norm = round(angle_deg / 360.0, 4)
                
                # 计算出真实面积（用于排序，找最小的）
                area = (x2 - x1) * (y2 - y1)
                
                all_detections.append({
                    "raw_box": (x1, y1, x2, y2),
                    "shape_class": shape_class,
                    "final_tag": final_tag,
                    "conf": conf_val,
                    "area": area,
                    "angle_norm": angle_norm,
                    "best_edge_global": best_edge_global,
                    "opp_pt_global": opp_pt_global,
                    "box_global": box_global
                })

            new_results = []
            for det in all_detections:
                x1, y1, x2, y2 = det['raw_box']
                shape_class = det['shape_class']
                w_orig = x2 - x1
                h_orig = y2 - y1
                p_proj_global = None
                new_center = None

                if shape_class == 'san' and det['best_edge_global'] is not None and det['opp_pt_global'] is not None:
                    P0 = np.array(det['best_edge_global'][0], dtype=float)
                    P1 = np.array(det['best_edge_global'][1], dtype=float)
                    P2 = np.array(det['opp_pt_global'], dtype=float)

                    v = P1 - P0
                    w = P2 - P0
                    v_sq = np.dot(v, v)
                    if v_sq > 0:
                        t = np.dot(w, v) / v_sq
                        P_proj = P0 + t * v
                    else:
                        P_proj = P0

                    C_new = (P2 + P_proj) / 2.0
                    cx, cy = C_new[0], C_new[1]
                    p_proj_global = (int(P_proj[0]), int(P_proj[1]))
                    new_center = (int(cx), int(cy))

                elif shape_class == 'ti' and det['opp_pt_global'] is not None:
                    cx, cy = det['opp_pt_global']
                    new_center = (int(cx), int(cy))
                else:
                    cx = x1 + w_orig / 2.0
                    cy = y1 + h_orig / 2.0

                if shape_class in ['san', 'ti']:
                    side = max(w_orig, h_orig)
                    x_final = int(cx - side / 2.0)
                    y_final = int(cy - side / 2.0)
                    w_final = int(side)
                    h_final = int(side)
                else:
                    x_final = int(x1)
                    y_final = int(y1)
                    w_final = int(w_orig)
                    h_final = int(h_orig)

                new_results.append({
                    "x": x_final, "y": y_final, "w": w_final, "h": h_final,
                    "angle_norm": det['angle_norm'],
                    "tag": det['final_tag'],
                    "conf": det['conf'],
                    "area": det['area'], # 保留面积字段供后续排序使用
                    "shape_class": shape_class,
                    "best_edge_global": det['best_edge_global'],
                    "opp_pt_global": det['opp_pt_global'],
                    "p_proj_global": p_proj_global,
                    "new_center": new_center,
                    "box_global": det['box_global']
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

    def handle_request(self, req_code):
        # 1. 摄像头断线检查 (返回空列表而不直接发 0x46，避免扰乱通信协议)
        if not self.camera_ok or self.latest_frame is None:
            print("[ERROR] 摄像头异常，返回空数据")
            return bytes([ASCII_STX]) + b"[]" + bytes([ASCII_ETX])

        # 2. 0x46 获取全部数据
        if req_code == 0x46:
            self.requested_target = "ALL (0x46)"
            self.last_cmd = "ALL"
            self.last_reply_time = time.time()
            
            encoded_str = self.encode_detections()
            data_str = "[" + encoded_str + "]" if encoded_str else "[]"
            return bytes([ASCII_STX]) + "[".encode('utf-8') + data_str.encode('utf-8')+ "]".encode('utf-8') + bytes([ASCII_ETX])

        # 3. 指定图形拉取 (单片机发送 1~6 字符)
        target_tag = CMD_MAP.get(req_code, None)
        self.requested_target = target_tag 

        if target_tag is None:
            return bytes([ASCII_STX]) + b"[]" + bytes([ASCII_ETX])

        self.last_cmd = target_tag

        with self.results_lock:
            matches = [d for d in self.detection_results if d['tag'] == target_tag]

        if not matches:
            return bytes([ASCII_STX]) + b"[]" + bytes([ASCII_ETX])

        # 【核心逻辑】：同类物体，找面积最小的！
        # area 越小，排序越靠前，所以 matches[0] 即为最小的
        matches.sort(key=lambda x: x['area'])
        best = matches[0]

        data_str = f"[{best['x']},{best['y']},{best['w']},{best['h']},{best['tag']}:{best['conf']:.4f}:{best['angle_norm']:.4f}]"
        self.last_reply_time = time.time()
        
        return bytes([ASCII_STX]) + "[".encode('utf-8') + data_str.encode('utf-8')+ "]".encode('utf-8') + bytes([ASCII_ETX])

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
                # 【严格保留模板中的包头包尾解析规则和握手协议】
                if self.ser.in_waiting:
                    byte = self.ser.read(1)
                    
                    if byte[0] == ASCII_SYN:
                        self.ser.write(bytes([ASCII_ACK]))
                        print("Handshake OK (SYN->ACK)")
                        continue
                        
                    if byte[0] == ASCII_STX:
                        data = bytearray()
                        while True:
                            b = self.ser.read(1)
                            if not b: break
                            data.append(b[0])
                            if b[0] == ASCII_ETX: break
                            
                        if len(data) >= 1 and data[-1] == ASCII_ETX:
                            # 提取单片机发来的有效请求码
                            req_code = data[0] if len(data) > 0 else 0
                            
                            req_name = "ALL SHAPES (0x46)" if req_code == 0x46 else CMD_MAP.get(req_code, f"Unknown(0x{req_code:02X})")
                            print(f"[POLL] 收到请求指令: '{chr(req_code) if 32 <= req_code <= 126 else hex(req_code)}' -> 意图: {req_name}")
                            
                            response = self.handle_request(req_code)
                            self.ser.write(response)
                            self.ser.flush()
                            print(f"[POLL] 同步回复: {response[:50]}...")
                            last_push_time = time.time() 
                            continue

                # 【10Hz 兜底或主动推送机制（与母版一致）】
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
                    y_bot = min(FRAME_HEIGHT, y + h + 20)
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
                
                # 【神级 UI 指示】：如果这个物体是单片机刚刚请求的最小面积物块，打上红标！
                if target_highlight and d == target_highlight and d['tag'] == self.last_cmd and time.time() - self.last_reply_time < 1.5:
                    cv2.putText(frame_disp, "[🎯 MIN TARGET SENT]", (x, max(15, y - 25)), cv2.FONT_HERSHEY_DUPLEX, 0.7, (0, 0, 255), 2)
                            
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