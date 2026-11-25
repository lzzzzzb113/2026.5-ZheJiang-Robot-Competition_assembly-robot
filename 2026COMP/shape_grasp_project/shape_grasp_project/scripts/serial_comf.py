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
SHAPE_MODEL_PATH = "runs/5151.pt"      
MATERIAL_MODEL_PATH = "runs/5141.pt"    
CAMERA_ID = 1

SERIAL_PORT = "AUTO" 
BAUDRATE = 115200

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
CONF_THRESHOLD = 0.25
USE_GPU = True

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

def auto_detect_serial_port(keyword=None, silent=False):
    ports = serial.tools.list_ports.comports()
    if not ports:
        if not silent:
            print("[ERROR] 未检测到任何可用串口，请检查设备连接！")
        return None

    if keyword:
        for port in ports:
            if keyword.lower() in port.description.lower():
                if not silent:
                    print(f"[INFO] 自动匹配到包含 '{keyword}' 的串口: {port.device} ({port.description})")
                return port.device

    selected_port = ports[0].device
    if not silent:
        print(f"[INFO] 自动选择第一个可用串口: {selected_port} ({ports[0].description})")
        print("[INFO] 当前系统可用串口列表:")
        for p in ports:
            print(f"       - {p.device}: {p.description}")
        
    return selected_port

def get_iou(boxA, boxB):
    xA = max(boxA[0], boxB[0])
    yA = max(boxA[1], boxB[1])
    xB = min(boxA[2], boxB[2])
    yB = min(boxA[3], boxB[3])
    interArea = max(0, xB - xA) * max(0, yB - yA)
    if interArea == 0:
        return 0.0
    boxAArea = (boxA[2] - boxA[0]) * (boxA[3] - boxA[1])
    boxBArea = (boxB[2] - boxB[0]) * (boxB[3] - boxB[1])
    return interArea / float(boxAArea + boxBArea - interArea)

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
                best_dy = dy1
            else:
                best_angle = angle2
                best_edge_pts = (p_left, p2)
                best_dy = dy2
                
            if best_dy < 0:
                best_angle += 180.0
                
            return best_angle, box, best_edge_pts
        else:
            return 0.0, box, None

    elif shape_type == 'ti':
        M = cv2.moments(cnt)
        if M['m00'] == 0:
            return 0.0, box, None
        mcx = M['m10'] / M['m00']
        mcy = M['m01'] / M['m00']
        vx = cx - mcx
        vy = cy - mcy
        best_angle = 0
        max_dot = -1
        for i in range(4):
            p1 = box[i]
            p2 = box[(i+1)%4]
            ex = p2[0] - p1[0]
            ey = p2[1] - p1[1]
            length = math.hypot(ex, ey)
            if length > 0:
                dx, dy = ex/length, ey/length
                dot = vx * dx + vy * dy
                if dot > max_dot:
                    max_dot = dot
                    best_angle = math.degrees(math.atan2(ey, ex))
        return best_angle, box, None
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
        if best_angle < 0:
            best_angle += 180
        if best_angle >= 180:
            best_angle -= 180
        return best_angle, box, None

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
    if not contours:
        return 0.0, None, None
    cnt = max(contours, key=cv2.contourArea)
    precise_angle, box_local, best_edge_local = get_robust_angle_and_box(cnt, shape_type)
    
    box_global = box_local + np.array([rx1, ry1])
    
    best_edge_global = None
    if best_edge_local is not None:
        p1 = best_edge_local[0] + np.array([rx1, ry1])
        p2 = best_edge_local[1] + np.array([rx1, ry1])
        best_edge_global = (tuple(map(int, p1)), tuple(map(int, p2)))
        
    return precise_angle, box_global, best_edge_global

class YOLOSerialServer:
    def __init__(self):
        if not os.path.exists(SHAPE_MODEL_PATH) or not os.path.exists(MATERIAL_MODEL_PATH):
            print("[ERROR] 找不到模型文件，请检查模型路径！")
            exit(1)
            
        device = 'cuda:0' if USE_GPU and torch.cuda.is_available() else 'cpu'
        print(f"[INFO] 使用设备: {device}")
        print("[INFO] 加载形状模型...")
        self.shape_model = YOLO(SHAPE_MODEL_PATH)
        print("[INFO] 加载材质模型...")
        self.material_model = YOLO(MATERIAL_MODEL_PATH)

        self.cap = cv2.VideoCapture(CAMERA_ID, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, -5)
        print("[INFO] 已关闭自动曝光，手动曝光值 -5")

        self.running = True
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.detection_results = []
        self.results_lock = threading.Lock()
        
        self.ser = None
        if not self.connect_serial(silent=False):
            print("[WARNING] 初始串口连接失败，系统将在后台自动尝试循环重连...")

    def connect_serial(self, silent=False):
        if self.ser is not None:
            try:
                self.ser.close()
            except:
                pass

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
                if not silent:
                    print(f"[ERROR] 无法打开串口 {port_to_use}: {e}")
                return False
        return False

    def capture_loop(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                with self.frame_lock:
                    self.latest_frame = frame.copy()
            else:
                time.sleep(0.01)

    def detection_loop(self):
        while self.running:
            with self.frame_lock:
                if self.latest_frame is None:
                    time.sleep(0.01)
                    continue
                frame = self.latest_frame.copy()

            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            _, frame_thresh = cv2.threshold(frame_gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

            shape_results = self.shape_model(frame, conf=CONF_THRESHOLD, verbose=False)[0]
            mat_results = self.material_model(frame, conf=CONF_THRESHOLD, verbose=False)[0]

            mat_boxes_info = []
            if hasattr(mat_results, 'boxes') and mat_results.boxes is not None:
                m_boxes = mat_results.boxes.xyxy.cpu().numpy()
                m_classes = mat_results.boxes.cls.cpu().numpy()
                for j in range(len(m_boxes)):
                    m_cls_name = self.material_model.names[int(m_classes[j])]
                    mat_boxes_info.append({'box': tuple(m_boxes[j]), 'material': m_cls_name})

            boxes_xyxy_raw = []
            classes_raw = []
            confs_raw = []
            
            if hasattr(shape_results, 'obb') and shape_results.obb is not None:
                boxes_xyxy_raw = shape_results.obb.xyxy.cpu().numpy()
                classes_raw = shape_results.obb.cls.cpu().numpy()
                confs_raw = shape_results.obb.conf.cpu().numpy()
            elif hasattr(shape_results, 'boxes') and shape_results.boxes is not None:
                boxes_xyxy_raw = shape_results.boxes.xyxy.cpu().numpy()
                classes_raw = shape_results.boxes.cls.cpu().numpy()
                confs_raw = shape_results.boxes.conf.cpu().numpy()

            shape_names = self.shape_model.named_parameters
            boxes_xyxy = []
            classes = []
            confs = []
            
            if len(boxes_xyxy_raw) > 0:
                if len(boxes_xyxy_raw) > 1:
                    areas = (boxes_xyxy_raw[:, 2] - boxes_xyxy_raw[:, 0]) * (boxes_xyxy_raw[:, 3] - boxes_xyxy_raw[:, 1])
                    min_idx = np.argmin(areas)
                    boxes_xyxy = [boxes_xyxy_raw[min_idx]]
                    classes = [classes_raw[min_idx]]
                    confs = [confs_raw[min_idx]]
                else:
                    boxes_xyxy = boxes_xyxy_raw
                    classes = classes_raw
                    confs = confs_raw

            new_detections = []
            
            for i in range(len(boxes_xyxy)):
                x1, y1, x2, y2 = boxes_xyxy[i]
                shape_class = shape_names[int(classes[i])]

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
                if shape_class == 'yuan':
                    angle_deg = 0.0
                else:
                    angle_deg, box_global, best_edge_global = process_roi_geometry(frame_thresh, x1, y1, x2, y2, shape_class)
                    angle_deg = angle_deg % 360.0

                angle_norm = round(angle_deg / 360.0, 4)
                w_orig = x2 - x1
                h_orig = y2 - y1

                if shape_class == 'san':
                    # 计算中心点
                    cx = x1 + w_orig / 2.0
                    cy = y1 + h_orig / 2.0
                    # 取最大的边，生成正方形
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
        

                new_detections.append({
                    "x": x_final, "y": y_final, "w": w_final, "h": h_final,
                    "angle_norm": angle_norm,
                    "tag": final_tag,
                    "shape_class": shape_class,
                    "best_edge_global": best_edge_global,
                    "box_global": box_global
                })

            with self.results_lock:
                self.detection_results = new_detections

            time.sleep(0.03)

    def encode_detections(self):
        parts = []
        for d in self.detection_results:
            parts.append(f"{d['x']},{d['y']},{d['w']},{d['h']},{d['tag']}:{d['angle_norm']:.4f}")
        encoded = "[" + "][".join(parts) + "]"
        return encoded

    def handle_request(self, req_code):
        if req_code == 0x46:
            data_str = "["+self.encode_detections()+"]"
        else:
            data_str = "[]"
        packet = bytes([ASCII_STX]) + data_str.encode('utf-8') + bytes([ASCII_ETX])
        return packet

    def serial_loop(self):
        retry_interval = 2.0  
        while self.running:
            if self.ser is None or not self.ser.is_open:
                if self.connect_serial(silent=True):
                    pass
                else:
                    time.sleep(retry_interval)
                    continue
            
            try:
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
                            if not b:
                                break
                            data.append(b[0])
                            if b[0] == ASCII_ETX:
                                break
                        if len(data) >= 1 and data[-1] == ASCII_ETX:
                            req_code = data[0] if len(data) > 0 else 0
                            print(f"Received request code: {req_code}")
                            response = self.handle_request(req_code)
                            self.ser.write(response)
                            print(f"Sent response: {response[:50]}...")
                else:
                    time.sleep(0.005)
                    
            except (serial.SerialException, OSError) as e:
                print(f"\n[WARNING] 串口连接断开 (通常由于线缆松动或设备重启)，异常: {e}")
                print("[INFO] 系统已进入后台自动重连模式...")
                if self.ser:
                    try:
                        self.ser.close()
                    except:
                        pass
                self.ser = None
                time.sleep(1)
                
            except Exception as e:
                print(f"[ERROR] 串口通信发生未知异常: {e}")
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
                for d in self.detection_results:
                    x, y, w, h = d['x'], d['y'], d['w'], d['h']
                    angle_norm = d['angle_norm']
                    tag = d['tag']
                    shape_class = d.get('shape_class', '')
                    best_edge_global = d.get('best_edge_global', None)
                    box_global = d.get('box_global', None)
                    
                    color = COLOR_MAP.get(tag, (255,255,255))
                    
                    
                    cv2.rectangle(frame_disp, (x, y), (x+w, y+h), color, 2)
                    
                
                    if box_global is not None:
                        cv2.polylines(frame_disp, [box_global], isClosed=True, color=(255, 255, 0), thickness=1)

                    if shape_class == 'san' and best_edge_global is not None:
                        anchor_pt = best_edge_global[0]
                        line_x = anchor_pt[0]
                        
                        y_top = max(0, y - 20)
                        y_bot = min(FRAME_HEIGHT, y + h + 20)
                        cv2.line(frame_disp, (line_x, y_top), (line_x, y_bot), (0, 255, 255), 2)
                        cv2.putText(frame_disp, "Left Anchor", (line_x - 10, y_top - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
                        
                        cv2.line(frame_disp, best_edge_global[0], best_edge_global[1], (0, 0, 255), 4)

                    label = f"{tag} {angle_norm:.4f}"
                    cv2.putText(frame_disp, label, (x, y-5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                                
            cv2.imshow("YOLO Detection - Material+Shape", frame_disp)
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