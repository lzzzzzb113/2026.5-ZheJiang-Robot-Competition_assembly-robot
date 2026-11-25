"""
【组合模块 V4】形状检测 + 精确角度计算 + 高级串口通讯
新增功能：自动串口检测扫描、断连自动重连、单片机同步答复(等待ACK)
运行：python shape_and_angle_combined.py
依赖：runs/5182.pt, pyserial, opencv-python, ultralytics
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

# ==================== 串口通信配置区 ====================
BAUD_RATE = 115200                         # 波特率

# 通讯协议字节
ASCII_STX = 0x02
ASCII_ETX = 0x03
ASCII_ACK = 0x06
ASCII_SYN = 0x16

class SerialManager:
    """高级串口管理器：负责自动检测、断线重连和同步收发"""
    def __init__(self, baudrate):
        self.baudrate = baudrate
        self.ser = None
        self.last_reconnect_time = 0
        self.reconnect_interval = 2.0  # 发生断连时，每隔 2 秒尝试重连一次

    def check_and_connect(self):
        """检查连接状态，若断开则自动扫描并连接"""
        if self.ser is not None and self.ser.is_open:
            return True

        current_time = time.time()
        # 避免疯狂重试导致资源占用
        if current_time - self.last_reconnect_time < self.reconnect_interval:
            return False

        self.last_reconnect_time = current_time
        ports = list(serial.tools.list_ports.comports())
        
        if not ports:
            print("[WARNING] 未检测到任何串口设备...")
            return False

        for p in ports:
            try:
                # timeout设置较短，防止读取时卡死主线程
                self.ser = serial.Serial(p.device, self.baudrate, timeout=0.02)
                print(f"[INFO] >>> 成功连接到串口: {p.device} ({p.description}) <<<")
                return True
            except serial.SerialException:
                continue
                
        print("[WARNING] 串口扫描完毕，未能成功打开任何端口，稍后重试...")
        return False

    def send_sync(self, data_str, timeout=0.1):
        """
        同步发送数据并等待单片机 ACK 答复
        :param timeout: 等待答复的超时时间（秒），默认100ms保证帧率
        """
        if not self.check_and_connect():
            return False

        try:
            # 1. 封包与发送
            packet = bytearray()
            packet.append(ASCII_STX)
            packet.extend(data_str.encode('utf-8'))
            packet.append(ASCII_ETX)
            
            # 清空接收缓冲区，避免读取到上一次的残留脏数据
            self.ser.reset_input_buffer()
            self.ser.write(packet)

            # 2. 等待 ACK 答复
            start_t = time.time()
            while time.time() - start_t < timeout:
                if self.ser.in_waiting > 0:
                    reply = self.ser.read(1)
                    if reply == bytes([ASCII_ACK]):
                        return True
                    # 这里可以根据你的单片机协议拓展，比如收到 NAK (0x15) 代表数据错乱
            
            print(f"[WARNING] 等待单片机 ACK 超时 (数据: {data_str})")
            return False

        except serial.SerialException as e:
            print(f"[ERROR] 串口通信异常，设备已断开: {e}")
            if self.ser:
                self.ser.close()
            self.ser = None  # 设为 None 触发下一次的自动重连
            return False

    def close(self):
        if self.ser is not None and self.ser.is_open:
            self.ser.close()


# ==================== 工具函数：检测框去重 ====================
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
    sorted_indices = np.argsort(areas)
    keep_boxes, keep_classes, keep_confs = [], [], []
    for idx in sorted_indices:
        boxA = boxes[idx]
        if not any(is_same_object(boxA, kb) for kb in keep_boxes):
            keep_boxes.append(boxA)
            keep_classes.append(classes[idx])
            keep_confs.append(confs[idx])
    return keep_boxes, keep_classes, keep_confs

# ==================== 极精角度计算（纯几何） ====================
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
            if len(pts) > 8: 
                pts = pts[:8]
            for combo in itertools.combinations(pts, 3):
                tri_pts = np.array(combo, dtype=np.int32)
                tri_mask = np.zeros(mask_shape, dtype=np.uint8)
                cv2.drawContours(tri_mask, [tri_pts], -1, 255, -1)

                intersection = cv2.bitwise_and(orig_mask, tri_mask)
                area_inter = cv2.countNonZero(intersection)
                area_union = (cv2.countNonZero(orig_mask) + cv2.countNonZero(tri_mask) - area_inter)
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

    elif shape_type == 'ti': 
        hull = cv2.convexHull(cnt)
        epsilon = 0.02 * cv2.arcLength(hull, True)
        approx = cv2.approxPolyDP(hull, epsilon, True)

        if len(approx) == 4:
            pts = approx.reshape(4, 2)
            edges = []
            for i in range(4):
                p1 = pts[i]
                p2 = pts[(i + 1) % 4]
                vx, vy = p2[0] - p1[0], p2[1] - p1[1]
                length = math.hypot(vx, vy)
                edges.append({'p1': p1, 'p2': p2, 'vx': vx, 'vy': vy, 'len': length})

            def calc_parallel_score(eA, eB):
                if eA['len'] == 0 or eB['len'] == 0: return 0
                dot = (eA['vx']*eB['vx'] + eA['vy']*eB['vy']) / (eA['len'] * eB['len'])
                return abs(dot)

            score02 = calc_parallel_score(edges[0], edges[2])
            score13 = calc_parallel_score(edges[1], edges[3])

            if score02 > score13:
                base1, base2 = edges[0], edges[2]
            else:
                base1, base2 = edges[1], edges[3]

            if base1['len'] > base2['len']:
                long_base, short_base = base1, base2
            else:
                long_base, short_base = base2, base1

            mid_long = (int((long_base['p1'][0] + long_base['p2'][0]) / 2),
                        int((long_base['p1'][1] + long_base['p2'][1]) / 2))
            mid_short = (int((short_base['p1'][0] + short_base['p2'][0]) / 2),
                         int((short_base['p1'][1] + short_base['p2'][1]) / 2))

            dx = mid_short[0] - mid_long[0]
            dy = mid_short[1] - mid_long[1]
            best_angle = math.degrees(math.atan2(dy, dx))
            best_angle = (best_angle + 360) % 360

            return best_angle, box, (mid_long, mid_short), None

        M = cv2.moments(cnt)
        if M['m00'] == 0: return 0.0, box, None, None
        mcx = M['m10'] / M['m00'] 
        mcy = M['m01'] / M['m00']

        vx = cx - mcx
        vy = cy - mcy
        if math.hypot(vx, vy) < 1.0: return 0.0, box, None, None

        best_angle = math.degrees(math.atan2(vy, vx))
        best_angle = (best_angle + 360) % 360
        arrow_len = min(w, h) * 0.8
        pt1 = (int(mcx), int(mcy))
        pt2 = (int(mcx + math.cos(math.radians(best_angle)) * arrow_len), 
               int(mcy + math.sin(math.radians(best_angle)) * arrow_len))
        
        return best_angle, box, (pt1, pt2), None

    else: 
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

# ==================== ROI抗边缘干扰几何处理器 ====================
def process_roi_geometry(frame_thresh, x1, y1, x2, y2, shape_type):
    h_frame, w_frame = frame_thresh.shape
    margin = 2
    rx1, ry1 = max(0, int(x1) - margin), max(0, int(y1) - margin)
    rx2, ry2 = min(w_frame, int(x2) + margin), min(h_frame, int(y2) + margin)

    if rx2 - rx1 < 5 or ry2 - ry1 < 5: return 0.0, None, None, None
    roi_thresh = frame_thresh[ry1:ry2, rx1:rx2].copy()
    h_roi, w_roi = roi_thresh.shape

    kernel_size = max(3, int(min(w_roi, h_roi) * 0.08))
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    eroded = cv2.erode(roi_thresh, kernel, iterations=1)

    contours_eroded, _ = cv2.findContours(eroded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best_cnt = None
    if contours_eroded:
        cx_roi, cy_roi = w_roi / 2.0, h_roi / 2.0
        min_dist = float('inf')
        for c in contours_eroded:
            M = cv2.moments(c)
            if M['m00'] == 0: continue
            ccx, ccy = M['m10'] / M['m00'], M['m01'] / M['m00']
            dist = (ccx - cx_roi) ** 2 + (ccy - cy_roi) ** 2
            if dist < min_dist:
                min_dist, best_cnt = dist, c

    if best_cnt is not None:
        clean_mask = np.zeros_like(roi_thresh)
        cv2.drawContours(clean_mask, [best_cnt], -1, 255, -1)
        dilate_kernel = np.ones((kernel_size + 2, kernel_size + 2), np.uint8)
        clean_mask = cv2.dilate(clean_mask, dilate_kernel, iterations=1)
        roi_thresh = cv2.bitwise_and(roi_thresh, clean_mask)

    cv2.rectangle(roi_thresh, (0, 0), (w_roi - 1, h_roi - 1), 0, 1)
    contours, _ = cv2.findContours(roi_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours: return 0.0, None, None, None
    cnt = max(contours, key=cv2.contourArea)

    if shape_type in ['san', 'ti']:
        epsilon = 0.035 * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)
        if len(approx) >= 3: cnt = approx

    precise_angle, box_local, best_edge_local, opp_pt_local = get_robust_angle_and_box(cnt, shape_type)
    box_global = box_local + np.array([rx1, ry1])

    best_edge_global = opp_pt_global = None
    if best_edge_local is not None:
        best_edge_global = (tuple(best_edge_local[0] + np.array([rx1, ry1])), 
                            tuple(best_edge_local[1] + np.array([rx1, ry1])))
    if opp_pt_local is not None:
        opp_pt_global = tuple(opp_pt_local + np.array([rx1, ry1]))

    return precise_angle, box_global, best_edge_global, opp_pt_global


# ==================== 主程序 ====================
def main():
    if not os.path.exists(SHAPE_MODEL_PATH):
        print(f"[ERROR] 模型文件不存在: {SHAPE_MODEL_PATH}")
        return

    # 初始化串口管理器（自动接管检测、连接、重连）
    serial_mgr = SerialManager(BAUD_RATE)
    # 启动时主动尝试扫描一次
    serial_mgr.check_and_connect()

    device = 'cuda:0' if USE_GPU and torch.cuda.is_available() else 'cpu'
    print(f"[INFO] 使用设备: {device}")
    shape_model = YOLO(SHAPE_MODEL_PATH)
    shape_model.to(device)

    cap = cv2.VideoCapture(CAMERA_ID, cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    print("[INFO] 视觉系统运行中...按 'q' 退出")
    
    while True:
        ret, frame = cap.read()
        if not ret: continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, frame_thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        results = shape_model(frame, conf=CONF_THRESHOLD, verbose=False)[0]
        
        # 兼容最新与旧版 YOLO 返回结构
        if hasattr(results, 'obb') and results.obb is not None:
            boxes_xyxy_raw = results.obb.xyxy.cpu().numpy()
            classes_raw = results.obb.cls.cpu().numpy()
            confs_raw = results.obb.conf.cpu().numpy()
        elif hasattr(results, 'boxes') and results.boxes is not None:
            boxes_xyxy_raw = results.boxes.xyxy.cpu().numpy()
            classes_raw = results.boxes.cls.cpu().numpy()
            confs_raw = results.boxes.conf.cpu().numpy()
        else:
            boxes_xyxy_raw, classes_raw, confs_raw = [], [], []

        shape_names = shape_model.names

        boxes_xyxy, classes, confs = filter_smallest_boxes(boxes_xyxy_raw, classes_raw, confs_raw)

        for i in range(len(boxes_xyxy)):
            x1, y1, x2, y2 = map(int, boxes_xyxy[i])
            cls_id = int(classes[i])
            conf = float(confs[i])
            shape_class = shape_names[cls_id]
            color = SHAPE_COLOR.get(shape_class, (255, 255, 255))

            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            angle_deg = 0.0

            if shape_class in ['san', 'ti']:
                angle_deg, box_global, best_edge_global, opp_pt_global = \
                    process_roi_geometry(frame_thresh, x1, y1, x2, y2, shape_class)

                angle_deg = angle_deg % 360.0 

                if box_global is not None:
                    cv2.polylines(frame, [box_global], True, (0, 255, 255), 2)
                if best_edge_global is not None:
                    cv2.arrowedLine(frame, best_edge_global[0], best_edge_global[1], (0, 0, 255), 3, tipLength=0.2)
                    if shape_class == 'san':
                        anchor_x = best_edge_global[0][0]
                        cv2.line(frame, (anchor_x, y1 - 20), (anchor_x, y2 + 20), (0, 255, 255), 2)
                if opp_pt_global is not None:
                    cv2.circle(frame, opp_pt_global, 5, (0, 255, 0), -1)

                label = f"{shape_class} {conf:.2f} Ang:{angle_deg:.1f}"
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            elif shape_class == 'si':
                angle_deg, box_global, _, _ = process_roi_geometry(frame_thresh, x1, y1, x2, y2, 'si')
                angle_deg = angle_deg % 90.0
                label = f"si {conf:.2f} Ang:{angle_deg:.1f}"
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            else:
                label = f"yuan {conf:.2f} Ang:0.0"
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # ==================== 打包与同步发送 ====================
            best = {
                'x': int((x1 + x2) / 2),
                'y': int((y1 + y2) / 2),
                'w': int(x2 - x1),
                'h': int(y2 - y1),
                'tag': shape_class,
                'conf': conf,
                'angle_norm': angle_deg
            }

            data_str = f"[{best['x']},{best['y']},{best['w']},{best['h']},{best['tag']}:{best['conf']:.4f}:{best['angle_norm']:.4f}]"

            # 发送并等待 ACK，如果在 100ms 内没有答复或者断开了，会自动在底层处理
            success = serial_mgr.send_sync(data_str, timeout=0.1)
            
            # 在画面左上角绘制通讯状态提示
            status_text = "Serial: OK" if success else "Serial: ERR / Wait"
            status_color = (0, 255, 0) if success else (0, 0, 255)
            cv2.putText(frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
            # ==========================================================

        cv2.imshow("Shape Detection + Precision Angle", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    serial_mgr.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()