import cv2
import serial
import threading
import time
import json
from ultralytics import YOLO
import struct

# ==================== 配置 ====================
MODEL_PATH = "runs/detect/train/weights/best.pt"   # 模型路径
CAMERA_ID = 0
SERIAL_PORT = "COM3"        # 根据实际修改
BAUDRATE = 115200
FRAME_WIDTH = 640           # 降低分辨率提高速度
FRAME_HEIGHT = 480
# 协议字符
ASCII_STX = 0x02
ASCII_ETX = 0x03
ASCII_ACK = 0x06
ASCII_SYN = 0x16
# ============================================

class YOLOSerialServer:
    def __init__(self):
        self.model = YOLO(MODEL_PATH)
        self.cap = cv2.VideoCapture(CAMERA_ID)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.01)
        self.running = True
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.detection_results = []   # 存储最新检测结果
        self.results_lock = threading.Lock()

    def capture_loop(self):
        """持续读取摄像头并更新最新帧（非阻塞）"""
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                with self.frame_lock:
                    self.latest_frame = frame.copy()
            else:
                time.sleep(0.01)

    def detection_loop(self):
        """持续对最新帧进行检测（独立线程）"""
        while self.running:
            with self.frame_lock:
                if self.latest_frame is None:
                    continue
                frame = self.latest_frame.copy()
            # 推理
            results = self.model(frame)[0]
            boxes = results.boxes
            detections = []
            if boxes is not None:
                xyxy = boxes.xyxy.cpu().numpy()   # [x1,y1,x2,y2]
                confs = boxes.conf.cpu().numpy()
                clss = boxes.cls.cpu().numpy()
                for i in range(len(xyxy)):
                    x1, y1, x2, y2 = xyxy[i].astype(int)
                    w = x2 - x1
                    h = y2 - y1
                    tag = self.model.names[int(clss[i])]
                    confidence = float(confs[i])
                    detections.append({
                        "x": x1, "y": y1, "w": w, "h": h,
                        "tag": tag, "confidence": confidence
                    })
            with self.results_lock:
                self.detection_results = detections
            # 控制检测频率（可根据需要调整）
            time.sleep(0.03)

    def encode_detections(self):
        """将检测结果编码为协议字符串: [[x,y,w,h,tag:confidence][...]]"""
        parts = []
        for d in self.detection_results:
            # 格式: x,y,w,h,tag:confidence
            # 注意：tag 不应包含空格或特殊字符
            parts.append(f"{d['x']},{d['y']},{d['w']},{d['h']},{d['tag']}:{d['confidence']:.3f}")
        encoded = "[" + "][".join(parts) + "]"
        return encoded

    def handle_request(self, req_code):
        """处理下位机请求，返回响应数据包"""
        # 这里可以根据 req_code 执行不同操作（如请求帧、重启等）
        if req_code == 0x01:   # 请求检测结果
            data_str = self.encode_detections()
        else:
            data_str = "[]"    # 未知请求返回空
        # 打包: STX + 数据 + ETX
        packet = bytes([ASCII_STX]) + data_str.encode('utf-8') + bytes([ASCII_ETX])
        return packet

    def serial_loop(self):
        """串口监听线程：解析下位机请求并回复"""
        # 首先处理握手：下位机发送 SYN(0x16)，上位机回复 ACK(0x06)
        # 注意：STM32 代码中 UpdateCamera() 会发送 SYN，等待 ACK
        handshake_done = False
        last_heartbeat = time.time()
        while self.running:
            if self.ser.in_waiting:
                byte = self.ser.read(1)
                if not handshake_done:
                    if byte[0] == ASCII_SYN:
                        self.ser.write(bytes([ASCII_ACK]))
                        print("Handshake OK (SYN->ACK)")
                        handshake_done = True
                    continue
                # 已经握手成功，开始解析请求包
                if byte[0] == ASCII_STX:
                    # 读取直到 ETX
                    data = bytearray()
                    while True:
                        b = self.ser.read(1)
                        if not b:
                            break
                        data.append(b[0])
                        if b[0] == ASCII_ETX:
                            break
                    if len(data) >= 2 and data[-1] == ASCII_ETX:
                        # 提取 ReqCode（数据部分的第一个字节）
                        req_code = data[0] if len(data) > 0 else 0
                        print(f"Received request, code={req_code}")
                        response = self.handle_request(req_code)
                        self.ser.write(response)
                        print(f"Sent response: {response}")
            else:
                time.sleep(0.005)

    def start(self):
        """启动所有线程"""
        # 线程1：捕获视频
        t_cap = threading.Thread(target=self.capture_loop, daemon=True)
        t_cap.start()
        # 线程2：目标检测
        t_det = threading.Thread(target=self.detection_loop, daemon=True)
        t_det.start()
        # 线程3：串口通信
        t_serial = threading.Thread(target=self.serial_loop, daemon=True)
        t_serial.start()
        # 主线程保持运行，可同时显示画面（可选）
        print("Server started. Press 'q' to quit.")
        while self.running:
            # 显示实时检测画面（可选）
            with self.frame_lock:
                if self.latest_frame is not None:
                    frame_disp = self.latest_frame.copy()
            with self.results_lock:
                for d in self.detection_results:
                    x, y, w, h = d['x'], d['y'], d['w'], d['h']
                    cv2.rectangle(frame_disp, (x, y), (x+w, y+h), (0,255,0), 2)
                    cv2.putText(frame_disp, f"{d['tag']} {d['confidence']:.2f}",
                                (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
            if frame_disp is not None:
                cv2.imshow("YOLO Detection", frame_disp)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.running = False
                break
        self.cap.release()
        self.ser.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    server = YOLOSerialServer()
    server.start()