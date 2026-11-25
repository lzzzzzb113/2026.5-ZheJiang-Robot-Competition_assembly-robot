import cv2
import argparse
import numpy as np
from ultralytics import YOLO

# ============================================
# 类别名称和颜色（与 data.yaml 一致）
# ============================================
CLASS_NAMES = {
    0: "metal",      # 金属
    1: "plastic",    # 塑料
}

# 每个类别的显示颜色 (BGR)
CLASS_COLORS = {
    0: (0, 0, 255),     # metal   → 红色
    1: (0, 255, 0),     # plastic → 绿色
}

# 材质图标（用于在画面上快速区分）
CLASS_ICONS = {
    0: "M",   # Metal
    1: "P",   # Plastic
}


# ============================================
# 推理主函数
# ============================================
def main():
    parser = argparse.ArgumentParser(description="YOLO 材质分类实时推理")
    parser.add_argument(
        "--model",
        type=str,
        default="runs/train1/material_yolov8/weights/best.pt",
        help="模型权重文件路径 (默认: runs/train1/material_yolov8/weights/best.pt)",
    )
    parser.add_argument(
        "--source",
        type=str,
        default="0",
        help="输入源: 0=摄像头, 或图片/视频文件路径",
    )
    parser.add_argument(
        "--conf",
        type=float,
        default=0.5,
        help="置信度阈值 (默认: 0.5)",
    )
    parser.add_argument(
        "--imgsz",
        type=int,
        default=640,
        help="推理图片尺寸 (默认: 640)",
    )
    args = parser.parse_args()

    # ---------- 加载模型 ----------
    print(f"[INFO] 加载模型: {args.model}")
    model = YOLO(args.model)

    # ---------- 确定输入源 ----------
    if args.source == "0":
        source = 0  # 摄像头
        print("[INFO] 输入源: 摄像头 (按 q 退出)")
    else:
        source = args.source
        print(f"[INFO] 输入源: {source}")

    # ---------- 开始推理 ----------
    results = model.predict(
        source=source,
        conf=args.conf,           # 置信度阈值
        imgsz=args.imgsz,         # 推理尺寸
        stream=True,              # 流式处理（对摄像头/视频节省内存）
        verbose=False,            # 不打印每帧详情
        # 注: 已移除 device="cpu"，YOLO将自动检测并使用GPU(如可用)
    )

    # ---------- 逐帧处理 ----------
    for r in results:
        # r.orig_img 是原始帧 (numpy array)
        frame = r.orig_img.copy()

        # 在画面顶部显示类别统计
        detected_classes = {}

        # 遍历检测结果
        if r.boxes is not None:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])

                class_name = CLASS_NAMES.get(cls_id, f"unknown_{cls_id}")
                color = CLASS_COLORS.get(cls_id, (255, 255, 255))
                icon = CLASS_ICONS.get(cls_id, "?")

                # 统计
                detected_classes[class_name] = detected_classes.get(class_name, 0) + 1

                # ----- 画检测框 -----
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)

                # ----- 标签背景 -----
                label_text = f"{icon} {class_name} {conf:.2f}"
                (label_w, label_h), _ = cv2.getTextSize(
                    label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2
                )
                cv2.rectangle(
                    frame,
                    (x1, y1 - label_h - 10),
                    (x1 + label_w + 10, y1),
                    color,
                    -1,  # 填充
                )

                # ----- 标签文字 -----
                cv2.putText(
                    frame,
                    label_text,
                    (x1 + 5, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),  # 白色文字
                    2,
                )

        # ----- 显示帧率 + 统计信息（左上角） -----
        info_y = 30
        
        # 修复 FPS 计算: 将毫秒耗时转换为每秒帧数
        speed_dict = getattr(r, 'speed', {})
        total_time_ms = speed_dict.get('preprocess', 0) + speed_dict.get('inference', 1) + speed_dict.get('postprocess', 0)
        fps = 1000.0 / total_time_ms if total_time_ms > 0 else 0
        
        cv2.putText(
            frame,
            f"FPS: {fps:.0f}",
            (10, info_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 0),
            2,
        )

        y_offset = info_y + 25
        for cls_name, count in detected_classes.items():
            cv2.putText(
                frame,
                f"{cls_name}: {count}",
                (10, y_offset),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (200, 200, 255),
                1,
            )
            y_offset += 20

        # ----- 底部提示 -----
        cv2.putText(
            frame,
            "Press 'q' to quit",
            (10, frame.shape[0] - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (100, 100, 100),
            1,
        )

        # ----- 显示画面 -----
        cv2.imshow("YOLO Material Classification - 材质分类", frame)

        # 按 q 退出
        if cv2.waitKey(1) & 0xFF == ord("q"):
            print("[INFO] 用户退出")
            break

    cv2.destroyAllWindows()
    print("[INFO] 推理结束")


if __name__ == "__main__":
    main()
