import json
import os
import cv2
import numpy as np

# ==================== 配置 ====================
# 1. 你的 Labelme JSON 文件存放的路径
JSON_DIR = r"C:\Users\lihua\Desktop\yolo_material\json" 
# 2. 转换后的 TXT 文件保存路径（通常是 dataset/labels/train）
SAVE_DIR = "./dataset/labels/train" 
# 3. 类别映射 (必须和你的 data.yaml 一致)
CLASS_MAP = {
    "metal": 0,
    "plastic": 1
}

def convert_labelme_to_yolo():
    if not os.path.exists(SAVE_DIR):
        os.makedirs(SAVE_DIR)

    json_files = [f for f in os.listdir(JSON_DIR) if f.endswith('.json')]
    
    if not json_files:
        print(f"[ERROR] 在 {JSON_DIR} 中没找到任何 JSON 文件！")
        return

    print(f"🚀 开始转换，共找到 {len(json_files)} 个标注文件...")

    for json_file in json_files:
        json_path = os.path.join(JSON_DIR, json_file)
        
        with open(json_path, 'r', encoding='utf-8') as f:
            data = json.load(f)

        # 获取图片宽高（YOLO 归一化需要）
        img_w = data['imageWidth']
        img_h = data['imageHeight']
        
        txt_name = json_file.replace('.json', '.txt')
        txt_path = os.path.join(SAVE_DIR, txt_name)

        with open(txt_path, 'w') as f_txt:
            for shape in data['shapes']:
                label = shape['label']
                
                # 检查标签是否在我们的映射表里
                if label not in CLASS_MAP:
                    print(f"[SKIP] 跳过未知标签: {label} (文件: {json_file})")
                    continue
                
                class_id = CLASS_MAP[label]
                points = np.array(shape['points'])

                # 计算外接矩形的左上角和右下角 (x_min, y_min, x_max, y_max)
                x_min = np.min(points[:, 0])
                y_min = np.min(points[:, 1])
                x_max = np.max(points[:, 0])
                y_max = np.max(points[:, 1])

                # 转换为 YOLO 格式: 中心点 x, 中心点 y, 宽度 w, 高度 h (并归一化)
                dw = 1. / img_w
                dh = 1. / img_h
                x_center = (x_min + x_max) / 2.0 * dw
                y_center = (y_min + y_max) / 2.0 * dh
                w = (x_max - x_min) * dw
                h = (y_max - y_min) * dh

                # 写入文件，保留 6 位小数
                f_txt.write(f"{class_id} {x_center:.6f} {y_center:.6f} {w:.6f} {h:.6f}\n")

    print(f"✅ 转换完成！TXT 文件已保存在: {SAVE_DIR}")

if __name__ == "__main__":
    convert_labelme_to_yolo()
