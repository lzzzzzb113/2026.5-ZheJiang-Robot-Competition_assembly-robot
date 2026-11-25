import json
import cv2
import numpy as np
import os
import shutil
import random
from pathlib import Path
from sklearn.model_selection import train_test_split

# ==================== 配置区域 ====================
IMAGES_DIR = "./images"           # 原始图片文件夹
ANNOTATIONS_DIR = "./annotations" # LabelMe JSON 文件夹
OUTPUT_DIR = "./datasets"         # 输出数据集根目录
TRAIN_RATIO = 0.8                 # 训练集比例
RANDOM_SEED = 42                  # 随机种子

# 类别映射（根据你的实际标签修改）
CLASS_MAPPING = {
    'si': 0,    # 四边形
    'san': 1,   # 三角形
    'ti': 2,    # 梯形
    'yuan': 3   # 圆形
}
# =================================================

def convert_polygon_to_obb(points, img_w, img_h):
    """多边形 -> 最小外接旋转矩形的四个角点（归一化）"""
    if len(points) < 3:
        return None
    pts = np.array(points, dtype=np.float32)
    rect = cv2.minAreaRect(pts)
    box = cv2.boxPoints(rect)
    norm_box = []
    for (x, y) in box:
        norm_box.append(x / img_w)
        norm_box.append(y / img_h)
    norm_box = [max(0.0, min(1.0, v)) for v in norm_box]
    return norm_box

def convert_circle_to_obb(points, img_w, img_h):
    """圆形（圆心+圆上一点） -> 外接正方形的四个角点（归一化）"""
    if len(points) < 2:
        return None
    center = np.array(points[0], dtype=np.float32)
    radius_point = np.array(points[1], dtype=np.float32)
    radius = np.linalg.norm(center - radius_point)
    # 外接正方形，边长 = 直径
    x_min = center[0] - radius
    x_max = center[0] + radius
    y_min = center[1] - radius
    y_max = center[1] + radius
    corners = [(x_min, y_min), (x_max, y_min), (x_max, y_max), (x_min, y_max)]
    norm_box = []
    for (x, y) in corners:
        norm_box.append(x / img_w)
        norm_box.append(y / img_h)
    norm_box = [max(0.0, min(1.0, v)) for v in norm_box]
    return norm_box

def process_json(json_path, img_dir, output_label_dir, class_mapping):
    print(f"处理: {os.path.basename(json_path)}")
    with open(json_path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    img_filename = data.get('imagePath')
    if not img_filename:
        print(f"  错误: 缺少 imagePath")
        return False
    img_filename = os.path.basename(img_filename)
    img_path = os.path.join(img_dir, img_filename)
    if not os.path.exists(img_path):
        print(f"  错误: 图片不存在 {img_path}")
        return False

    img = cv2.imread(img_path)
    if img is None:
        print(f"  错误: 无法读取图片 {img_path}")
        return False
    img_h, img_w = img.shape[:2]

    obb_lines = []
    shapes = data.get('shapes', [])
    if not shapes:
        print(f"  警告: 没有 shapes")
        return False

    for shape in shapes:
        shape_type = shape.get('shape_type')
        label = shape.get('label')
        points = shape.get('points', [])
        print(f"  shape: type={shape_type}, label={label}, points={len(points)}")

        if label not in class_mapping:
            print(f"    跳过: 未知标签 '{label}'")
            continue

        if shape_type == 'polygon':
            if len(points) < 3:
                print(f"    跳过: 多边形点数不足3")
                continue
            norm_corners = convert_polygon_to_obb(points, img_w, img_h)
        elif shape_type == 'circle':
            if len(points) < 2:
                print(f"    跳过: 圆形点数不足2")
                continue
            norm_corners = convert_circle_to_obb(points, img_w, img_h)
        else:
            print(f"    跳过: 不支持的 shape_type '{shape_type}'，仅支持 polygon 和 circle")
            continue

        if norm_corners is None:
            print(f"    跳过: OBB转换失败")
            continue

        class_id = class_mapping[label]
        line = f"{class_id} " + " ".join(f"{v:.6f}" for v in norm_corners)
        obb_lines.append(line)
        print(f"    成功: 生成 OBB 标注")

    if not obb_lines:
        print(f"  警告: 没有有效的 OBB 标注")
        return False

    txt_filename = Path(json_path).stem + '.txt'
    txt_path = os.path.join(output_label_dir, txt_filename)
    with open(txt_path, 'w') as f:
        f.write("\n".join(obb_lines))
    print(f"  成功写入标签: {txt_path}")
    return True

def main():
    random.seed(RANDOM_SEED)

    json_files = [f for f in os.listdir(ANNOTATIONS_DIR) if f.endswith('.json')]
    if not json_files:
        print(f"错误: {ANNOTATIONS_DIR} 中没有 JSON 文件")
        return

    train_json, val_json = train_test_split(json_files, train_size=TRAIN_RATIO, random_state=RANDOM_SEED)
    print(f"总样本: {len(json_files)}, 训练: {len(train_json)}, 验证: {len(val_json)}")

    # 创建目录
    for split in ['train', 'val']:
        os.makedirs(os.path.join(OUTPUT_DIR, 'images', split), exist_ok=True)
        os.makedirs(os.path.join(OUTPUT_DIR, 'labels', split), exist_ok=True)

    # 处理训练集
    for json_file in train_json:
        json_path = os.path.join(ANNOTATIONS_DIR, json_file)
        with open(json_path, 'r') as f:
            data = json.load(f)
        img_filename = os.path.basename(data.get('imagePath', ''))
        img_src = os.path.join(IMAGES_DIR, img_filename)
        img_dst = os.path.join(OUTPUT_DIR, 'images', 'train', img_filename)
        if os.path.exists(img_src):
            shutil.copy2(img_src, img_dst)
        else:
            print(f"训练集图片缺失: {img_src}")
        process_json(json_path, IMAGES_DIR, os.path.join(OUTPUT_DIR, 'labels', 'train'), CLASS_MAPPING)

    # 处理验证集
    for json_file in val_json:
        json_path = os.path.join(ANNOTATIONS_DIR, json_file)
        with open(json_path, 'r') as f:
            data = json.load(f)
        img_filename = os.path.basename(data.get('imagePath', ''))
        img_src = os.path.join(IMAGES_DIR, img_filename)
        img_dst = os.path.join(OUTPUT_DIR, 'images', 'val', img_filename)
        if os.path.exists(img_src):
            shutil.copy2(img_src, img_dst)
        else:
            print(f"验证集图片缺失: {img_src}")
        process_json(json_path, IMAGES_DIR, os.path.join(OUTPUT_DIR, 'labels', 'val'), CLASS_MAPPING)

    # 生成 dataset.yaml
    yaml_content = f"""path: {os.path.abspath(OUTPUT_DIR)}
train: images/train
val: images/val
nc: {len(CLASS_MAPPING)}
names: {list(CLASS_MAPPING.keys())}
"""
    yaml_path = os.path.join(OUTPUT_DIR, 'dataset.yaml')
    with open(yaml_path, 'w') as f:
        f.write(yaml_content)
    print(f"配置文件生成: {yaml_path}")

if __name__ == '__main__':
    main()