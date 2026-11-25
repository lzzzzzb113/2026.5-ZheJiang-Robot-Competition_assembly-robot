import json
import os
import cv2
import sys

# ==================== 自动路径配置 ====================
# 获取当前脚本所在目录（scripts文件夹）
script_dir = os.path.dirname(os.path.abspath(__file__))
# 项目根目录为脚本所在目录的上一级
project_dir = os.path.dirname(script_dir)

# 设置路径（基于项目根目录）
images_dir = os.path.join(project_dir, "images")
json_dir = os.path.join(project_dir, "annotations")
output_dir = os.path.join(project_dir, "datasets", "yolo_det_dataset")

# 你的类别名称列表（根据你的标签修改）
classes = ['si', 'san', 'ti', 'yuan']   # 对应：四边形、三角形、梯形、圆形
# =====================================================

def convert_labelme_to_yolo_det():
    # 创建输出目录
    os.makedirs(os.path.join(output_dir, "images", "train"), exist_ok=True)
    os.makedirs(os.path.join(output_dir, "labels", "train"), exist_ok=True)

    # 检查必要目录是否存在
    if not os.path.exists(images_dir):
        print(f"错误: 图片目录不存在 {images_dir}")
        sys.exit(1)
    if not os.path.exists(json_dir):
        print(f"错误: 标注目录不存在 {json_dir}")
        sys.exit(1)

    # 遍历所有json文件
    for json_file in os.listdir(json_dir):
        if not json_file.endswith('.json'):
            continue

        json_path = os.path.join(json_dir, json_file)
        with open(json_path, 'r', encoding='utf-8') as f:
            data = json.load(f)

        # 获取对应的图片文件名（去除可能存在的路径）
        image_file = data['imagePath']
        image_file = os.path.basename(image_file)  # 只取文件名
        image_path = os.path.join(images_dir, image_file)

        # 读取图片获取尺寸
        img = cv2.imread(image_path)
        if img is None:
            print(f"警告: 无法读取图片 {image_path}，跳过 {json_file}")
            continue
        img_h, img_w = img.shape[:2]

        # 复制图片到输出目录
        out_img_path = os.path.join(output_dir, "images", "train", image_file)
        cv2.imwrite(out_img_path, img)

        # 准备写入的txt内容
        txt_lines = []

        # 解析标注形状
        shapes = data.get('shapes', [])
        for shape in shapes:
            if shape.get('shape_type') != 'rectangle':
                continue

            label = shape.get('label')
            if label not in classes:
                print(f"警告: 未知标签 '{label}' 在 {json_file} 中，跳过该对象")
                continue

            class_id = classes.index(label)

            points = shape.get('points', [])
            if len(points) != 2:
                print(f"警告: 矩形框点数量错误 {json_file}，跳过该对象")
                continue

            x1, y1 = points[0]
            x2, y2 = points[1]

            x_min = min(x1, x2)
            x_max = max(x1, x2)
            y_min = min(y1, y2)
            y_max = max(y1, y2)

            # 归一化
            box_w = (x_max - x_min) / img_w
            box_h = (y_max - y_min) / img_h
            box_cx = (x_min + x_max) / 2 / img_w
            box_cy = (y_min + y_max) / 2 / img_h

            # 限制范围
            box_cx = max(0, min(box_cx, 1))
            box_cy = max(0, min(box_cy, 1))
            box_w = max(0, min(box_w, 1))
            box_h = max(0, min(box_h, 1))

            line = f"{class_id} {box_cx:.6f} {box_cy:.6f} {box_w:.6f} {box_h:.6f}\n"
            txt_lines.append(line)

        # 写入txt文件
        if txt_lines:
            txt_filename = os.path.splitext(image_file)[0] + '.txt'
            txt_path = os.path.join(output_dir, "labels", "train", txt_filename)
            with open(txt_path, 'w') as f:
                f.writelines(txt_lines)
            print(f"已处理: {json_file} -> {txt_filename}")
        else:
            print(f"警告: {json_file} 中没有有效的矩形框，未生成标签文件")

    print("转换完成！")

if __name__ == '__main__':
    convert_labelme_to_yolo_det()