import os
import random
import shutil

DATASET_ROOT = "./dataset"
VAL_RATIO = 0.2  # 移动 20% 的数据到验证集

def main():
    # 设定路径
    train_img_dir = os.path.join(DATASET_ROOT, "images", "train")
    val_img_dir = os.path.join(DATASET_ROOT, "images", "val")
    train_lbl_dir = os.path.join(DATASET_ROOT, "labels", "train")
    val_lbl_dir = os.path.join(DATASET_ROOT, "labels", "val")

    # 检查训练集是否存在
    if not os.path.exists(train_img_dir):
        print(f"[ERROR] 找不到训练集目录: {train_img_dir}")
        return

    # 获取所有图片文件
    images = [f for f in os.listdir(train_img_dir) if f.endswith('.jpg')]
    if len(images) == 0:
        print("[INFO] train 文件夹中没有图片，无需移动。")
        return

    # 计算移动数量
    num_val = int(len(images) * VAL_RATIO)
    print(f"[INFO] 检测到 {len(images)} 张图片，准备移动 {num_val} 张到验证集...")

    # 随机选择要移动的文件
    val_images = random.sample(images, num_val)

    # 开始移动
    count = 0
    for img_name in val_images:
        # 图片路径
        src_img = os.path.join(train_img_dir, img_name)
        dst_img = os.path.join(val_img_dir, img_name)
        
        # 对应标签路径
        label_name = img_name.replace('.jpg', '.txt')
        src_lbl = os.path.join(train_lbl_dir, label_name)
        dst_lbl = os.path.join(val_lbl_dir, label_name)

        # 执行移动
        if os.path.exists(src_img):
            shutil.move(src_img, dst_img)
        if os.path.exists(src_lbl):
            shutil.move(src_lbl, dst_lbl)
        count += 1

    # 清理旧的缓存文件 (防止 YOLO 报错)
    for cache in ["train.cache", "val.cache"]:
        cache_path = os.path.join(DATASET_ROOT, "labels", cache)
        if os.path.exists(cache_path):
            os.remove(cache_path)

    print(f"[SUCCESS] 已成功移动 {count} 组数据到 val 文件夹。")

if __name__ == "__main__":
    main()
