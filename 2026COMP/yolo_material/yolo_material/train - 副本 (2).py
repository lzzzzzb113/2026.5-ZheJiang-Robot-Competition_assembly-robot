import os
import shutil
from ultralytics import YOLO
import torch
def restore_val_to_train(dataset_root):
    """
    将 val 文件夹中的所有数据移回 train 文件夹
    """
    print("\n" + "-" * 30)
    print("[INFO] 正在将验证集数据归位到训练集...")
    
    paths = [
        ('images/val', 'images/train'),
        ('labels/val', 'labels/train')
    ]
    
    move_count = 0
    for val_sub, train_sub in paths:
        val_dir = os.path.join(dataset_root, val_sub)
        train_dir = os.path.join(dataset_root, train_sub)
        
        if os.path.exists(val_dir):
            files = os.listdir(val_dir)
            for f in files:
                # 移动文件
                shutil.move(os.path.join(val_dir, f), os.path.join(train_dir, f))
                move_count += 1
                
    # 同时清理掉缓存文件，防止下次训练冲突
    for cache in ["train.cache", "val.cache"]:
        cache_path = os.path.join(dataset_root, "labels", cache)
        if os.path.exists(cache_path):
            os.remove(cache_path)

    print(f"[SUCCESS] 归位完成，共移动 {move_count} 个文件，已清理缓存。")
    print("-" * 30)

def main():
    device = "cuda:0" if torch.cuda.is_available() else "cpu"       
    if device == "cpu":
        print("[WARNING] 未检测到 GPU，当前为 CPU 模式！")
    else:
        print(f"[INFO] 显卡已就绪: {torch.cuda.get_device_name(0)}")
        torch.backends.cudnn.benchmark = True 

    model = YOLO("yolo26m.pt")

    # 开始训练
    results = model.train(
        # ---------- 数据集 ----------
        data="./data.yaml",          
        epochs=200,                  
        patience=15,
        batch=12,     
        amp=True ,         
        imgsz=640,                   

        # ---------- 高阶数据增强 (抗光照核心，不吃显存) ----------
        hsv_h=0.015,              
        hsv_s=0.7,                   
        hsv_v=0.5,                   
        degrees=15.0,            
        translate=0.2,            
        scale=0.6,                  
        shear=2.0,                  
        perspective=0.0,
        mosaic=1.0,
        fliplr=0.5,                  
        flipud=0.5,                
        erasing=0.3,                 
        mixup=0.1,                   
        copy_paste=0.0,
        
        # ---------- 优化器与策略 ----------
        optimizer="AdamW",           
        lr0=0.002,                   
        lrf=0.05,                    
        cos_lr=True,                 
        close_mosaic=10,           
        weight_decay=0.0005,         
        warmup_epochs=3.0,           
        warmup_momentum=0.8,         

        # ---------- 验证与保存 ----------
        val=True,                    
        save=True,                   
        save_period=0,              
        project="runs/train1",        
        name="material_yolov8",
        exist_ok=True,               
        pretrained=True,             
        verbose=True,                

        # ---------- 硬件资源分配 ----------
        device=device,               
        workers=4,
    )

    print("\n[FINISH] 训练任务已完成！")
    
    # 自动执行归位功能
    try:
        restore_val_to_train("./dataset")
    except Exception as e:
        print(f"[ERROR] 数据归位失败: {e}")

    print("=" * 60)
    print("所有流程结束。")
    print("最佳权重位置: runs/train1/material_yolov8/weights/best.pt")
    print("=" * 60)

if __name__ == "__main__":
    main()