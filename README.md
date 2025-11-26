# ROS2 智慧型羽球自動收集機器人
## Intelligent Shuttlecock Collection Robot Based on ROS2 and YOLOv8

**國立雲林科技大學 資訊工程系  
大學部實務專題（June 2026）**

指導教授：李建緯 博士  
專題成員：高啟軒、利瑋哲、唐禮直

---

## 📌 專題簡介
本專題開發一台可在羽球場中自動偵測並收集羽球的智慧型移動機器人。  
系統整合 **ROS2 導航（Nav2）**、**SLAM 建圖**、**機器學習導航策略**，並使用 **YOLOv8 深度學習模型** 進行羽球物件偵測。  
機器人能夠自主定位、移動、避障並完成羽球收集任務。

---

## 📌 系統架構
- **感測器：** LiDAR、深度相機（Astra Pro / RealSense）、編碼器、IMU  
- **物件偵測：** YOLOv8  
- **導航：** ROS2 Galactic + Nav2  
- **控制：** ROS2 節點管理 / tf2 座標轉換  
- **機構：** 雙輪差速底盤、自動收集機構

---

## 📌 開發環境
- Ubuntu 20.04  
- ROS2 Galactic（0.10.3）  
- Python 3.8  
- PyTorch、Ultralytics YOLOv8  
- SLAM Toolbox、Nav2、OpenCV

---

## 📌 系統流程
1. 相機取得影像  
2. YOLOv8 偵測羽球  
3. 取得羽球相對位置並轉換至地圖座標  
4. Nav2 進行路徑規劃與避障  
5. 前往羽球並執行收集動作

---

## 📌 結果
- 可在室內羽球場域進行自主移動與避障  
- YOLOv8 可穩定偵測羽球  
- 系統可完成多顆羽球的持續收集流程  

---

## 📌 關鍵字
ROS2、YOLOv8、Nav2、SLAM、自主導航、智慧機器人
