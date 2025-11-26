import cv2
import numpy as np

def main():
    cap = cv2.VideoCapture(0)  # 開啟攝影機

    if not cap.isOpened():
        print("Error: Unable to open camera.")
        return

    # 設置攝影機格式為 MJPG
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # 設定影像寬度
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # 設定影像高度
    cap.set(cv2.CAP_PROP_FPS, 30)  # 設定幀率

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Unable to read frame.")
            break

        # 轉換為 HSV 顏色空間
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 定義羽球顏色範圍（根據實際顏色進行調整）
        lower_yellow = np.array([20, 100, 100])  # 調整此範圍以適應羽球的顏色
        upper_yellow = np.array([30, 255, 255])

        # 創建顏色遮罩
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # 查找輪廓
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 100:  # 可根據實際情況調整
                x, y, w, h = cv2.boundingRect(contour)
                center_x = x + w // 2
                center_y = y + h // 2

                # 畫出紅色邊界框
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)

                # 在畫面中顯示 (x, y) 距離
                cv2.putText(frame, f"X: {center_x}, Y: {center_y}", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # 顯示影像
        cv2.imshow('Camera Image', frame)

        # 按 'q' 鍵退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
