import cv2

def main():
    # 使用 Astra Pro 的正確裝置，例如 /dev/video0
    cap = cv2.VideoCapture("/dev/video0")  

    # 設置影像格式
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))  # 設置為 MJPG 格式
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # 設置寬度
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # 設置高度

    if not cap.isOpened():
        print("Error: Unable to open camera.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Unable to read frame.")
            break

        # 顯示影像
        cv2.imshow('Astra Pro Camera', frame)

        # 按 'q' 鍵退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 釋放資源
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

