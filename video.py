import cv2
cap = cv2.VideoCapture(0)
# 设置分辨率
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# 开启自动增益和白平衡
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)
cap.set(cv2.CAP_PROP_AUTO_WB, 1)
# Check if camera opened successfully
if (cap.isOpened() == False):
    print("Error opening video stream or file")

while(cap.isOpened()):
    # Capture frame-by-frame
    ret, img = cap.read()
    if not ret:
        break

    cv2.imshow("img",img)

    if cv2.waitKey(25) & 0xFF == ord('q'):
        break

# When everything done, release the video capture object
cap.release()

# Closes all the frames
cv2.destroyAllWindows()
