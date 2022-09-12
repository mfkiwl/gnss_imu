import cv2

img_height = 480
img_width = 680


cap =  cv2.VideoCapture(0)
if not (cap.isOpened()):
    print("Wrong video device, please check video source again")
else:
    print("Video device is opened")

cap.set(cv2.CAP_PROP_FRAME_WIDTH, img_width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, img_height)


while(True):
    ret, frame = cap.read()
    cv2.imshow('NM33_image', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()