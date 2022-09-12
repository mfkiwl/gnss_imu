#!/usr/bin/env python

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospy

cap =  cv2.VideoCapture(0)
if not (cap.isOpened()):
    print("Wrong video device, please check video source again")
else:
    print("Video device is opened")
bridge = CvBridge()

def talker():
    pub = rospy.Publisher("/nm33/image_raw", Image, queue_size=1)
    rospy.init_node("nm33_img_publisher", anonymous= False)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            print("Didn't recieve image")
            break

        msg = bridge.cv2_to_imgmsg(frame, "bgr8")
        pub.publish(msg)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass