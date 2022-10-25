#!/usr/bin/env python

import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import math
import numpy as np

rospy.init_node('NLOS_detect', anonymous=True)
bridge = CvBridge()

def sky_segmantation(img):
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur_img = cv2.medianBlur(gray_img, 5)
    blur_img = cv2.blur(blur_img, (7,7))
    blur_img = cv2.blur(blur_img, (5,5)) 
    cont_img = modify_contrast_and_brightness2(blur_img)
    canny_img = cv2.Canny(cont_img, 10, 210)
    # ret, bin_img = cv2.threshold(canny_img, 127, 255, cv2.THRESH_BINARY)
    show_img(canny_img)

def modify_contrast_and_brightness2(img):
    brightness = 0
    contrast = 130 # parm for decrease or increase the contrast

    B = brightness / 255.0
    c = contrast / 255.0 
    k = math.tan((45 + 44 * c) / 180 * math.pi)

    img = (img - 127.5 * (1 - B)) * k + 127.5 * (1 + B)
    img = np.clip(img, 0, 255).astype(np.uint8)
    return img

def show_img(img):
    cv2.imshow("Image", img)
    cv2.waitKey(13)

def image_callback(img_msg):

    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    except CvBridgeError, e:
        rospy.logerr("CvBridge Error: {0}".format(e))

    sky_segmantation(cv_image)


sub_image = rospy.Subscriber("/nm33/image_raw", Image, image_callback)

while not rospy.is_shutdown():
    rospy.spin()