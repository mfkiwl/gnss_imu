#!/usr/bin/env python

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import math
import numpy as np
import re
import message_filters

from gnss_imu.msg import raw
from gnss_imu.msg import gnss_raw

img_width_ori = 640
img_width = 480
img_height = 480
img_width_center = 240 # 317.181 
img_height_center = 240 # 244.3747
img_offset = 80
font_offset_x = 10
font_offset_y = 5

# millimeter
focal_length = 109
# elevation threshold
elev_threshold = 25
# carrier to noise ratio threshold
cno_threshold = 35

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
    # show_img(img)

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

# def ImageCallback(img_msg):

#     try:
#         cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
#     except CvBridgeError, e:
#         rospy.logerr("CvBridge Error: {0}".format(e))

#     plotSatelliteToImage(cv_image)
    # sky_segmantation(cv_image)

def plotSatelliteToImage(img):
    print(len(sat2plot))
    show_img(img)

def calcutateSatelliteToImage(image_sub, satellite_sub): 
    try:
        cv_image_ori = bridge.imgmsg_to_cv2(image_sub, "passthrough")
        
        # image crop
        cv_image_ori = cv_image_ori[:, img_offset:img_width_ori - img_offset]
        cv_image = cv_image_ori.copy()

    except CvBridgeError, e:
        rospy.logerr("CvBridge Error: {0}".format(e))

    # check satellite projection
    # cv2.circle(cv_image, (int(img_width_center), int(img_height_center)), 5, (255, 0, 0), -1)
    cv2.line(cv_image, (img_width_center, 0), (img_width_center, img_height), (255, 0, 0), 1)
    cv2.line(cv_image, (0, img_height_center), (img_width, img_height_center), (255, 0, 0), 1)
    
    # gain ros message
    vehicle_heading = satellite_sub.heading
    num_sv = satellite_sub.numSvs
    meas = str(satellite_sub.meas)

    # classification
    gps = meas.split(",", num_sv)
    for i in range(num_sv):
        gps[i] = re.findall(r"\d+\.?\d*", gps[i])
        print(gps[i])

    # transfer satillite to image
    sat2plot = []
    for i in range(num_sv):
        # exclude unhealthy satellite
        if(int(gps[i][1]) <= elev_threshold or int(gps[i][3]) <= cno_threshold):continue

        # global cordinate to image plane
        theta = 90 - int(gps[i][1])
        rpix = 2 * focal_length * math.tan(math.radians(theta / 2))
        
        x_sat = img_width_center - rpix * math.cos(math.radians(vehicle_heading + int(gps[i][2]) + 90))
        y_sat = img_height_center + rpix * math.sin(math.radians(vehicle_heading + int(gps[i][2]) + 90))
        # x_sat = img_width_center + rpix * math.cos(math.radians(vehicle_heading + int(gps[i][2])))
        # y_sat = img_height_center - rpix * math.sin(math.radians(vehicle_heading + int(gps[i][2])))

        print(int(gps[i][0]), theta, rpix, x_sat, y_sat)
        
        # if(x_sat >= img_width):
        #     x_sat = img_width
        # elif(x_sat <= 0):
        #     x_sat = 0
        # if(y_sat >= img_height):
        #     y_sat = img_height
        # elif(y_sat <= 0):
        #     y_sat = 0
        
        sat2plot.append([int(gps[i][0]), int(x_sat), int(y_sat)])
        cv2.circle(cv_image, (int(x_sat), int(y_sat)), 10, (0, 0, 255), -1)
        cv2.putText(cv_image, str(gps[i][0]), (int(x_sat) - font_offset_x, int(y_sat) + font_offset_y), 
                    cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.8, (0, 255, 0), 1)


    if not sat2plot:
        print("Waiting for GNSS Raw Data from Topic /ublox_f9k/navsat...")
    else:
        
        # print(sat2plot)
        print("---------------------------------------")

    show_img(cv_image)

rospy.loginfo('Waiting for topics...')
rospy.wait_for_message("/nm33/image_raw", Image)
rospy.wait_for_message("/gps_raw", gnss_raw)
rospy.loginfo('Time synchronize!')

image_sub = message_filters.Subscriber("/nm33/image_raw", Image)
satellite_sub = message_filters.Subscriber("/gps_raw", gnss_raw)

# Synchronize images
sub = message_filters.ApproximateTimeSynchronizer([image_sub, satellite_sub], 1, 1, allow_headerless=True)
sub.registerCallback(calcutateSatelliteToImage)


while not rospy.is_shutdown():
    rospy.spin()