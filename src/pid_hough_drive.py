#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, rospkg
import numpy as np
import cv2, random, math
from cv_bridge import CvBridge
from xycar_motor.msg import xycar_motor
from sensor_msgs.msg import Image
import time

import sys
import os
import signal

def PID(input_data, kp, ki, kd):
    global start_time, end_time, prev_error, i_error
    end_time = time.time()
    dt = end_time - start_time
    start_time = end_time

    error = 320 - input_data
    derror = error - prev_error

    p_error = kp * error
    i_error = i_error + ki * error * dt
    d_error = kd * derror / dt

    output = p_error + i_error + d_error
    prev_error = error

    if output > 50:
        output = 50
    elif output < -50:
        output = -50
    
    return -output

def start():
    global pub, image, cap, Width, Height

    rospy.init_node('auto_drive')
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

    image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    print "------------------Xycar A2 v1.0 ------------------------"
    rospy.sleep(2)

    while True:
        while not image.size == (640*480*3):
            continue

        lpos, rpos = process_image(image)

        center = (lpos + rpos) / 2
        angle = PID(center, 0.45, 0.0007, 0.25)
        drive(angle, speed)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    rospy.spin()

if __name__ == '__main__':

    i_error = 0.0
    prev_error = 0.0
    start_time = time.time()

    start()
