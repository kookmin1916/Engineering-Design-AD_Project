#!/usr/bin/env python

from colordetect import *
import cv2, time, rospy
import numpy as np
from obstacledetector import *
from motordriver import *

motor_driver = MotorDriver("xycar_motor_msg")
rospy.init_node("AD_Project_node")
color_detect = ColorDetector("/usb_cam/image_raw")
obs = ObstacleDetector("/ultrasonic")

time.sleep(2)
averRadius = []
angle = 90
speed = 90
last = 90
average = None
while True:
    if(cv2.waitKey(1) > 0):
        break
    frame = color_detect.cap
    pos, radius = color_detect.find_red()

    if len(averRadius) < 30:
        if radius is not None:
            averRadius.append(radius)
        continue
    elif len(averRadius) == 30 and average is None:
        average = sum(averRadius) / 30


    l, m, r = obs.get_distance()
    if pos is not None:
        x, y = pos
	angle = 90 + float(50) / 320 * (x - 320)
        speed = 90 + (1 - radius / average) * 100
        speed = min(max(speed, 40), 140)
        speed = 90 if 55 <= speed <= 100 else speed
        if speed < 90:
            angle = 180 - angle
        print(speed)
        """
        speed = 130
        if 1 < m < 60:
            for i in range(3):
                motor_driver.drive(90, 90)
                time.sleep(0.01)
                motor_driver.drive(90, 80)
                time.sleep(0.01)
            speed = 90
        """
    else:
        speed = 90
    if speed < 90 and last >= 90:
        for i in range(2):
            motor_driver.drive(90, 90)
            time.sleep(0.1)
            motor_driver.drive(90, 70)
            time.sleep(0.1)
    motor_driver.drive(angle, speed)
    last = speed
    #cv2.imshow("Camera", frame)
print("Finish")
