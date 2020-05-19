#!/usr/bin/python
"""
Created on Sat May  2 12:39:05 2020
@author: tank
"""
import numpy as np
import cv2
import logging
import math
import datetime
import sys
import time
import rospy
from math import *
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from cv_bridge         import CvBridge, CvBridgeError
from sensor_msgs.msg        import Image
from collections import deque, defaultdict
from std_msgs.msg import String
import os
### ros setting ####
#image_pub = rospy.Publisher("/blob/image_blob",Image,queue_size=1)
#mask_pub = rospy.Publisher("/blob/image_mask",Image,queue_size=1)
bridge = CvBridge()
vel_pub = rospy.Publisher('vel_text', String, queue_size=10)
y_crop = 0.7
######################
def callback(data):
        #--- Assuming image is 320x240
    try:
       frame = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
    
    #s = cv2.waitKey(1)
    cv2.imshow("frame",frame)
    #path = '/home/chanapol/catkin_ws/src/test_cv/scripts/lanepics'
    cv2.waitKey(1)
    lane_lines=detect_lane(frame)
    #frame,lines = process(frame)
    #lane_lines = average_slope_intercept(frame, lines)
    #fin_img    = drow_the_lines(frame, lane_lines)
    lane_lines_image = display_lines(frame, lane_lines)
    steering_angle = compute_steering_angle(frame,lane_lines)
    cv2.imshow("lane lines", lane_lines_image)
    #cv2.waitKey(1)
    print(steering_angle)
    #if (s == ord('s')):
    #    cv2.imwrite(path+'/f'+str(i)+'.jpg',frame)
    #    global i
    #    i += 1
    #    print('img_save')
    #    ()
    pub_steer(steering_angle)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()    
def detect_edges(frame):
    # filter for blue lane lines
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #cv2.imshow("hsv", hsv)
    cv2.waitKey(1)
    lower_blue = np.array([60, 40, 40])
    upper_blue = np.array([150, 255, 255])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    #cv2.imshow("blue mask", mask)
    #cv2.waitKey(1)
    # detect edges
    edges = cv2.Canny(mask, 200, 400)
    #cv2.imshow('edge',edges)
    return edges

def region_of_interest(edges):
    height, width = edges.shape
    mask = np.zeros_like(edges)

    # only focus bottom half of the screen
    polygon = np.array([[
        (0, height * y_crop),
        (width, height * y_crop),
        (width, height),
        (0, height),
    ]], np.int32)

    cv2.fillPoly(mask, polygon, 255)
    cropped_edges = cv2.bitwise_and(edges, mask)
    #cv2.imshow('crop',cropped_edges)
    return cropped_edges


def detect_line_segments(cropped_edges):
    # tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
    rho = 1  # distance precision in pixel, i.e. 1 pixel
    angle = np.pi / 180  # angular precision in radian, i.e. 1 degree
    min_threshold = 10  # minimal of votes
    line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold, 
                                    np.array([]), minLineLength=8, maxLineGap=4)

    return line_segments


def average_slope_intercept(frame, line_segments):
    """
    This function combines line segments into one or two lane lines
    If all line slopes are < 0: then we only have detected left lane
    If all line slopes are > 0: then we only have detected right lane
    """
    lane_lines = []
    if line_segments is None:
        logging.info('No line_segment segments detected')
        return lane_lines

    height, width, _ = frame.shape
    left_fit = []
    right_fit = []

    boundary = 0.5
    left_region_boundary = width * (1-boundary)# (1-boundary)  # left lane line segment should be on left 2/3 of the screen
    right_region_boundary = width * (boundary) # right lane line segment should be on left 1/3 of the screen
    
    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                logging.info('skipping vertical line segment (slope=inf): %s' % line_segment)
                continue
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = fit[0]
            intercept = fit[1]
            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
                    print(x1,x2,y1,y2,4444)
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))
   # print(len(left_fit),-1)
    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))

    logging.debug('lane lines: %s' % lane_lines)  # [[[316, 720, 484, 432]], [[1009, 720, 718, 432]]]

    return lane_lines
def make_points(frame, line):
    height, width, _ = frame.shape
    print(height,width,12)
    slope, intercept = line
    y1 = height  # bottom of the frame
    y2 = int(y1 * y_crop)  # make points from middle of the frame down

    # bound the coordinates within the frame
    x1 = max(-width, min(2*width, int((y1 - intercept) / slope)))
    x2 = max(-width, min(2*width, int((y2 - intercept) / slope)))
    return [[x1, y1, x2, y2]]
def detect_lane(frame):
    
    edges = detect_edges(frame)
    cropped_edges = region_of_interest(edges)
    cv2.imshow("croped",cropped_edges)
    line_segments = detect_line_segments(cropped_edges)
    lane_lines = average_slope_intercept(frame, line_segments)
    
    return lane_lines
def display_lines(frame, lines, line_color=(0, 255, 0), line_width=2):
    line_image = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    return line_image
def compute_steering_angle(frame, lane_lines):
    """ Find the steering angle based on lane line coordinate
        We assume that camera is calibrated to point to dead center
    """
    if len(lane_lines) == 0:
        logging.info('No lane lines detected, do nothing')
        return -90

    height, width, _ = frame.shape
    if len(lane_lines) == 1:
        logging.debug('Only detected one lane line, just follow it. %s' % lane_lines[0])
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = x2 - x1
    else:
        _, _, left_x2, _ = lane_lines[0][0]
        _, _, right_x2, _ = lane_lines[1][0]
        print(left_x2,right_x2)
        camera_mid_offset_percent = 0.00 # 0.0 means car pointing to center, -0.03: car is centered to left, +0.03 means car pointing to right
        mid = width / 2.0 * (1 + camera_mid_offset_percent)
        x_offset = (left_x2 + right_x2) / 2.0 - mid

    # find the steering angle, which is angle between navigation direction to end of center line
    y_offset = height * y_crop
    #print(x_offset,y_offset,000)
    angle_to_mid_radian = math.atan(x_offset / y_offset)  # angle (in radian) to center vertical line
    print(angle_to_mid_radian,555)
    angle_to_mid_deg = np.rad2deg(angle_to_mid_radian)  # angle (in degrees) to center vertical line
    #steering_angle = angle_to_mid_deg + 90  # this is the steering angle needed by picar front wheel
    steering_angle = -angle_to_mid_deg/100
    logging.debug('new steering angle: %s' % steering_angle)
    return steering_angle
def pub_steer(steering_angle):
    vel_com = str(steering_angle)
    vel_pub.publish(vel_com)

def main():
    rospy.init_node('lane2steer', anonymous=True)
    frame_sub = rospy.Subscriber("/rrbot/camera1/image_raw",Image,callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__=="__main__" :
    main()