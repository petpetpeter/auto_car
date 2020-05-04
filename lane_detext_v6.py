# -*- coding: utf-8 -*-
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
from math import *
from collections import deque, defaultdict
def detect_edges(frame):
    # filter for blue lane lines
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    cv2.imshow("hsv", hsv)
    cv2.waitKey(1)
    lower_blue = np.array([60, 40, 40])
    upper_blue = np.array([150, 255, 255])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    #cv2.imshow("blue mask", mask)
    #cv2.waitKey(1)
    # detect edges
    edges = cv2.Canny(mask, 200, 400)

    return edges

def region_of_interest(edges):
    height, width = edges.shape
    mask = np.zeros_like(edges)

    # only focus bottom half of the screen
    polygon = np.array([[
        (0, height*0.6),
        (width*1/3, height*0.5),
        (width*1.8/3, height*0.5),
        (width, height*0.6),

    ]], np.int32)
    cv2.fillPoly(mask, polygon, 255)
    cropped_edges = cv2.bitwise_and(edges, mask)
    return cropped_edges


def detect_line_segments(cropped_edges):
    # tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
    rho = 2  # distance precision in pixel, i.e. 1 pixel
    angle = np.pi / 180  # angular precision in radian, i.e. 1 degree
    min_threshold = 50  # minimal of votes
    line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold, 
                                    np.array([]), minLineLength=10, maxLineGap=3)

    return line_segments


def average_slope_intercept(lines):
    avgLines = defaultdict(list)
    weights = defaultdict(list)
    count_left=0
    count_right=0
    for line in lines:
        print(line)
        
        for x1, y1, x2, y2 in line:
            if x2 == x1:
                continue # Ignore a vertical line
            slope = (y2 - y1) / float(x2 - x1)
            slope = floor(slope * 10) / 10

            if slope == 0:
                continue # Avoid division by zero

            intercept = y1 - slope * x1
            length = np.sqrt((y2 - y1)**2 + (x2 - x1)**2)
            
            if slope >0 :
                if count_left==0 :
                    x1_check_left=x1
                    count_left+=1
                    leftLane=[[x1, y1, x2, y2]]
                if  x1>x1_check_left:   
                    leftLane=[[x1, y1, x2, y2]]
            if slope <0 :
                if count_right==0 :
                    x1_check_right=x1
                    count_right+=1
                    rightLane=[[x1, y1, x2, y2]]
                if  x1>x1_check_right:   
                    rightLane=[[x1, y1, x2, y2]]
                
    return leftLane, rightLane 
def makeLinePoints(y1, y2, line):
    """
    Convert a line represented in slope and intercept into pixel points
    """
    if line is None:
        return None
    
    slope, intercept = line
    
    # cv2.line requires integers
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    y1 = int(y1)
    y2 = int(y2)
    
    return [[x1, y1, x2, y2]]
def laneLines(image, lines):
    leftLane, rightLane = average_slope_intercept(lines)
    
    y1 = image.shape[0] # Bottom of the image
    y2 = y1 * 0.6       # Slightly lower than the middle

    leftLine  = makeLinePoints(y1, y2, leftLane)
    rightLine = makeLinePoints(y1, y2, rightLane)
    emp_lst = []
    emp_lst.append(leftLine)
    emp_lst.append(rightLine)
    return emp_lst
def detect_lane(frame):
    
    edges = detect_edges(frame)
    cropped_edges = region_of_interest(edges)
    cv2.imshow("ada ads", cropped_edges)
    line_segments = detect_line_segments(cropped_edges)
    line_segs_img=display_lines(frame, line_segments)
    cv2.imshow("lane ads", line_segs_img)
    lane_lines =average_slope_intercept(line_segments)
    
    return  lane_lines
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
        camera_mid_offset_percent = 0.02 # 0.0 means car pointing to center, -0.03: car is centered to left, +0.03 means car pointing to right
        mid = int(width / 2 * (1 + camera_mid_offset_percent))
        x_offset = (left_x2 + right_x2) / 2 - mid

    # find the steering angle, which is angle between navigation direction to end of center line
    y_offset = int(height / 2)

    angle_to_mid_radian = math.atan(x_offset / y_offset)  # angle (in radian) to center vertical line
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  # angle (in degrees) to center vertical line
    steering_angle = angle_to_mid_deg + 90  # this is the steering angle needed by picar front wheel

    logging.debug('new steering angle: %s' % steering_angle)
    return steering_angle




def main():
    frame = cv2.imread('D:\\tank\Year4term_2\Mechatronics_2/f0.jpg')
    #cv2.imshow("frame",frame)
    #cv2.waitKey(1)
    lane_lines=detect_lane(frame)
    lane_lines_image = display_lines(frame, lane_lines)
    steering_angle = compute_steering_angle(frame,lane_lines)
    cv2.imshow("lane lines", lane_lines_image)
    print(steering_angle)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__=="__main__" :
    main()