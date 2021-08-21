#!/usr/bin/python3
import rospy
import cv2
from cv_bridge import CvBridge
import numpy as np
from std_msgs.msg import Int32, Int32MultiArray, Float32
from sensor_msgs.msg import Image
from decoder import decodeImage
import time


LANE_DETECTION_NODE_NAME = 'lane_detection_node'
CAMERA_TOPIC_NAME = 'camera_rgb'
CENTROID_TOPIC_NAME = '/centroid'

global mid_x, mid_y
mid_x = Int32()
mid_y = Int32()


class LaneDetection:
    def __init__(self):
        # Initialize node and create publishers/subscribers
        self.init_node = rospy.init_node(LANE_DETECTION_NODE_NAME, anonymous=False)
        self.camera_subscriber = rospy.Subscriber(CAMERA_TOPIC_NAME, Image, self.locate_centroid)
        self.centroid_error_publisher = rospy.Publisher(CENTROID_TOPIC_NAME, Float32, queue_size=1)
        self.centroid_error = Float32()
        self.bridge = CvBridge()

        # Getting ROS parameters set from calibration Node
        self.Hue_low = rospy.get_param('Hue_low')
        self.Hue_high = rospy.get_param('Hue_high')
        self.Saturation_low = rospy.get_param('Saturation_low')
        self.Saturation_high = rospy.get_param('Saturation_high')
        self.Value_low = rospy.get_param('Value_low')
        self.Value_high = rospy.get_param('Value_high')
        self.gray_lower = rospy.get_param('gray_thresh')
        self.inverted_filter = rospy.get_param('inverted_filter')
        self.number_of_lines = rospy.get_param('number_of_lines')
        self.error_threshold = rospy.get_param('error_threshold')
        self.min_width = rospy.get_param('Width_min')
        self.max_width = rospy.get_param('Width_max')
        self.start_height = rospy.get_param('camera_start_height')
        self.bottom_height = rospy.get_param('camera_bottom_height')
        self.left_width = rospy.get_param('camera_left_width')
        self.right_width = rospy.get_param('camera_right_width')

        # Display Parameters
        rospy.loginfo(
            f'\nHue_low: {self.Hue_low}'
            f'\nHue_high: {self.Hue_high}'
            f'\nSaturation_low: {self.Saturation_low}'
            f'\nSaturation_high: {self.Saturation_high}'
            f'\nValue_low: {self.Value_low}'
            f'\nValue_high: {self.Value_high}'
            f'\ngray_lower: {self.gray_lower}'
            f'\ninverted_filter: {self.inverted_filter}'
            f'\nnumber_of_lines: {self.number_of_lines}'
            f'\nerror_threshold: {self.error_threshold}'
            f'\nmin_width: {self.min_width}'
            f'\nmax_width: {self.max_width}'
            f'\nstart_height: {self.start_height}'
            f'\nbottom_height: {self.bottom_height}'
            f'\nleft_width: {self.left_width}'
            f'\nright_width: {self.right_width}')

    def locate_centroid(self, data):
        # Image processing from rosparams
        frame = self.bridge.imgmsg_to_cv2(data)

        self.image_width = int(self.right_width - self.left_width)
        img = frame[self.start_height:self.bottom_height, self.left_width:self.right_width]
        image_width = self.right_width-self.left_width
        image_height = self.bottom_height-self.start_height

        # changing color space to HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # setting threshold limits for white color filter
        lower = np.array([self.Hue_low, self.Saturation_low, self.Value_low])
        upper = np.array([self.Hue_high, self.Saturation_high, self.Value_high])
        mask = cv2.inRange(hsv, lower, upper)

        # creating true/false image
        if self.inverted_filter == 1:
            bitwise_mask = cv2.bitwise_and(img, img, mask=cv2.bitwise_not(mask))
        else:
            bitwise_mask = cv2.bitwise_and(img, img, mask=mask)

        # changing to gray color space
        gray = cv2.cvtColor(bitwise_mask, cv2.COLOR_BGR2GRAY)

        # changing to black and white color space
        # gray_lower = 50
        gray_upper = 255
        (dummy, blackAndWhiteImage) = cv2.threshold(gray, self.gray_lower, gray_upper, cv2.THRESH_BINARY)
        contours, dummy = cv2.findContours(blackAndWhiteImage, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        # Setting up data arrays
        centers = []
        cx_list = []
        cy_list = []

        # Defining points of a line to be drawn for visualizing error
        start_point = (int(self.image_width/2),0)
        end_point = (int(self.image_width/2),int(self.bottom_height))

        start_point_thresh_pos_x = int((self.image_width/2)*(1-self.error_threshold))
        start_point_thresh_neg_x = int((self.image_width/2)*(1+self.error_threshold))
        
        start_point_thresh_pos = (start_point_thresh_pos_x,0)
        end_point_thresh_pos = (start_point_thresh_pos_x, int(self.bottom_height))

        start_point_thresh_neg = (start_point_thresh_neg_x,0)
        end_point_thresh_neg = (start_point_thresh_neg_x, int(self.bottom_height))

        # plotting contours and their centroids
        for contour in contours[:self.number_of_lines]:
            x, y, w, h = cv2.boundingRect(contour)
            if self.min_width < w < self.max_width:
                try:
                    x, y, w, h = cv2.boundingRect(contour)
                    img = cv2.drawContours(img, contour, -1, (0, 255, 0), 3)
                    m = cv2.moments(contour)
                    cx = int(m['m10'] / m['m00'])
                    cy = int(m['m01'] / m['m00'])
                    centers.append([cx, cy])
                    cx_list.append(cx)
                    cy_list.append(cy)
                    cv2.circle(img, (cx, cy), 7, (0, 255, 0), -1)
                    img = cv2.line(img, start_point, end_point, (0,255,0), 4)
                    img = cv2.line(img, start_point_thresh_pos, end_point_thresh_pos, (0,0,255), 2)
                    img = cv2.line(img, start_point_thresh_neg, end_point_thresh_neg, (0,0,255), 2)
                except ZeroDivisionError:
                    pass
        # Further image processing to determine optimal steering value
        try:
            if len(cx_list) > 1:
                error_list = []
                count = 0
                for cx_pos in cx_list:
                    error = float(((self.image_width/2) - cx_pos) / (self.image_width/2))
                    error_list.append(error)
                avg_error = (sum(error_list) / float(len(error_list)))
                p_horizon_diff = error_list[0] - error_list[-1]
                if abs(p_horizon_diff) <= self.error_threshold:
                    error_x = avg_error
                    pixel_error = int((self.image_width/2)*(1-error_x))
                    mid_x, mid_y = pixel_error, int((image_height/2))
                    rospy.loginfo(f"straight curve: {error_x}, {error_list}")
                else: 
                    for error in error_list:
                        if abs(error) < self.error_threshold:
                            error = 1
                            error_list[count] = error
                        count+=1
                    error_x = min(error_list, key=abs)
                    error_x_index = error_list.index(min(error_list, key=abs))
                    mid_x, mid_y = cx_list[error_x_index], cy_list[error_x_index]
                    rospy.loginfo(f"curvy road: {error_x}, {error_list}")
                
                cv2.circle(img, (mid_x, mid_y), 7, (255, 0, 0), -1)
                start_point_error = (int(image_width/2), mid_y)
                img = cv2.line(img, start_point_error, (mid_x, mid_y), (0,0,255), 4)
                self.centroid_error.data = float(error_x)
                self.centroid_error_publisher.publish(self.centroid_error)
                centers = []
                cx_list = []
                cy_list = []
            elif len(cx_list) == 1:
                mid_x, mid_y = cx_list[0], cy_list[0]
                error_x = float(((self.image_width/2) - mid_x) / (self.image_width/2))
                cv2.circle(img, (mid_x, mid_y), 7, (0, 0, 255), -1)
                self.centroid_error.data = error_x
                self.centroid_error_publisher.publish(self.centroid_error)
                rospy.loginfo(f"only detected one line")

            centers = []
            cx_list = []
            cy_list = []
            # error_list = [0] * self.number_of_lines
            error_list = []
        except ValueError:
            pass

        # plotting results
        cv2.imshow('img', img)
        cv2.imshow('blackAndWhiteImage', blackAndWhiteImage)
        cv2.waitKey(1)


def main():
    lane_detector = LaneDetection()
    rate = rospy.Rate(15)
    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()


if __name__ == '__main__':
    main()
