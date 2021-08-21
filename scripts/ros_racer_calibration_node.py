#!/usr/bin/python3
import os.path
import rospy
import cv2
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from decoder import decodeImage

RR_CALIBRATION_NODE_NAME = 'ros_racer_calibration_node'
CAMERA_TOPIC_NAME = 'camera_rgb'
STEERING_TOPIC_NAME = '/steering'
THROTTLE_TOPIC_NAME = '/throttle'

IMG_WINDOW_NAME = 'img'
BW_WINDOW_NAME = 'blackAndWhiteImage'
MASK_WINDOW_NAME = 'mask'
THR_STR_WINDOW_NAME = 'throttle_and_steering'

cv2.namedWindow(IMG_WINDOW_NAME)
cv2.namedWindow(BW_WINDOW_NAME)
cv2.namedWindow(MASK_WINDOW_NAME)
cv2.namedWindow(THR_STR_WINDOW_NAME)

def callback(x):
    pass

def slider_to_normalized(slider_input):
    input_start = 0
    input_end = 2000
    output_start = -1
    output_end = 1
    normalized_output = output_start + (slider_input - input_start) * (
            (output_end - output_start) / (input_end - input_start))
    return normalized_output

lowH = 0
highH = 179
lowS = 0
highS = 255
lowV = 0
highV = 255

glow = 0
ghigh = 255

not_inverted = 0
inverted = 1

min_width = 10
max_width = 500

max_number_of_lines = 100
max_error_threshold = 100

min_frame_height = 1
max_frame_height = 100
default_frame_width = 100
max_frame_width = 100
default_min_rows = 50
max_rows = 100
default_min_offset = 50
max_offset = 100

steer_left = 0
steer_straight = 1000
steer_right = 2000

steer_sensitivity_max = 100
steer_sensitivity_default = 100

throttle_reverse = 0
throttle_neutral = 1100
throttle_forward = 2000

zero_throttle_mode = 0
zero_error_throttle_mode = 1
error_throttle_mode = 2


cv2.createTrackbar('lowH', MASK_WINDOW_NAME , lowH, highH, callback)
cv2.createTrackbar('highH', MASK_WINDOW_NAME , highH, highH, callback)
cv2.createTrackbar('lowS', MASK_WINDOW_NAME , lowS, highS, callback)
cv2.createTrackbar('highS', MASK_WINDOW_NAME , highS, highS, callback)
cv2.createTrackbar('lowV', MASK_WINDOW_NAME , lowV, highV, callback)
cv2.createTrackbar('highV', MASK_WINDOW_NAME , highV, highV, callback)


cv2.createTrackbar('gray_thresh', BW_WINDOW_NAME , glow, ghigh, callback)
cv2.createTrackbar('Inverted_filter', BW_WINDOW_NAME , not_inverted, inverted, callback)

cv2.createTrackbar('min_width', IMG_WINDOW_NAME, min_width, max_width, callback)
cv2.createTrackbar('max_width', IMG_WINDOW_NAME, max_width, max_width, callback)
cv2.createTrackbar('number_of_lines', IMG_WINDOW_NAME, max_number_of_lines, max_number_of_lines, callback)
cv2.createTrackbar('error_threshold', IMG_WINDOW_NAME, max_error_threshold, max_error_threshold, callback)

cv2.createTrackbar('frame_width', IMG_WINDOW_NAME, default_frame_width, max_frame_width, callback)
cv2.createTrackbar('rows_to_watch', IMG_WINDOW_NAME, default_min_rows, max_rows, callback)
cv2.createTrackbar('rows_offset', IMG_WINDOW_NAME, default_min_offset, max_offset, callback)

cv2.createTrackbar('Steering_sensitivity', THR_STR_WINDOW_NAME, steer_sensitivity_default, steer_sensitivity_max, callback)
cv2.createTrackbar('Steering_value', THR_STR_WINDOW_NAME, steer_straight, steer_right, callback)
cv2.createTrackbar('Throttle_mode', THR_STR_WINDOW_NAME, zero_throttle_mode, error_throttle_mode, callback)
cv2.createTrackbar('Throttle_value', THR_STR_WINDOW_NAME, throttle_neutral, throttle_forward, callback)

class Calibration:
    def __init__(self):
        self.init_node = rospy.init_node(RR_CALIBRATION_NODE_NAME, anonymous=False)
        self.camera_sub = rospy.Subscriber(CAMERA_TOPIC_NAME, Image, self.live_calibration_values)
        self.steering_pub = rospy.Publisher(STEERING_TOPIC_NAME, Float32, queue_size=1)
        self.throttle_pub = rospy.Publisher(THROTTLE_TOPIC_NAME, Float32, queue_size=1)
        self.steering_float = Float32()
        self.throttle_float = Float32()

        # setting default values for actuators
        self.zero_throttle = None
        self.zero_error_throttle = None
        self.error_throttle = None


    def live_calibration_values(self,data):
        # get trackbar positions
        lowH = cv2.getTrackbarPos('lowH', MASK_WINDOW_NAME)
        highH = cv2.getTrackbarPos('highH', MASK_WINDOW_NAME)
        lowS = cv2.getTrackbarPos('lowS', MASK_WINDOW_NAME)
        highS = cv2.getTrackbarPos('highS', MASK_WINDOW_NAME)
        lowV = cv2.getTrackbarPos('lowV', MASK_WINDOW_NAME)
        highV = cv2.getTrackbarPos('highV', MASK_WINDOW_NAME)

        gray_thresh = cv2.getTrackbarPos('gray_thresh', BW_WINDOW_NAME)
        inverted_filter = cv2.getTrackbarPos('Inverted_filter', BW_WINDOW_NAME)

        min_width = cv2.getTrackbarPos('min_width', IMG_WINDOW_NAME)
        max_width = cv2.getTrackbarPos('max_width', IMG_WINDOW_NAME)
        number_of_lines = cv2.getTrackbarPos('number_of_lines', IMG_WINDOW_NAME)
        error_threshold = float(cv2.getTrackbarPos('error_threshold', IMG_WINDOW_NAME)/100)
        crop_width_percent = cv2.getTrackbarPos('frame_width', IMG_WINDOW_NAME)
        rows_to_watch_percent = cv2.getTrackbarPos('rows_to_watch', IMG_WINDOW_NAME)
        rows_offset_percent = cv2.getTrackbarPos('rows_offset', IMG_WINDOW_NAME)

        steer_input = cv2.getTrackbarPos('Steering_value', THR_STR_WINDOW_NAME)
        Steering_sensitivity = float(cv2.getTrackbarPos('Steering_sensitivity', THR_STR_WINDOW_NAME)/100)
        Throttle_mode = cv2.getTrackbarPos('Throttle_mode', THR_STR_WINDOW_NAME)
        throttle_input = cv2.getTrackbarPos('Throttle_value', THR_STR_WINDOW_NAME)

        # Setting throttle and steering values
        if Throttle_mode == 0:
            self.zero_throttle = slider_to_normalized(throttle_input)
        elif Throttle_mode == 1:
            self.zero_error_throttle = slider_to_normalized(throttle_input)
        elif Throttle_mode == 2:
            self.error_throttle = slider_to_normalized(throttle_input)

        self.steering_float.data = Steering_sensitivity*slider_to_normalized(steer_input)
        self.throttle_float.data = slider_to_normalized(throttle_input)

        self.steering_pub.publish(self.steering_float)
        self.throttle_pub.publish(self.throttle_float)

        # Setting lower constraints on camera values
        if crop_width_percent < 1:
            crop_width_percent = 1

        if rows_to_watch_percent < 1:
            rows_to_watch_percent = 1

        if rows_offset_percent < 1:
            rows_offset_percent = 1

        # Image processing from slider values
        frame = decodeImage(data.data, data.height, data.width)
        height, width, channels = frame.shape

        rows_to_watch_decimal = rows_to_watch_percent / 100
        rows_offset_decimal = rows_offset_percent / 100
        crop_width_decimal = crop_width_percent / 100

        rows_to_watch = int(height * rows_to_watch_decimal)
        rows_offset = int(height * (1 - rows_offset_decimal))

        start_height = int(height - rows_offset)
        bottom_height = int(start_height + rows_to_watch)

        left_width = int((width / 2) * (1 - crop_width_decimal))
        right_width = int((width / 2) * (1 + crop_width_decimal))

        img = frame[start_height:bottom_height, left_width:right_width]

        image_width = right_width-left_width
        image_height = bottom_height-start_height

        # changing color space to HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower = np.array([lowH, lowS, lowV])
        higher = np.array([highH, highS, highV])
        mask = cv2.inRange(hsv, lower, higher)

        if inverted_filter == 1:
            bitwise_mask = cv2.bitwise_and(img, img, mask=cv2.bitwise_not(mask))
        else:
            bitwise_mask = cv2.bitwise_and(img, img, mask=mask)

        # changing to gray color space
        gray = cv2.cvtColor(bitwise_mask, cv2.COLOR_BGR2GRAY)

        # changing to black and white color space
        gray_upper = 255
        (dummy, blackAndWhiteImage) = cv2.threshold(gray, gray_thresh, gray_upper, cv2.THRESH_BINARY)
        contours, dummy = cv2.findContours(blackAndWhiteImage, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        centers = []
        cx_list = []
        cy_list = []

        # Creating points to be drawn on image 
        start_point = (int(image_width/2),0)
        end_point = (int(image_width/2),int(bottom_height))

        start_point_thresh_pos_x = int((image_width/2)*(1-error_threshold))
        start_point_thresh_neg_x = int((image_width/2)*(1+error_threshold))
        
        start_point_thresh_pos = (start_point_thresh_pos_x,0)
        end_point_thresh_pos = (start_point_thresh_pos_x, int(bottom_height))

        start_point_thresh_neg = (start_point_thresh_neg_x,0)
        end_point_thresh_neg = (start_point_thresh_neg_x, int(bottom_height))

        # plotting contours and their centroids
        for contour in contours[:number_of_lines]:
            x, y, w, h = cv2.boundingRect(contour)
            if min_width < w < max_width:
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
        try:
            if len(cx_list) > 1:
                error_list = []
                count = 0
                for cx_pos in cx_list:
                    error = float(((image_width/2) - cx_pos) / (image_width/2))
                    error_list.append(error)
                avg_error = (sum(error_list) / float(len(error_list)))
                p_horizon_diff = error_list[0] - error_list[-1]
                if abs(p_horizon_diff) <=error_threshold:
                    error_x = avg_error
                    pixel_error = int((image_width/2)*(1-error_x))
                    mid_x, mid_y = pixel_error, int((image_height/2))
                    rospy.loginfo(f"straight curve: {error_x}, {error_list}")
                else: 
                    for error in error_list:
                        if abs(error) < error_threshold:
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
                centers = []
                cx_list = []
                cy_list = []
            elif len(cx_list) == 1:
                mid_x, mid_y = cx_list[0], cy_list[0]
                rospy.loginfo(f"only detected one line")
                cv2.circle(img, (mid_x, mid_y), 7, (0, 0, 255), -1)
            else:
                pass
        except ValueError:
            pass

        # plotting results
        cv2.imshow(IMG_WINDOW_NAME, img)
        cv2.imshow(MASK_WINDOW_NAME, mask)
        # cv2.imshow('bitwise_mask', bitwise_mask)
        # cv2.imshow('gray', gray)
        cv2.imshow(BW_WINDOW_NAME, blackAndWhiteImage)
        cv2.waitKey(1)

        # Write files to yaml file for storage
        f = open(os.path.dirname(__file__) + '/../config/ros_racer_calibration.yaml', "w")
        f.write(f"Hue_low : {lowH} \n"
                f"Hue_high : {highH} \n"
                f"Saturation_low : {lowS} \n"
                f"Saturation_high : {highS} \n"
                f"Value_low : {lowV} \n"
                f"Value_high : {highV} \n"
                f"gray_thresh : {gray_thresh} \n"
                f"number_of_lines : {number_of_lines} \n"
                f"error_threshold : {error_threshold} \n"
                f"Width_min : {min_width} \n"
                f"Width_max : {max_width} \n"
                f"inverted_filter : {inverted_filter} \n"
                f"camera_start_height : {start_height} \n"
                f"camera_bottom_height : {bottom_height} \n"
                f"camera_left_width : {left_width} \n"
                f"camera_right_width : {right_width} \n"
                f"steering_sensitivity : {Steering_sensitivity} \n"
                f"zero_throttle : {self.zero_throttle} \n"
                f"no_error_throttle : {self.zero_error_throttle} \n"
                f"error_throttle : {self.error_throttle} \n"
                )
        f.close()

def main():
    RR_calibration = Calibration()
    rate = rospy.Rate(15)
    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()

if __name__ == '__main__':
    main()
