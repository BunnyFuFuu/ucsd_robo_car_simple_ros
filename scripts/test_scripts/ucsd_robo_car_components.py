#!/usr/bin/python3
import cv2
from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)
cv2.namedWindow('sliders')

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

#THESE VALUES ARE ALL SCALED FOR FINE TUNINING. ACTUAL VALUES RANGE FROM [-1,1]
steer_left = 0
steer_straight = 1000
steer_right = 2000

#THESE VALUES ARE ALL SCALED FOR FINE TUNINING. ACTUAL VALUES RANGE FROM [-1,1]
throttle_reverse = 0
throttle_neutral = 1100
throttle_forward = 2000


cv2.createTrackbar('Steering_value', 'sliders', steer_straight, steer_right, callback)
cv2.createTrackbar('Throttle_value', 'sliders', throttle_neutral, throttle_forward, callback)


kit.servo[1].angle = 90 # steering servo is on channel 1
kit.continuous_servo[2].throttle = 0.1 # DC motor is on channel 2
cap = cv2.VideoCapture(0)

class TestingCar:
    def __init__(self):
        self.dummy = None

    def test_car(self):
        while(True):
            try:  
                # Capture the camera feed frame by frame
                ret, frame = cap.read()
                steer_input = cv2.getTrackbarPos('Steering_value', 'sliders')
                throttle_input = cv2.getTrackbarPos('Throttle_value', 'sliders')
                
                steering_float = Steering_sensitivity*slider_to_normalized(steer_input)
                throttle_floaT = slider_to_normalized(throttle_input)


                # Display the resulting frame
                cv2.imshow('frame', frame)
            except KeyboardInterrupt:  
                break
        
        # After the loop release the cap object
        cap.release()
        # Destroy all the windows
        cv2.destroyAllWindows()

if __name__ == '__main__':
