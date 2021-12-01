#!/usr/bin/python3
import cv2
from adafruit_servokit import ServoKit


vid = cv2.VideoCapture(0)
cv2.namedWindow('frame')
kit = ServoKit(channels=16)

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

# # THESE VALUES ARE ALL SCALED FOR FINE TUNINING. ACTUAL VALUES RANGE FROM [-1,1]
steer_left = 0
steer_straight = 1000
steer_right = 2000
degree_90 = 90

# #THESE VALUES ARE ALL SCALED FOR FINE TUNINING. ACTUAL VALUES RANGE FROM [-1,1]
throttle_reverse = 0
throttle_neutral = 1100
throttle_forward = 2000

cv2.createTrackbar('Steering_value', 'frame', steer_straight, steer_right, callback)
cv2.createTrackbar('Throttle_value', 'frame', throttle_neutral, throttle_forward, callback)

while(True):
      
    # Capture the video frame by frame
    ret, frame = vid.read()

    # Get slider values
    steer_input = cv2.getTrackbarPos('Steering_value', 'frame')
    throttle_input = cv2.getTrackbarPos('Throttle_value', 'frame')

    # Re-map input values
    steering_float = slider_to_normalized(steer_input)
    throttle_float = slider_to_normalized(throttle_input)

    # Publish values to actuators
    angle_delta = steering_float * degree_90  # difference in degrees from the center 90 degrees
    kit.servo[1].angle = degree_90 + angle_delta # steering servo is on channel 1 (0:left, 90:straight, 180:right)
    kit.continuous_servo[2].throttle = throttle_float # DC motor is on channel 2

    print(f"\r[steering, throttle]: [{round(steering_float, 2)}, {round(throttle_float, 2)}]", end=" ")
    
    # Display the resulting frame
    cv2.imshow('frame', frame)
      
    # the 'q' button is set as the quitting button you may use any desired button of your choice
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
  
# After the loop release the cap object
vid.release()
# Destroy all the windows
cv2.destroyAllWindows()
