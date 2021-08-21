#!/usr/bin/python3
import rospy
from std_msgs.msg import Float32
from adafruit_servokit import ServoKit

THROTTLE_NODE_NAME = 'throttle_client'
THROTTLE_TOPIC_NAME = '/throttle'
kit = ServoKit(channels=16)
'''
    more documentation at https://learn.adafruit.com/16-channel-pwm-servo-driver/python-circuitpython
    throttle servo is on channel 2
'''

def callback(data):
    # output_start= rospy.get_param("/Throttle_max_reverse")
    # output_end = rospy.get_param("/Throttle_max_forward")
    # Throttle_neutral = rospy.get_param("/Throttle_neutral")
    #
    # input_start = -1
    # input_end = 1
    #
    # input_throttle = data.data
    # normalized_throttle = output_start + (input_throttle - input_start) * ((output_end - output_start) / (input_end - input_start))
    normalized_throttle = data.data
    kit.continuous_servo[2].throttle = normalized_throttle


def listener():
    rospy.init_node(THROTTLE_NODE_NAME, disable_signals=True, anonymous=False)
    rospy.Subscriber(THROTTLE_TOPIC_NAME, Float32, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
