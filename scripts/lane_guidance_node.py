#!/usr/bin/python3
import rospy
from std_msgs.msg import Float32, Int32, Int32MultiArray, Float32MultiArray

LANE_GUIDANCE_NODE_NAME = 'lane_guidance_node'
STEERING_TOPIC_NAME = '/steering'
THROTTLE_TOPIC_NAME = '/throttle'
CENTROID_TOPIC_NAME = '/centroid'
OBS_DETECTION_TOPIC_NAME = '/obstacle_detection'

class PathPlanner:
    def __init__(self):
        # Initialize node and create publishers/subscribers
        self.init_node = rospy.init_node(LANE_GUIDANCE_NODE_NAME, anonymous=False)
        self.steering_publisher = rospy.Publisher(STEERING_TOPIC_NAME, Float32, queue_size=1)
        self.throttle_publisher = rospy.Publisher(THROTTLE_TOPIC_NAME, Float32, queue_size=1)
        self.steering_float = Float32()
        self.throttle_float = Float32()
        self.centroid_subscriber = rospy.Subscriber(CENTROID_TOPIC_NAME, Float32, self.controller)
        self.obstacle_detection_subscriber = rospy.Subscriber(OBS_DETECTION_TOPIC_NAME, Float32MultiArray, self.det_handler)

        self.stored_det = [-1.0, -1.0, 0.0]
        self.last_detected = rospy.get_time()

        # Getting ROS parameters set from calibration Node
        self.steering_sensitivity = rospy.get_param('steering_sensitivity')
        self.no_error_throttle = rospy.get_param('no_error_throttle')
        self.error_throttle = rospy.get_param('error_throttle')
        self.error_threshold = rospy.get_param('error_threshold')
        self.zero_throttle = rospy.get_param('zero_throttle')
        # # Display Parameters
        rospy.loginfo(
            f'\nsteering_sensitivity: {self.steering_sensitivity}'
            f'\nno_error_throttle: {self.no_error_throttle}'
            f'\nerror_throttle: {self.error_throttle}'
            f'\nerror_threshold: {self.error_threshold}')

    def det_handler(self, data):
        self.stored_det = [data.data[0], data.data[1], data.data[2]]
        self.last_detected = rospy.get_time()
        print("Ran the det_handler")

    def controller(self, data):
        # try:
        kp = self.steering_sensitivity
        error_x = data.data
        rospy.loginfo(f"{error_x}")
        if error_x <= self.error_threshold:
            throttle_float = self.no_error_throttle
        else:
            throttle_float = self.error_throttle
        decay = 0.8**(rospy.get_time() - self.last_detected)
        obstacle_error = self.stored_det[2] * (self.stored_det[1]/abs(self.stored_det[1]) * ( (math.pi/2)-abs(self.stored_det[1]) )/(math.pi/2)
        # rospy.loginfo(f'\nNOOOOO Obstacle error: {obstacle_error}')
        print(f'Obstacle error: {obstacle_error}')
        print("From obs det:" + str(self.stored_det))
        #print("Obstacle error:" + str(obstacle_error))
        steering_float = -float(kp * error_x) + float(kp * obstacle_error)
        if steering_float < -1.0:
            steering_float = -1.0
        elif steering_float > 1.0:
            steering_float = 1.0
        else:
            pass
        self.steering_float.data = steering_float
        self.throttle_float.data = throttle_float
        self.steering_publisher.publish(self.steering_float)
        self.throttle_publisher.publish(self.throttle_float)
        rospy.on_shutdown(self.clean_shutdown)

    def clean_shutdown(self):
        print("Shutting Down...")
        self.throttle_float.data = self.zero_throttle
        self.throttle_publisher.publish(self.throttle_float)
        print("Shut down complete")


def main():
    path_planner = PathPlanner()
    rate = rospy.Rate(15)
    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()


if __name__ == '__main__':
    main()
