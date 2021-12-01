#!/usr/bin/python3
import rospy
from std_msgs.msg import Float32, Int32, Int32MultiArray, Bool

OBSTACLE_DETECTION_NODE_NAME = 'obstacle_detection_node'
OBSTACLE_DETECTED_TOPIC_NAME = '/obstacle_detection'
FINISHED_NAV_TOPIC_NAME = '/finishednav'


class ObstacleDetection:
    def __init__(self):
        # Initialize node and create publishers/subscribers
        self.init_node = rospy.init_node(OBSTACLE_DETECTION_NODE_NAME, anonymous=False)
        self.obsdetected = rospy.Publisher(OBSTACLE_DETECTED_TOPIC_NAME, Bool, queue_size=1)
        self.detected = Bool()
        self.can_update = True
        self.lidar_subscriber = rospy.Subscriber(LIDAR_TOPIC_NAME, Float32, self.detector) #TODO: Set up lidar node
        self.finished_nav_subscriber = rospy.Subscriber(FINISHED_NAV_TOPIC_NAME, Bool, self.set_update) #TODO: create this topic in nav node

    def set_update(self, data):
        # If data is true, set can_update to true
        if(data is True):
            self.can_update = True
        else:
            self.can_update = False
            self.detected.data = False
            self.obsdetected.publish(detected)
        # If not, set can_update to false


    def detector(self, data):
        # If can update, run detection and update 
        if(self.can_update is True):
            self.detected.data = True # change this according to detection




            self.obsdetected.publish(detected)





        rospy.on_shutdown(self.clean_shutdown)

    def clean_shutdown(self):
        print("Shutting Down...")
        print("Shut down complete")


def main():
    obstacle_detection = ObstacleDetection()
    rate = rospy.Rate(15)
    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()


if __name__ == '__main__':
    main()
