#!/usr/bin/python3
import rospy
from std_msgs.msg import Float32, Float32MultiArray, Bool
from sensor_msgs.msg import LaserScan
import math
import time


OBSTACLE_DETECTION_NODE_NAME = 'simple_obstacle_detection_node'
SUBSCRIBER_TOPIC_NAME = '/LiDAR/LD06'
OBSTACLE_DETECTED_TOPIC_NAME = '/obstacle_detection'

class ObstacleDetection:
    def __init__(self):

        self.init_node = rospy.init_node(OBSTACLE_DETECTION_NODE_NAME, anonymous=False)
        self.sub = rospy.Subscriber(SUBSCRIBER_TOPIC_NAME, LaserScan, self.detect_obstacle)
        self.obstacle_pub = rospy.Publisher(OBSTACLE_DETECTED_TOPIC_NAME, Float32MultiArray, queue_size=1)
        self.obstacle_info = Float32MultiArray()

        # Lidar properties (needs to be updated to be ros parameters loaded from config depending on lidar brand)
        self.viewing_angle = 360

        # Obstacle distance limits (meters) (update/calibrate as needed)
        self.max_distance_tolerance = 1.2
        self.min_distance_tolerance = 0.15

        '''
        For LD06
        values at 0 degrees   ---> (straight)
        values at 90 degrees  ---> (full right)
        values at -90 degrees ---> (full left)
        '''

    def detect_obstacle(self, data):
        # rospy.loginfo("----  START PRINTING DEBUG MESSAGES  -----")
        # rospy.loginfo("angle_min:" + str(data.angle_min))
        # rospy.loginfo("angle_max:" + str(data.angle_max))
        # rospy.loginfo("angle_increment:" + str(data.angle_increment))
        # rospy.loginfo("time_increment:" + str(data.time_increment))
        # rospy.loginfo("scan_time:" + str(data.scan_time))
        # rospy.loginfo("range_min:" + str(data.range_min))
        # rospy.loginfo("range_max:" + str(data.range_max))
        # rospy.loginfo("ranges:" + str(data.ranges))
        # rospy.loginfo("intesnsities:" + str(data.intensities))
        # rospy.loginfo("----  END PRINTING DEBUG MESSAGES  -----")
        total_number_of_scans = len(data.ranges)
        scans_per_degree = float(total_number_of_scans/self.viewing_angle)

        #angle_values = [0, 11.5, 22.5, 33.5, 45, 56.5, 67.5, 348.5, 337.5, 326.5, 315, 303.5, 292.5, 270, 225, 180, 135]
        #65 - -65 degrees
        angle_values = [1,5,10,15,20,25,30,35,40,45,50,55,60,65,355,350,345,340,335,330,325,320,315,310,305,300,295]
        range_values = []
        for angle in angle_values:
            bs = data.ranges[round(angle*scans_per_degree)]
            if self.max_distance_tolerance >= bs >= self.min_distance_tolerance:
                range_values.append(data.ranges[round(angle*scans_per_degree)])
            else:
                range_values.append(float(self.max_distance_tolerance)+1)
        #print("Range values: " + str(range_values))
        min_distance = min(range_values)
        min_angle_index = range_values.index(min(range_values))
        min_angle = angle_values[min_angle_index]
        print('Min Distance %s at %s degrees.' %(str(min_distance),str(min_angle)))
        obstacle_info = []
        #rospy.loginfo("Obstacle Detected:" + str(min_distance))
        if self.max_distance_tolerance >= abs(min_distance) >= self.min_distance_tolerance:
            #make the angle nuegative if its on the right side
            if min_angle > 180: min_angle = min_angle - 360
            angle_rad = (min_angle * math.pi) / 180
            normalized_angle = math.sin(angle_rad)
            obstacle_detected = 1.0

            # Publish ROS message
            obstacle_info.append(min_distance)
            obstacle_info.append(min_angle)
            obstacle_info.append(obstacle_detected)
            self.obstacle_pub.publish(Float32MultiArray(data=obstacle_info))
            rospy.loginfo("Obstacle Detected:" + str(obstacle_info))

        else:
            # nonsense values
            min_distance = -1.0
            normalized_angle = -1.0
            obstacle_detected = 0.0

            # Publish ROS message
            obstacle_info.append(min_distance)
            obstacle_info.append(normalized_angle)
            obstacle_info.append(obstacle_detected)
            self.obstacle_pub.publish(Float32MultiArray(data=obstacle_info))


def main():
    obstacle_detection = ObstacleDetection()
    rate = rospy.Rate(15)
    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()


if __name__ == '__main__':
    main()
