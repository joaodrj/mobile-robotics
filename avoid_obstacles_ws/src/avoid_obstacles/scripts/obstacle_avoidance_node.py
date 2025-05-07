#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidanceNode:
    def __init__(self):
        rospy.init_node('obstacle_avoidance_node')

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        self.safe_distance = 0.8
        self.twist = Twist()

        rospy.loginfo("ObstacleAvoidanceNode initialized")

    def scan_callback(self, scan_msg):
        ranges = scan_msg.ranges
        front = ranges[len(ranges) // 2]

        if front < self.safe_distance:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.5  # Turn
        else:
            self.twist.linear.x = 0.3  # Go forward
            self.twist.angular.z = 0.0

        self.cmd_pub.publish(self.twist)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = ObstacleAvoidanceNode()
    node.run()
