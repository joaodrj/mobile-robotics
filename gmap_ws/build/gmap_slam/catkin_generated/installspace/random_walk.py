#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import random

OBSTACLE_THRESHOLD = 0.5  
front_distance = float('inf')

def scan_callback(msg):
    global front_distance

    total_ranges = len(msg.ranges)
    
    start_index = int(total_ranges * 0.35)
    end_index = int(total_ranges * 0.65)

    window = msg.ranges[start_index:end_index]

    valid_ranges = [r for r in window if not math.isinf(r) and not math.isnan(r)]

    if valid_ranges:
        front_distance = min(valid_ranges)
    else:
        front_distance = float('inf')

    rospy.loginfo(f"[LIDAR] Janela {start_index}-{end_index} → menor distância: {front_distance:.2f} m")

def main():
    global front_distance

    rospy.init_node('random_walk_with_avoidance')
    rospy.sleep(1.0)

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/scan', LaserScan, scan_callback)

    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        twist = Twist()

        if front_distance > OBSTACLE_THRESHOLD:
            twist.linear.x = random.uniform(0.1, 0.2)
            twist.angular.z = random.uniform(-0.5, 0.5)
            rospy.loginfo("🟢 Caminho livre → andando")
        else:
            twist.linear.x = 0.0
            twist.angular.z = random.choice([-1.5, 1.5])
            rospy.loginfo("🔴 Obstáculo detectado → girando")

        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

