#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
import math

class PillarNavigator:
    def __init__(self):
        rospy.loginfo("PillarNavigator started.")
        # escuta o LaserScan direto do rslidar
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.cmd_pub  = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.marker_pub = rospy.Publisher("/pillar_marker", Marker, queue_size=1)
        self.k_lin = rospy.get_param("~linear_gain", 0.5)
        self.k_ang = rospy.get_param("~angular_gain", 2.0)

    def scan_callback(self, data):
        ranges = [r for r in data.ranges if not math.isinf(r)]
        if not ranges:
            return
        rmin = min(ranges)
        idx  = data.ranges.index(rmin)
        ang  = data.angle_min + idx * data.angle_increment
        x, y = rmin * math.cos(ang), rmin * math.sin(ang)
        # marker
        m = Marker()
        m.header.frame_id = data.header.frame_id
        m.header.stamp = rospy.Time.now()
        m.ns, m.id = "pillar", 0
        m.type, m.action = Marker.SPHERE, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, 0
        m.scale.x = m.scale.y = m.scale.z = 0.2
        m.color.a, m.color.r = 1.0, 1.0
        self.marker_pub.publish(m)
        # comando de movimento
        cmd = Twist()
        cmd.linear.x  = self.k_lin  * rmin
        cmd.angular.z = -self.k_ang * ang
        self.cmd_pub.publish(cmd)

if __name__=="__main__":
    rospy.init_node("pillar_navigator")
    PillarNavigator()
    rospy.spin()

