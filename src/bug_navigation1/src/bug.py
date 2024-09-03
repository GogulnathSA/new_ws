#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math

class SimpleBug2:
    def __init__(self):
        rospy.init_node('simple_bug2_algorithm')

        # Publishers and subscribers
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Robot state
        self.goal = Point(2.0, 2.0, 0)  # Set a fixed goal point
        self.position = Point()
        self.yaw = 0
        self.regions = {'right': 0, 'front': 0, 'left': 0}

        # Control parameters
        self.state = 'move_to_goal'  # Can be 'move_to_goal' or 'follow_wall'
        self.dist_threshold = 0.5
        self.angle_threshold = math.pi / 18
        self.laser_threshold = 0.3

        # Main loop
        self.rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.position:
                self.bug2_logic()
            self.rate.sleep()

    def scan_callback(self, data):
        self.regions = {
            'right': min(min(data.ranges[0:240]), 10),
            'front': min(min(data.ranges[240:480]), 10),
            'left': min(min(data.ranges[480:720]), 10),
        }

    def odom_callback(self, data):
        self.position = data.pose.pose.position
        orientation = data.pose.pose.orientation
        _, _, self.yaw = self.quaternion_to_euler(orientation.x, orientation.y, orientation.z, orientation.w)

    def quaternion_to_euler(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

    def bug2_logic(self):
        goal_angle = math.atan2(self.goal.y - self.position.y, self.goal.x - self.position.x)
        angle_diff = goal_angle - self.yaw
        distance_to_goal = math.sqrt((self.goal.x - self.position.x)**2 + (self.goal.y - self.position.y)**2)

        twist_msg = Twist()

        if self.state == 'move_to_goal':
            if distance_to_goal < self.dist_threshold:
                rospy.loginfo("Goal Reached!")
                twist_msg.linear.x = 0
                twist_msg.angular.z = 0
                self.cmd_pub.publish(twist_msg)
                return

            if abs(angle_diff) > self.angle_threshold:
                twist_msg.angular.z = 0.5 if angle_diff > 0 else -0.5
            else:
                if self.regions['front'] > self.laser_threshold:
                    twist_msg.linear.x = 0.3
                else:
                    self.state = 'follow_wall'

        elif self.state == 'follow_wall':
            if self.regions['front'] < self.laser_threshold:
                twist_msg.angular.z = 0.5
            elif self.regions['right'] > self.laser_threshold:
                twist_msg.linear.x = 0.3
                twist_msg.angular.z = -0.5
            else:
                twist_msg.linear.x = 0.3

            if abs(angle_diff) < self.angle_threshold and self.regions['front'] > self.laser_threshold:
                self.state = 'move_to_goal'

        self.cmd_pub.publish(twist_msg)

if __name__ == '__main__':
    try:
        SimpleBug2()
    except rospy.ROSInterruptException:
        pass

