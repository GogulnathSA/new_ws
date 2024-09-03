#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class Bug2Algorithm:
    def __init__(self):
        rospy.init_node('bug2_algorithm')

        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.current_goal = None
        self.position = None
        self.yaw = None
        self.heading_to_goal = False
        self.following_obstacle = False
        self.distance_threshold = 0.1  # Threshold distance to consider the goal reached
        self.obstacle_threshold = 0.5  # Threshold distance to consider an obstacle in front
        self.kp_angular = 1.0        # Further reduced proportional gain for angular velocity
        self.kp_linear = 0.2           # Further increased proportional gain for linear velocity
        self.previous_distance_to_goal = float('inf')  # Initialize with a large value

    def goal_callback(self, msg):
        self.current_goal = msg.pose
        self.heading_to_goal = True
        self.following_obstacle = False
        self.previous_distance_to_goal = float('inf')  # Reset when a new goal is received

    def odom_callback(self, msg):
        self.position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        (_, _, self.yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

    def scan_callback(self, msg):
        if self.heading_to_goal:
            self.move_towards_goal(msg)
        elif self.following_obstacle:
            self.follow_obstacle(msg)

    def move_towards_goal(self, scan_data):
        if not self.current_goal or not self.position:
            return

        # Calculate distance to goal
        goal_x = self.current_goal.position.x
        goal_y = self.current_goal.position.y
        distance_to_goal = math.sqrt((goal_x - self.position.x) ** 2 + (goal_y - self.position.y) ** 2)

        if distance_to_goal < self.distance_threshold:
            rospy.loginfo("Goal reached!")
            self.stop_robot()
            self.heading_to_goal = False
            return

        # Update the previous distance to the goal
        self.previous_distance_to_goal = distance_to_goal

        # Calculate angle to goal
        goal_angle = math.atan2(goal_y - self.position.y, goal_x - self.position.x)
        angle_diff = self.normalize_angle(goal_angle - self.yaw)

        if abs(angle_diff) > 0.1:
            self.rotate_to_goal(angle_diff)
        else:
            if min(scan_data.ranges) < self.obstacle_threshold:
                rospy.loginfo("Obstacle detected, switching to obstacle avoidance.")
                self.following_obstacle = True
                self.heading_to_goal = False
            else:
                self.move_forward()

    def rotate_to_goal(self, angle_diff):
        twist_msg = Twist()
        twist_msg.angular.z = self.kp_angular * angle_diff
        # Add a small forward movement to help prevent getting stuck rotating
        twist_msg.linear.x = self.kp_linear * 0.1
        self.cmd_pub.publish(twist_msg)

    def move_forward(self):
        twist_msg = Twist()
        twist_msg.linear.x = self.kp_linear
        twist_msg.angular.z = 0.0  # Ensure no unwanted rotation while moving forward
        self.cmd_pub.publish(twist_msg)

    def follow_obstacle(self, scan_data):
        # Simplified obstacle following logic
        twist_msg = Twist()

        left_scan = min(scan_data.ranges[:len(scan_data.ranges) // 2])
        right_scan = min(scan_data.ranges[len(scan_data.ranges) // 2:])
        front_scan = min(scan_data.ranges[len(scan_data.ranges) // 3:2 * len(scan_data.ranges) // 3])

        if front_scan < self.obstacle_threshold:
            twist_msg.angular.z = self.kp_angular  # Rotate to avoid obstacle
        else:
            twist_msg.linear.x = self.kp_linear * 0.5  # Move forward

        self.cmd_pub.publish(twist_msg)

        # Check if we're back on the line towards the goal
        goal_x = self.current_goal.position.x
        goal_y = self.current_goal.position.y
        goal_angle = math.atan2(goal_y - self.position.y, goal_x - self.position.x)
        angle_diff = self.normalize_angle(goal_angle - self.yaw)

        if abs(angle_diff) < 0.1 and self.is_closer_to_goal(goal_x, goal_y):
            rospy.loginfo("Back on track towards the goal.")
            self.following_obstacle = False
            self.heading_to_goal = True

    def is_closer_to_goal(self, goal_x, goal_y):
        # Calculate the distance to goal to check if the robot is closer than before
        new_distance = math.sqrt((goal_x - self.position.x) ** 2 + (goal_y - self.position.y) ** 2)
        return new_distance < self.previous_distance_to_goal

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    bug2 = Bug2Algorithm()
    bug2.run()

