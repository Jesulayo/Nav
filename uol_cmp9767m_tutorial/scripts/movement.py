#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped, Point, Twist,PoseStamped
from nav_msgs.msg import Odometry, Path
import math

class MoveRobot:
    def __init__(self):
        self.free_space_center_point = rospy.Subscriber("/free_space_center_point", PointStamped, self.determine_robot_movement)
        self.robot_odom = rospy.Subscriber("/thorvald_001/odometry/gazebo", Odometry, self.robot_odometry)
        self.robot_path = rospy.Publisher('/robot_path', Path, queue_size =1)
        self.command_velocity = rospy.Publisher("/thorvald_001/teleop_joy/cmd_vel", Twist, queue_size=1)
        self.rate = rospy.Rate(10)

        self.path_taken_by_robot = Path()

    def determine_robot_movement(self, data):
        angle_to_goal = math.atan2(data.point.y, data.point.x)

        move_robot = Twist()
        if (angle_to_goal > -0.05 and data.point.y > -0.601):
            move_robot.angular.z = 0.12
            move_robot.linear.x = 0.5
            print('robot is turning left')
        elif (angle_to_goal < -0.05  and data.point.y < -0.3):
            move_robot.angular.z = -0.12
            move_robot.linear.x = 0.5
            print('robot is turning right')
        else :
            move_robot.angular.z = 0.0
            move_robot.linear.x = 0.5
            print('move forward')

        # publish a command to move the robot in the desired direction
        self.command_velocity.publish(move_robot)

    # this is not to move the robot. It is added to draw the path travelled by the robot.
    def robot_odometry(self, data):
        poseStamped = PoseStamped()
        poseStamped.pose = data.pose.pose
        poseStamped.header = data.header
        self.path_taken_by_robot.poses.append(poseStamped)
        self.path_taken_by_robot.header = data.header

        self.robot_path.publish(self.path_taken_by_robot)

if __name__ == '__main__':
    rospy.init_node('move_robot', anonymous=True)
    move_robot = MoveRobot()
    rospy.spin()