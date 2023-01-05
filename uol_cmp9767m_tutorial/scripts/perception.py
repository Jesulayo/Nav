#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, Point, PointStamped
import math
import tf
from rospy import Time

class ScanEnvironment:
    def __init__(self):
        self.subscriber = rospy.Subscriber("/thorvald_001/front_scan", LaserScan, self.scan_for_obstacle)
        self.free_space_center_point = rospy.Publisher("/free_space_center_point", PointStamped, queue_size=1)
        self.filtered_scan = rospy.Publisher("/thorvald_001/filtered_front_scan", LaserScan, queue_size=10)
        self.rate = rospy.Rate(10)

    def scan_for_obstacle(self, data):
        print('checking for obstacle')

        roi_range = []
        filtered_range = []
        left_axis = []
        right_axis = []

        new_scan = LaserScan()
        new_scan.header.frame_id = data.header.frame_id
        new_scan.header.stamp.secs = data.header.stamp.secs
        new_scan.angle_increment = data.angle_increment
        new_scan.time_increment = data.time_increment
        new_scan.range_max = data.range_max
        new_scan.range_min = data.range_min
        new_scan.angle_min = data.angle_min
        new_scan.angle_max = data.angle_max

        # get ranges with 3m of the robot in the x direction
        #convert all ranges into cartesian coordinate (x,y)
        for i, j in enumerate(data.ranges):
            z = data.angle_min + i*data.angle_increment
            x = j*math.cos(z)
            y = j*math.sin(z)

            #if the x axis is within 3m, append the range into roi_range array else append 0
            if( x >= 0 and x < 3):
                roi_range.append(j)
            else:
                roi_range.append(0)

        # get ranges with 3m of the robot in the y direction
        for i, j in enumerate(roi_range):
            z = data.angle_min + i*data.angle_increment
            x = j*math.cos(z)
            y = j*math.sin(z)

            if( y >= -3 and y < 3 ):
                filtered_range.append(j)
                if( y >= -3 and y < 0 ):
                    right_axis.append(y)
                if( y >= 0 and y < 3 ):
                    left_axis.append(y)
            else:
                filtered_range.append(0)
       
        new_scan.ranges = filtered_range

        right_axis = [a for a in right_axis if a != 0]
        right_point = max(right_axis)
        print('right point: ' + str(right_point))

        left_axis = [a for a in left_axis if a != 0]
        left_point = min(left_axis)
        print('left point: ' + str(left_point))

        free_space = (left_point + right_point)/2

        point = Point()
        point.x = 3
        point.y = free_space

        tf_listener = tf.TransformListener()
        tf_listener.waitForTransform("thorvald_001/hokuyo_front", "thorvald_001/base_link", rospy.Time(), rospy.Duration(3.0))

        free_space_point = PointStamped()
        free_space_point.header.frame_id = "thorvald_001/hokuyo_front"
        free_space_point.header.stamp = rospy.Time(0)
        free_space_point.point = point

        try:
            freespace = tf_listener.transformPoint("thorvald_001/base_link", free_space_point)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print('')

        self.free_space_center_point.publish(freespace)
        self.filtered_scan.publish(new_scan)

if __name__ == '__main__':
    rospy.init_node('scan_robot_environment', anonymous=True)
    scan_environment = ScanEnvironment()
    rospy.spin()