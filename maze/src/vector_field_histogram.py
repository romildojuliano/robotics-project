#!/usr/bin/env python3

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import sys

class VectorFieldHistogram:
    def __init__(self):
        self.cmd_vel_publisher = rospy.Publisher('cmd_vel',
                                                 Twist,
                                                 queue_size=1)

        self.lidar_subscriber = rospy.Subscriber('lidar',
                                                 LaserScan,
                                                 self.calculate_steering_direction,
                                                 queue_size=1)

        self.stop_flag = False
        self.num_angular_sectors = rospy.get_param('/num_angular_sectors')
        self.angle_offset = rospy.get_param('/angle_offset')

        self.lower_range_threshold = rospy.get_param('/lower_range_threshold')
        self.upper_range_threshold = rospy.get_param('/upper_range_threshold')

        self.linear_velocity = rospy.get_param('/linear_velocity')
        self.angular_velocity = rospy.get_param('/angular_velocity')

    def calculate_steering_direction(self, msg):
        histogram = np.zeros(self.num_angular_sectors)
        angle_min = msg.angle_min + self.angle_offset
        angle_max = msg.angle_max + self.angle_offset

        if angle_min > angle_max:
            angle_min, angle_max = angle_max, angle_min

        angle = angle_min
        sector_angle = np.pi / self.num_angular_sectors

        for range in msg.ranges:
            if self.lower_range_threshold < range < self.upper_range_threshold:
                sector = int(np.floor((angle + np.pi / 2) / sector_angle))
                if 0 <= sector < self.num_angular_sectors:
                    histogram[sector] += 1
            angle += msg.angle_increment

        # print(histogram)
        histogram = histogram >= 5
        # print(histogram)
        best_sector = None
        min_distance = np.inf

        subset_sectors = []

        for i, sector_points in enumerate(histogram):
            if sector_points == 0:
                subset_sectors.append(i)
                distance = abs(i - self.num_angular_sectors // 2)
                if distance < min_distance:
                    best_sector = i
                    min_distance = distance

        msg = Twist()
        
        if not self.stop_flag and len(subset_sectors) > 1:
            best_sector = subset_sectors[len(subset_sectors) // 2]
            # print(f"Best sector: {best_sector}")

            msg.linear.x = self.linear_velocity
            center_sector = self.num_angular_sectors // 2

            if best_sector == center_sector:
                msg.angular.z = 0.0
            elif best_sector < center_sector:
                msg.linear.x = self.linear_velocity * 0.5
                msg.angular.z = -self.angular_velocity
            else:
                msg.linear.x = self.linear_velocity * 0.5
                msg.angular.z = self.angular_velocity
        else:
            self.stop_flag = True
        self.cmd_vel_publisher.publish(msg)


if __name__ == '__main__':
    rospy.init_node('vector_field_histogram')

    vector_field_histogram = VectorFieldHistogram()
    vector_field_histogram_rate = rospy.Rate(rospy.get_param('/update_frequency'))

    try:
        while not rospy.is_shutdown():
            vector_field_histogram_rate.sleep()
    finally:
        vector_field_histogram.cmd_vel_publisher.publish(Twist())
