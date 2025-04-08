#!/usr/bin/env python3

import time
import numpy as np
import ydlidar
import rospy
from sensor_msgs.msg import LaserScan


if __name__ == '__main__':
    rospy.init_node('lidar')
    ydlidar.os_init()

    lidar_publisher = rospy.Publisher('lidar', LaserScan, queue_size=1)
    lidar_rate = rospy.Rate(rospy.get_param('/scan_frequency'))
    laser = ydlidar.CYdLidar()

    laser.setlidaropt(ydlidar.LidarPropSerialPort, rospy.get_param('/port'))
    laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, rospy.get_param('/baudrate'))
    laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
    laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
    laser.setlidaropt(ydlidar.LidarPropSampleRate, rospy.get_param('/sample_rate'))
    laser.setlidaropt(ydlidar.LidarPropAbnormalCheckCount, rospy.get_param('/abnormal_check_count'))
    laser.setlidaropt(ydlidar.LidarPropFixedResolution, rospy.get_param('/fixed_resolution'))
    laser.setlidaropt(ydlidar.LidarPropAutoReconnect, rospy.get_param('/auto_reconnect'))
    laser.setlidaropt(ydlidar.LidarPropReversion, rospy.get_param('/reversion'))
    laser.setlidaropt(ydlidar.LidarPropInverted, rospy.get_param('/inverted'))
    laser.setlidaropt(ydlidar.LidarPropSingleChannel, rospy.get_param('/single_channel'))
    laser.setlidaropt(ydlidar.LidarPropIntenstiy, rospy.get_param('/intensity'))
    laser.setlidaropt(ydlidar.LidarPropSupportMotorDtrCtrl, rospy.get_param('/support_motor_dtr_ctrl'))
    laser.setlidaropt(ydlidar.LidarPropMinAngle, rospy.get_param('/min_angle'))
    laser.setlidaropt(ydlidar.LidarPropMaxAngle, rospy.get_param('/max_angle'))
    laser.setlidaropt(ydlidar.LidarPropMinRange, rospy.get_param('/min_range'))
    laser.setlidaropt(ydlidar.LidarPropMaxRange, rospy.get_param('/max_range'))
    laser.setlidaropt(ydlidar.LidarPropScanFrequency, rospy.get_param('/scan_frequency'))
    angle_offset = rospy.get_param('/angle_offset')

    ret = laser.initialize()
    ret = laser.turnOn()

    try:
        while not rospy.is_shutdown():
            scan = ydlidar.LaserScan()

            if ret and ydlidar.os_isOk():
                r = laser.doProcessSimple(scan)

                if r:
                    msg = LaserScan()
                    msg.header.stamp = rospy.Time.now()
                    msg.angle_min = scan.config.min_angle
                    msg.angle_max = scan.config.max_angle
                    msg.angle_increment = scan.config.angle_increment
                    msg.time_increment = scan.config.time_increment
                    msg.scan_time = scan.config.scan_time
                    msg.range_min = scan.config.min_range
                    msg.range_max = scan.config.max_range
                    msg.ranges = [point.range for point in scan.points]
                    msg.intensities = [point.intensity for point in scan.points]
                    lidar_publisher.publish(msg)

            lidar_rate.sleep()
    finally:
        laser.turnOff()
        laser.disconnecting()
