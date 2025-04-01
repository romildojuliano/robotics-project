#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan, PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32
import ydlidar
import numpy as np

class YdLidarDriver:
    def __init__(self):
        rospy.init_node('ydlidar_driver')
        self.load_parameters()
        self.init_lidar()
        self.init_publishers()
        
    def load_parameters(self):
        """Load all parameters from ROS parameter server"""
        # Connection parameters
        self.port = rospy.get_param('/port', '/dev/ydlidar')
        self.baudrate = rospy.get_param('/baudrate', 115200)
        
        # Frame and scan parameters
        self.frame_id = rospy.get_param('/frame_id', 'laser_frame')
        self.angle_min = rospy.get_param('/angle_min', -180.0)
        self.angle_max = rospy.get_param('/angle_max', 180.0)
        self.range_min = rospy.get_param('/range_min', 0.1)
        self.range_max = rospy.get_param('/range_max', 16.0)
        self.scan_frequency = rospy.get_param('/frequency', 10.0)
        
        # Debug parameters
        self.publish_pointcloud = rospy.get_param('/publish_pointcloud', False)
        
    def init_lidar(self):
        """Initialize YDLidar connection"""
        self.lidar = ydlidar.YDLidar()
        self.lidar.port = self.port
        self.lidar.baudrate = self.baudrate
        
        # Configure lidar settings
        self.lidar.unit_is_mm = False
        self.lidar.reversion = rospy.get_param('/reversion', True)
        self.lidar.auto_reconnect = rospy.get_param('/auto_reconnect', True)
        self.lidar.intensity = rospy.get_param('/intensity', False)
        
        if not self.lidar.connect():
            rospy.logerr("Failed to connect to LiDAR at %s", self.port)
            rospy.signal_shutdown("LiDAR connection failed")
            return
            
        rospy.loginfo("LiDAR connected successfully")
        self.lidar.start_scan()

    def init_publishers(self):
        """Initialize ROS publishers"""
        self.scan_pub = rospy.Publisher('scan', LaserScan, queue_size=1)
        
        if self.publish_pointcloud:
            self.pc_pub = rospy.Publisher('point_cloud', PointCloud, queue_size=1)

    def process_scan(self):
        """Main processing loop"""
        rate = rospy.Rate(self.scan_frequency)
        
        while not rospy.is_shutdown() and self.lidar.connected:
            try:
                scan = self.lidar.get_scan_data()
                
                if scan:
                    self.publish_laserscan(scan)
                    
                    if self.publish_pointcloud:
                        self.publish_pointcloud(scan)

            except Exception as e:
                rospy.logerr("Scan error: %s", str(e))
                
            rate.sleep()

    def publish_laserscan(self, scan):
        """Convert and publish LaserScan message"""
        scan_msg = LaserScan()
        scan_msg.header.stamp = rospy.Time.now()
        scan_msg.header.frame_id = self.frame_id
        
        scan_msg.angle_min = np.deg2rad(self.angle_min) 
        scan_msg.angle_max = np.deg2rad(self.angle_max)
        scan_msg.angle_increment = np.deg2rad((self.angle_max - self.angle_min) / len(scan))
        scan_msg.scan_time = 1.0 / self.scan_frequency
        scan_msg.time_increment = scan_msg.scan_time / len(scan)
        scan_msg.range_min = self.range_min
        scan_msg.range_max = self.range_max
        
        scan_msg.ranges = [p.range for p in scan]
        scan_msg.intensities = [p.intensity for p in scan]
        
        self.scan_pub.publish(scan_msg)

    def publish_point_cloud(self, scan):
        """Convert and publish PointCloud message (debug)"""
        pc_msg = PointCloud()
        pc_msg.header.stamp = rospy.Time.now()
        pc_msg.header.frame_id = self.frame_id
        
        intensity_channel = ChannelFloat32()
        intensity_channel.name = "intensities"
        
        for point in scan:
            pc_point = Point32()
            pc_point.x = point.range * np.cos(point.angle)
            pc_point.y = point.range * np.sin(point.angle)
            pc_point.z = 0.0
            
            pc_msg.points.append(pc_point)
            intensity_channel.values.append(point.intensity)
            
        pc_msg.channels.append(intensity_channel)
        self.pc_pub.publish(pc_msg)

    def shutdown(self):
        """Cleanup on shutdown"""
        if self.lidar.connected:
            self.lidar.stop_scan()
            self.lidar.disconnect()

if __name__ == '__main__':
    try:
        driver = YdLidarDriver()
        driver.process_scan()
    except rospy.ROSInterruptException:
        pass
    finally:
        driver.shutdown()