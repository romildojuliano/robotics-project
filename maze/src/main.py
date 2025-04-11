#!/usr/bin/env python3

import rospy
import numpy as np
from enum import Enum
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from drivers.srv import Camera, ServoController
import time

class State(Enum):
    NAVIGATE = 1
    STOPPED = 2
    SCAN_COLOR = 3
    MOVE_DIRECTION = 4

class MazeRobot:
    def __init__(self):
        # Initialize node
        rospy.init_node('maze_robot')
        
        # State machine
        self.state = State.NAVIGATE
        
        # Subscribe to the VFH node's output
        # We don't need lidar directly since VFH will process it
        self.best_sector = None
        self.num_angular_sectors = rospy.get_param('/num_angular_sectors')
        
        self.cmd_vel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # We'll subscribe to the cmd_vel topic that VFH publishes to
        # and use that to track what VFH is doing
        self.cmd_vel_subscriber = rospy.Subscriber('cmd_vel', Twist, self.main_loop, queue_size=1)
        
        # Service clients
        rospy.wait_for_service('camera')
        rospy.wait_for_service('servo_controller')
        self.camera_service = rospy.ServiceProxy('camera', Camera)
        self.servo_service = rospy.ServiceProxy('servo_controller', ServoController)
        
        # Cache parameters
        self.linear_velocity = rospy.get_param('/linear_velocity')
        self.angular_velocity = rospy.get_param('/angular_velocity')
        
        # Pre-create twist messages
        self.stop_msg = Twist()
        
        self.rate = rospy.Rate(10)
        
        # Initialize servo position to 0
        self.set_servo_position(0.0)
        
        rospy.loginfo("Maze robot initialized")

    def set_servo_position(self, position):
        """Set servo position and wait for it to stabilize"""
        trajectory = JointTrajectory()
        point = JointTrajectoryPoint()
        point.positions = [position]
        trajectory.points = [point]
        
        try:
            self.servo_service(trajectory)
            self.last_servo_position = position  # Store the last position
            rospy.loginfo(f"Servo rotated to {position} radians")
        except rospy.ServiceException as e:
            rospy.logerr(f"Servo service call failed: {e}")

    def main_loop(self, msg):
        print(msg)
        print(self.state)
        if self.state == State.NAVIGATE:
            # Let VFH handle navigation, we just monitor
                # Stop the robot
                if msg == self.stop_msg:
                    print('stopped')
                    self.state = State.STOPPED
                # Otherwise, VFH is already handling navigation through its own cmd_vel publishing
        elif self.state == State.STOPPED:
            # Rotate servo to pi/2
            self.set_servo_position(np.pi/2)
            self.state = State.SCAN_COLOR
        
        elif self.state == State.SCAN_COLOR:
            # Check color with camera
            try:
                resp = self.camera_service()
                if resp.result == "verde":
                    rospy.loginfo("Green detected, moving in that direction")
                    self.state = State.MOVE_DIRECTION
                elif resp.result == "vermelho":
                    rospy.loginfo("Red detected, looking to other side")
                    # Rotate servo to -pi/2
                    self.set_servo_position(-np.pi/2)
                    time.sleep(1)
                    # Check other side
                    resp = self.camera_service()
                    if resp.result == "verde":
                        rospy.loginfo("Green detected on other side")
                        self.state = State.MOVE_DIRECTION
                        
                    else:
                        # Reset servo to forward position
                        self.set_servo_position(0.0)
                        # Reset to navigation if no green found
                        self.state = State.NAVIGATE
                else:
                    # Reset servo to forward position
                    self.set_servo_position(0.0)
                    # No color detected, go back to navigation
                    rospy.loginfo("No color detected, returning to navigation")
                    self.state = State.NAVIGATE
            except rospy.ServiceException as e:
                rospy.logerr(f"Camera service call failed: {e}")
                # Reset servo to forward position
                self.set_servo_position(0.0)
                self.state = State.NAVIGATE
        
        elif self.state == State.MOVE_DIRECTION:
            # Store the current servo position to know which direction to turn
            # Store the current servo position
            servo_position = np.pi/2  # Default to right
            
            # Check which direction we scanned last
            if hasattr(self, 'last_servo_position'):
                servo_position = self.last_servo_position
            
            # First turn in the direction the camera is pointing
            turn_msg = Twist()
            
            if servo_position > 0:  # Camera pointing right, turn right
                rospy.loginfo("Turning right toward green marker")
                turn_msg.angular.z = -self.angular_velocity  # Negative for right turn
            else:  # Camera pointing left, turn left
                rospy.loginfo("Turning left toward green marker")
                turn_msg.angular.z = self.angular_velocity   # Positive for left turn
            
            # Turn for a fixed duration - enough to make approximately a 90-degree turn
            self.cmd_vel_pub.publish(turn_msg)
            
            # Then move forward
            move_msg = Twist()
            move_msg.linear.x = self.linear_velocity
            self.cmd_vel_pub.publish(move_msg)
            rospy.loginfo("Moving forward")
            
            # Stop the robot
            self.cmd_vel_pub.publish(self.stop_msg)
            
            rospy.loginfo("Goal reached - shutting down")
            rospy.signal_shutdown("Goal reached")
    

if __name__ == '__main__':
    robot = MazeRobot()
    
    while not rospy.is_shutdown():
        try:
            rospy.spin()
        except:
            break