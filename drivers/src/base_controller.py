#!/usr/bin/env python3

import numpy as np
import RPi.GPIO as GPIO
import rospy
from geometry_msgs.msg import Twist


class BaseController:
    def __init__(self):
        self.cmd_vel_subscriber = rospy.Subscriber('cmd_vel', Twist, self.update_desired_motor_velocities, queue_size=1)

        self.left_motor_pwm = rospy.get_param('/left_motor_pwm')
        self.right_motor_pwm = rospy.get_param('/right_motor_pwm')

        self.left_motor_direction_a = rospy.get_param('/left_motor_direction_a')
        self.left_motor_direction_b = rospy.get_param('/left_motor_direction_b')
        self.right_motor_direction_a = rospy.get_param('/right_motor_direction_a')
        self.right_motor_direction_b = rospy.get_param('/right_motor_direction_b')

        self.left_encoder_channel_a = rospy.get_param('/left_encoder_channel_a')
        self.left_encoder_channel_b = rospy.get_param('/left_encoder_channel_b')
        self.right_encoder_channel_a = rospy.get_param('/right_encoder_channel_a')
        self.right_encoder_channel_b = rospy.get_param('/right_encoder_channel_b')

        self.pulses_per_revolution = rospy.get_param('/pulses_per_revolution')
        self.gearbox_reduction_ratio = rospy.get_param('/gearbox_reduction_ratio')

        self.track_width = rospy.get_param('/track_width')
        self.wheel_radius = rospy.get_param('/wheel_radius')

        self.kp = rospy.get_param('/kp')
        self.ki = rospy.get_param('/ki')
        self.kd = rospy.get_param('/kd')

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.left_motor_pwm, GPIO.OUT)
        GPIO.setup(self.right_motor_pwm, GPIO.OUT)

        GPIO.setup(self.left_motor_direction_a, GPIO.OUT)
        GPIO.setup(self.left_motor_direction_b, GPIO.OUT)
        GPIO.setup(self.right_motor_direction_a, GPIO.OUT)
        GPIO.setup(self.right_motor_direction_b, GPIO.OUT)

        GPIO.setup(self.left_encoder_channel_a, GPIO.IN)
        GPIO.setup(self.left_encoder_channel_b, GPIO.IN)
        GPIO.setup(self.right_encoder_channel_a, GPIO.IN)
        GPIO.setup(self.right_encoder_channel_b, GPIO.IN)

        self.left_encoder_pulse_count = 0
        self.right_encoder_pulse_count = 0

        GPIO.add_event_detect(self.left_encoder_channel_a, GPIO.RISING, callback=self.left_encoder_callback)
        GPIO.add_event_detect(self.right_encoder_channel_a, GPIO.RISING, callback=self.right_encoder_callback)

        self.left_motor_velocity = 0
        self.right_motor_velocity = 0

        self.desired_left_motor_velocity = 0
        self.desired_right_motor_velocity = 0

    def __del__(self):
        GPIO.output(self.left_motor_pwm, GPIO.LOW)
        GPIO.output(self.right_motor_pwm, GPIO.LOW)

        GPIO.output(self.left_motor_direction_a, GPIO.LOW)
        GPIO.output(self.left_motor_direction_b, GPIO.LOW)
        GPIO.output(self.right_motor_direction_a, GPIO.LOW)
        GPIO.output(self.right_motor_direction_b, GPIO.LOW)

        GPIO.cleanup()

    def left_encoder_callback(self, channel):
        if GPIO.input(self.left_encoder_channel_b) == GPIO.HIGH:
            self.left_encoder_pulse_count += 1
        else:
            self.left_encoder_pulse_count -= 1

    def right_encoder_callback(self, channel):
        if GPIO.input(self.right_encoder_channel_b) == GPIO.HIGH:
            self.right_encoder_pulse_count += 1
        else:
            self.right_encoder_pulse_count -= 1

    def update_motor_velocities(self):
        self.left_motor_velocity = 2 * np.pi * self.left_encoder_pulse_count / (self.pulses_per_revolution * self.gearbox_reduction_ratio)
        self.right_motor_velocity = 2 * np.pi * self.right_encoder_pulse_count / (self.pulses_per_revolution * self.gearbox_reduction_ratio)
        self.left_encoder_pulse_count = 0
        self.right_encoder_pulse_count = 0

    def update_desired_motor_velocities(self, msg):
        self.desired_left_motor_velocity = (msg.linear.x - msg.angular.z * (self.track_width / 2)) / self.wheel_radius
        self.desired_right_motor_velocity = (msg.linear.x + msg.angular.z * (self.track_width / 2)) / self.wheel_radius

    def control_loop(self):
        self.update_motor_velocities()

        # TODO: Implement a PID controller to determine the PWM duty cycle for each motor

        # TODO: Set the PWM duty cycle for each motor


if __name__ == '__main__':
    rospy.init_node('base_controller')

    base_controller = BaseController()
    base_controller_rate = rospy.Rate(rospy.get_param('/control_loop_frequency'))

    while not rospy.is_shutdown():
        base_controller.control_loop()
        base_controller_rate.sleep()
