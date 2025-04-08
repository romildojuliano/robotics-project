#!/usr/bin/env python3

import time
import numpy as np
import RPi.GPIO as GPIO
import rospy
from drivers.srv import ServoController, ServoControllerResponse

pwm = None
neutral_position_duty_cycle = None
pulse_width_variation_per_degree = None
pwm_frequency = None


def servo_controller_callback(req):
    global pwm, neutral_position_duty_cycle, pulse_width_variation_per_degree, pwm_frequency

    trajectory = req.trajectory
    angle = trajectory.points[0].positions[0]

    pulse_duration = 180.0 * (pulse_width_variation_per_degree * angle) / np.pi
    duty_cycle = neutral_position_duty_cycle + pulse_duration * pwm_frequency * 1e-4

    pwm.ChangeDutyCycle(duty_cycle)
    return ServoControllerResponse()


if __name__ == '__main__':
    pwm_pin = rospy.get_param('/servo_pwm')
    pwm_frequency = rospy.get_param('/servo_pwm_frequency')
    neutral_position_duty_cycle = rospy.get_param('/neutral_position_duty_cycle')
    pulse_width_variation_per_degree = rospy.get_param('/pulse_width_variation_per_degree')

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(pwm_pin, GPIO.OUT)
    pwm = GPIO.PWM(pwm_pin, pwm_frequency)
    pwm.start(0)

    rospy.init_node('servo_controller')
    rospy.Service('servo_controller', ServoController, servo_controller_callback)

    try:
        rospy.spin()
    finally:
        GPIO.cleanup()