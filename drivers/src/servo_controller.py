#! /usr/bin/python3

import rospy
import RPi.GPIO as GPIO
import time
import numpy as np
from drivers.srv import ServoController as ServoControllerService  # Assuming this is your service class

# Variáveis globais
pwm = None
neutral_position_duty_cycle = None
pulse_width_variation_per_degree = None
pwm_frequency = None

def servo_controller_callback(req):
    global pwm, neutral_position_duty_cycle, pulse_width_variation_per_degree, pwm_frequency
    
    trajectory = req.trajectory
    print(f'Trajectory: {trajectory}')
    
    # Extrai ângulo da requisição
    angle = trajectory.points[0].positions[0]
    
    # Calcula a duração do pulso com base no ângulo
    pulse_duration = 180.0 * (pulse_width_variation_per_degree * angle) / np.pi
    duty_cycle = neutral_position_duty_cycle + pulse_duration * pwm_frequency * 1e-4
    
    # Altera o ciclo de trabalho para controlar o servo
    pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(5)

def control_servo():
    global pwm, neutral_position_duty_cycle, pulse_width_variation_per_degree, pwm_frequency
    
    pwm_pin = rospy.get_param('/servo_pwm')
    pwm_frequency = rospy.get_param('/servo_pwm_frequency')
    neutral_position_duty_cycle = rospy.get_param('/neutral_position_duty_cycle')
    pulse_width_variation_per_degree = rospy.get_param('/pulse_width_variation_per_degree')
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(pwm_pin, GPIO.OUT)
    pwm = GPIO.PWM(pwm_pin, pwm_frequency)
    
    # Inicia o PWM com um ciclo de trabalho inicial de 0
    pwm.start(0)
    
    # Configura o servidor do serviço
    rospy.Service('servo_controller', ServoControllerService, servo_controller_callback)
    
    print('Ready to control servo.')
    
    try:
        rospy.spin()
    finally:
        pwm.stop()
        GPIO.cleanup()

if __name__ == '__main__':
    rospy.init_node('servo_controller')
    control_servo()
