import RPi.GPIO as GPIO
import time

# Definição dos pinos do Raspberry Pi conectados ao L298N
ENA = 12  # PWM Motor A (pino físico 32)
ENB = 13  # PWM Motor B (pino físico 33)
IN1 = 16   # Sentido do Motor A
IN2 = 26   # Sentido do Motor A
IN3 = 5  # Sentido do Motor B
IN4 = 6  # Sentido do Motor B

# Configuração dos pinos
GPIO.setmode(GPIO.BCM)
GPIO.setup([IN1, IN2, IN3, IN4], GPIO.OUT)
GPIO.setup([ENA, ENB], GPIO.OUT)

# Configuração dos sinais PWM
pwm_a = GPIO.PWM(ENA, 1000)  # Frequência de 1 kHz
pwm_b = GPIO.PWM(ENB, 1000)  # Frequência de 1 kHz
pwm_a.start(0)  # Inicializa com 0% do ciclo de trabalho
pwm_b.start(0)

def motor_frente():
    """ Faz os motores girarem para frente """
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_a.ChangeDutyCycle(70)  # Define 70% do ciclo de trabalho
    pwm_b.ChangeDutyCycle(70)

def motor_reverso():
    """ Faz os motores girarem para trás """
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_a.ChangeDutyCycle(70)
    pwm_b.ChangeDutyCycle(70)

def motor_parado():
    """ Para os motores """
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0)

try:
    print("Motores para frente")
    motor_frente()
    time.sleep(3)

    print("Motores para trás")
    motor_reverso()
    time.sleep(3)

    print("Parando motores")
    motor_parado()
    time.sleep(1)

except KeyboardInterrupt:
    print("Interrompido pelo usuário")

finally:
    pwm_a.stop()
    pwm_b.stop()
    GPIO.cleanup()  # Libera os pinos GPIO
    