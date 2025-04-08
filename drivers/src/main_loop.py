#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Int16, Float32
from drivers.srv import ServoController, Camera
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Estado global
ultima_cor_detectada = "nenhuma"
obstaculo_proximo = False
direcao_alvo = None  # Representa a direção vetorial em radianos

# Callback do LiDAR
def lidar_callback(msg):
    global obstaculo_proximo
    # Define limite mínimo (ex: 0.3m) para considerar obstáculo à frente
    ranges = list(msg.ranges)
    # Filtra apenas os pontos à frente (em um cone de ~30 graus)
    forward_ranges = []
    angle_increment = msg.angle_increment
    num_samples = len(ranges)
    
    # Determina os índices que correspondem ao cone frontal
    mid_idx = num_samples // 2
    angle_span = int(math.radians(15) / angle_increment)  # 15 graus para cada lado
    
    for i in range(mid_idx - angle_span, mid_idx + angle_span):
        if 0 <= i < num_samples and ranges[i] > 0.01:  # Ignora leituras zero/inválidas
            forward_ranges.append(ranges[i])
    
    if len(forward_ranges) > 0:
        obstaculo_proximo = min(forward_ranges) < 0.3
    else:
        obstaculo_proximo = False
    
    rospy.logdebug(f"Obstáculo próximo: {obstaculo_proximo}")

# Callback da direção vetorial
def callback_direcao(msg):
    global direcao_alvo
    direcao_alvo = msg.data
    rospy.logdebug(f"Nova direção recebida: {direcao_alvo}")

# Função para mover o servo para um ângulo específico
def mover_servo(angulo):
    try:
        rospy.wait_for_service('servo_controller', timeout=1.0)
        servo_service = rospy.ServiceProxy('servo_controller', ServoController)
        
        # Cria a mensagem de trajetória esperada pelo serviço
        trajectory = JointTrajectory()
        point = JointTrajectoryPoint()
        
        # Converte o ângulo de graus para radianos, pois o servo espera radianos
        point.positions = [math.radians(angulo)]
        trajectory.points = [point]
        
        # Chama o serviço
        servo_service(trajectory)
        rospy.loginfo(f"Servo movido para {angulo} graus")
        return True
    except rospy.ServiceException as e:
        rospy.logerr(f"Falha ao chamar o serviço servo_controller: {e}")
        return False
    except rospy.ROSException as e:
        rospy.logerr(f"Timeout ao esperar pelo serviço servo_controller: {e}")
        return False

# Função de decisão com base nas cores detectadas
def detectar_cor_em_direcao(angulo):
    global ultima_cor_detectada
    try:
        # Move o servo para o ângulo desejado
        if not mover_servo(angulo):
            return "nenhuma"
            
        rospy.sleep(1.5)  # Espera o servo posicionar e a imagem estabilizar
        
        # Chama o serviço da câmera para análise de cor
        rospy.wait_for_service('camera', timeout=2.0)
        camera = rospy.ServiceProxy('camera', Camera)
        resposta = camera()
        cor = resposta.result
        
        rospy.loginfo(f"Cor detectada a {angulo} graus: {cor}")
        ultima_cor_detectada = cor
        return cor
    except rospy.ServiceException as e:
        rospy.logwarn(f"Serviço de câmera falhou: {e}")
        return "nenhuma"
    except rospy.ROSException as e:
        rospy.logwarn(f"Timeout ao esperar pelo serviço de câmera: {e}")
        return "nenhuma"

if __name__ == '__main__':
    rospy.init_node('main_loop')

    # Configurações de log
    rospy.loginfo("Iniciando o main_loop")
    
    # Publisher para controlar o movimento
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    # Subscribers
    rospy.Subscriber('/lidar', LaserScan, lidar_callback)
    rospy.Subscriber('/navegacao/direcao', Float32, callback_direcao)
    
    rate = rospy.Rate(5)  # 5 Hz

    # Centraliza o servo no início
    mover_servo(0)

    try:
        while not rospy.is_shutdown():
            twist = Twist()

            if obstaculo_proximo:
                rospy.loginfo("Obstáculo detectado: analisando lados")
                
                # Verifica as cores à direita e à esquerda
                cor_direita = detectar_cor_em_direcao(45)  # Ajustado para 45° em vez de 90°
                cor_esquerda = detectar_cor_em_direcao(-45)  # Ajustado para -45° em vez de -90°
                mover_servo(0)  # Volta a câmera pro centro (0°)

                if cor_direita == "verde":
                    rospy.loginfo("Verde à direita: girando pra direita")
                    twist.linear.x = 0.0
                    twist.angular.z = -0.5  # Giro no sentido horário
                elif cor_esquerda == "verde":
                    rospy.loginfo("Verde à esquerda: girando pra esquerda")
                    twist.linear.x = 0.0
                    twist.angular.z = 0.5  # Giro no sentido anti-horário
                else:
                    rospy.loginfo("Sem verde nas laterais e obstáculo à frente: recuando")
                    twist.linear.x = -0.1  # Recua lentamente
                    twist.angular.z = 0.0
            else:
                if direcao_alvo is not None:
                    rospy.loginfo(f"Caminho livre com direção vetorial: {direcao_alvo}")
                    twist.linear.x = 0.2
                    # Limita o ajuste angular para evitar giros bruscos
                    twist.angular.z = max(-0.5, min(0.5, direcao_alvo * 0.01))
                else:
                    rospy.loginfo("Caminho livre sem direção vetorial: seguindo reto")
                    twist.linear.x = 0.2
                    twist.angular.z = 0.0

            pub.publish(twist)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        # Garante que o robô pare se o nó for encerrado
        stop_twist = Twist()
        pub.publish(stop_twist)
        rospy.loginfo("Main loop encerrado, robô parado.") 