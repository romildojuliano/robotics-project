#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import numpy as np
import math

# Configuração do gráfico (fora do callback, para reaproveitar a figura)
RMAX = 32.0
fig = plt.figure()
fig.canvas.manager.set_window_title('ROS LIDAR Monitor')
lidar_polar = plt.subplot(polar=True)

def callback(scan_data):
    angles = []
    ranges = []

    angle = scan_data.angle_min
    for r in scan_data.ranges:
        if not math.isinf(r):
            # Conversão para coordenadas polares com 90° no topo
            angles.append(angle - 1.1)
            ranges.append(r)
        angle += scan_data.angle_increment

    # Atualiza o gráfico
    lidar_polar.clear()
    lidar_polar.set_theta_zero_location('N')  # 90° no topo
    lidar_polar.set_theta_direction(-1)       # Sentido anti-horário
    lidar_polar.set_rmax(RMAX)
    lidar_polar.grid(True)
    lidar_polar.scatter(angles, ranges, c='r', s=5, alpha=0.75)

    # Salva a figura
    plt.savefig('scan.png')
    rospy.loginfo("Imagem salva como scan.png")

def listener():
    rospy.init_node('lidar_plotter', anonymous=True)
    rospy.Subscriber('/lidar', LaserScan, callback)
    rospy.loginfo("Aguardando dados do tópico /lidar...")
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
