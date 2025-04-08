#!/usr/bin/env python3

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import rospy
from drivers.srv import Camera, CameraResponse
from sensor_msgs.msg import Image

# Último frame capturado (para evitar múltiplas capturas desnecessárias)
last_frame = None
bridge = CvBridge()

def contem_cor(hsv_img, lower, upper, min_pixels):
    """Verifica se a imagem contém uma quantidade mínima de pixels de uma determinada cor."""
    mask = cv2.inRange(hsv_img, lower, upper)
    return np.sum(mask > 0) > min_pixels

def capture_frame():
    """Captura um frame da câmera."""
    global last_frame
    
    cap = cv2.VideoCapture(rospy.get_param('/device'), cv2.CAP_V4L)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, rospy.get_param('/width'))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, rospy.get_param('/height'))
    
    ret, frame = cap.read()
    cap.release()
    
    if ret:
        frame = cv2.flip(frame, -1)
        last_frame = frame
        return frame
    return last_frame  # Retorna o último frame válido se a captura falhar

def analisa_cor(frame):
    """Analisa cores presentes no frame."""
    if frame is None:
        return "erro"
        
    try:
        # Converte para HSV para análise de cor
        img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Lê parâmetros do YAML via ROS
        min_pixels = rospy.get_param('/min_pixels')
        
        verde_inf = np.array(rospy.get_param('/verde_inf'))
        verde_sup = np.array(rospy.get_param('/verde_sup'))
        
        vermelho_inf1 = np.array(rospy.get_param('/vermelho_inf1'))
        vermelho_sup1 = np.array(rospy.get_param('/vermelho_sup1'))
        vermelho_inf2 = np.array(rospy.get_param('/vermelho_inf2'))
        vermelho_sup2 = np.array(rospy.get_param('/vermelho_sup2'))
        
        # Verifica se contém as cores
        tem_verde = contem_cor(img_hsv, verde_inf, verde_sup, min_pixels)
        tem_vermelho = (
            contem_cor(img_hsv, vermelho_inf1, vermelho_sup1, min_pixels) or
            contem_cor(img_hsv, vermelho_inf2, vermelho_sup2, min_pixels)
        )
        
        if tem_verde:
            return "verde"
        elif tem_vermelho:
            return "vermelho"
        else:
            return "nenhuma"
            
    except Exception as e:
        rospy.logerr(f"Erro na análise de cor: {e}")
        return "erro"

def camera_callback(req):
    """Callback para o serviço de câmera."""
    frame = capture_frame()
    
    if frame is None:
        return CameraResponse(result="erro")
    
    # Realiza a análise de cor
    cor = analisa_cor(frame)
    
    # Retorna o resultado da cor
    return CameraResponse(result=cor)

if __name__ == '__main__':
    rospy.init_node('camera_service')
    
    # Inicia o serviço
    camera_srv = rospy.Service('camera', Camera, camera_callback)
    
    rospy.loginfo("Serviço de câmera iniciado.")
    rospy.spin()
