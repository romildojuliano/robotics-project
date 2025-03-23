#!/usr/bin/env python3

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from drivers.srv import Camera, CameraResponse


def camera_callback(req):
    bridge = CvBridge()
    cap = cv2.VideoCapture(rospy.get_param('/device'), cv2.CAP_V4L)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, rospy.get_param('/width'))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, rospy.get_param('/height'))

    ret, frame = cap.read()
    cap.release()

    if ret:
        try:
            image = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            return CameraResponse(image)
        except CvBridgeError as e:
            rospy.logerr(e)
            return CameraResponse()
    else:
        return CameraResponse()


if __name__ == '__main__':
    rospy.init_node('camera')
    camera = rospy.Service('camera', Camera, camera_callback)
    rospy.spin()