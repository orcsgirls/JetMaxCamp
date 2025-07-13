#!/usr/bin/env python3
import sys
import cv2
import math
import time
import rospy
import numpy as np
import threading
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
from std_srvs.srv import Empty
from std_msgs.msg import String
from jetmax_control.msg import SetServo
import hiwonder
import queue
import pupil_apriltags as apriltag
import yaml
import json

ROS_NODE_NAME = "apriltag_detector_publisher"
TAG_SIZE = 33.30


class AprilTagDetect:
    def __init__(self):
        self.camera_params = None
        self.K = None
        self.R = None
        self.T = None

    def load_camera_params(self):
        self.camera_params = rospy.get_param('/camera_cal/block_params', self.camera_params)
        if self.camera_params is not None:
            self.K = np.array(self.camera_params['K'], dtype=np.float64).reshape(3, 3)
            self.T = np.array(self.camera_params['T'], dtype=np.float64).reshape(3, 1)
            self.R = np.array(self.camera_params['R'], dtype=np.float64).reshape(3, 1)
            r_mat = np.zeros((3, 3), dtype=np.float64)
            cv2.Rodrigues(self.R, r_mat)
            self.r_mat = r_mat

def camera_to_world(cam_mtx, r_mat, t, img_points):
    inv_k = np.asmatrix(cam_mtx).I
    # invR * T
    inv_r = np.asmatrix(r_mat).I  # 3*3
    transPlaneToCam = np.dot(inv_r, np.asmatrix(t))  # 3*3 dot 3*1 = 3*1
    world_pt = []
    coords = np.zeros((3, 1), dtype=np.float64)
    for img_pt in img_points:
        coords[0][0] = img_pt[0][0]
        coords[1][0] = img_pt[0][1]
        coords[2][0] = 1.0
        worldPtCam = np.dot(inv_k, coords)  # 3*3 dot 3*1 = 3*1
        # [x,y,1] * invR
        worldPtPlane = np.dot(inv_r, worldPtCam)  # 3*3 dot 3*1 = 3*1
        # zc
        scale = transPlaneToCam[2][0] / worldPtPlane[2][0]
        # zc * [x,y,1] * invR
        scale_worldPtPlane = np.multiply(scale, worldPtPlane)
        # [X,Y,Z]=zc*[x,y,1]*invR - invR*T
        worldPtPlaneReproject = np.asmatrix(scale_worldPtPlane) - np.asmatrix(transPlaneToCam)  # 3*1 dot 1*3 = 3*3
        pt = np.zeros((3, 1), dtype=np.float64)
        pt[0][0] = worldPtPlaneReproject[0][0]
        pt[1][0] = worldPtPlaneReproject[1][0]
        pt[2][0] = 0
        world_pt.append(pt.T.tolist())
    return world_pt

def image_proc_a(img):
    frame_gray = cv2.cvtColor(np.copy(img), cv2.COLOR_RGB2GRAY)
    params = [state.K[0][0], state.K[1][1], state.K[0][2], state.K[1][2]] 
    tags = at_detector.detect(frame_gray, estimate_tag_pose=True, camera_params=params, tag_size=TAG_SIZE)
    data=[]
    for tag in tags:
        corners = tag.corners.reshape(1, -1, 2).astype(int)
        center = tag.center.astype(int)

        cv2.drawContours(img, corners, -1, (255, 0, 0), 3)
        cv2.circle(img, tuple(center), 5, (255, 255, 0), 10)

        rotM = tag.pose_R
        tvec = tag.pose_t

        # Distance

        point_3d = np.array([[16.65, -16.65, 0], [-16.65,-16.65, 0], [-16.65, 16.65, 0], [16.65, 16.65, 0]], dtype=np.double)
        point_2d = np.array([tag.corners[0].astype(int), tag.corners[1].astype(int), tag.corners[2].astype(int),
                             tag.corners[3].astype(int)],
                             dtype=np.double)

        dist_coefs = np.array([0,0,0,0], dtype=np.double)
        found, rvec, tvec = cv2.solvePnP(point_3d, point_2d, state.K,  None)
        rotM = cv2.Rodrigues(rvec)[0]
        camera_position = -np.matrix(rotM).T * np.matrix(tvec)
        distance = -camera_position.T.tolist()[0][2]

        # Position and rotation

        x, y, _ = camera_to_world(state.K, state.r_mat, state.T, tag.center.reshape((1,1,2)))[0][0]
        theta_z = math.atan2(rotM[1, 0], rotM[0, 0])*180.0/math.pi
        theta_y = math.atan2(-1.0*rotM[2, 0], math.sqrt(rotM[2, 1]**2 + rotM[2, 2]**2))*180.0/math.pi
        theta_x = math.atan2(rotM[2, 1], rotM[2, 2])*180.0/math.pi

        s1 = "id: {}".format(tag.tag_id)
        s2 = "x: {:0.1f}mm, y: {:0.1f}mm".format(x, y)
        s3 = "angle: {:0.1f}deg".format(theta_z)
        s4 = "dist: {:0.0f}mm".format(distance)
        cv2.putText(img, s1, (center[0] + 50, center[1]    ), 0, 0.5, (255, 255, 0), 2)
        cv2.putText(img, s2, (center[0] + 50, center[1]+ 20), 0, 0.5, (255, 255, 0), 2)
        cv2.putText(img, s3, (center[0] + 50, center[1]+ 40), 0, 0.5, (255, 255, 0), 2)
        cv2.putText(img, s4, (center[0] + 50, center[1]+ 60), 0, 0.5, (255, 255, 0), 2)

        # Data package

        data.append({"id": tag.tag_id, "x": x, "y": y, "angle": theta_z, "distance": distance})


    pub.publish(json.dumps(data))

    img_h, img_w = img.shape[:2]
    cv2.line(img, (int(img_w / 2 - 10), int(img_h / 2)), (int(img_w / 2 + 10), int(img_h / 2)), (0, 255, 255), 2)
    cv2.line(img, (int(img_w / 2), int(img_h / 2 - 10)), (int(img_w / 2), int(img_h / 2 + 10)), (0, 255, 255), 2)
    return img


def image_proc_b():
    ros_image = image_queue.get(block=True)
    image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)
    frame_result = image.copy()
    frame_result = image_proc_a(frame_result)
    bgr_image = cv2.cvtColor(frame_result, cv2.COLOR_RGB2BGR)
    cv2.imshow('result', bgr_image)
    cv2.waitKey(1)


def image_callback(ros_image):
    try:
        image_queue.put_nowait(ros_image)
    except queue.Full:
        pass


if __name__ == '__main__':
    state = AprilTagDetect()
    at_detector = apriltag.Detector()
    image_queue = queue.Queue(maxsize=1)
    rospy.init_node(ROS_NODE_NAME, log_level=rospy.DEBUG)
    pub=rospy.Publisher('apriltag_location', String, queue_size=1)

    state.load_camera_params()
    if state.camera_params is None:
        rospy.logerr("Can not load camera params")
        sys.exit(-1)
    rospy.sleep(1)
    image_sub = rospy.Subscriber('/usb_cam/image_rect_color', Image, image_callback, queue_size=1)

    while not rospy.is_shutdown():
        image_proc_b()
