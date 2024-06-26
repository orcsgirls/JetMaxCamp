import sys
import cv2
import math
import threading
import numpy as np
import rospy
from sensor_msgs.msg import Image
import hiwonder

ROS_NODE_NAME = "color_angle"
unit_block_corners = np.asarray([[0, 0, 0],
                                 [20, -20, 0],  # TAG_SIZE = 33.30mm
                                 [-20, -20, 0],
                                 [-20, 20, 0],
                                 [20, 20, 0]],
                                dtype=np.float64)
unit_block_img_pts = None


class State:
    def __init__(self):
        self.target_colors = {}
        self.runner = None
        self.camera_params = None
        self.K = None
        self.R = None
        self.T = None
        self.WIDTH = None


    def load_camera_params(self):
        global unit_block_img_pts
        self.camera_params = rospy.get_param('/camera_cal/block_params', self.camera_params)
        if self.camera_params is not None:
            self.K = np.array(self.camera_params['K'], dtype=np.float64).reshape(3, 3)
            self.R = np.array(self.camera_params['R'], dtype=np.float64).reshape(3, 1)
            self.T = np.array(self.camera_params['T'], dtype=np.float64).reshape(3, 1)
            img_pts, jac = cv2.projectPoints(unit_block_corners, self.R, self.T, self.K, None)
            unit_block_img_pts = img_pts.reshape(5, 2)
            l_p1 = unit_block_img_pts[-1]
            l_p2 = unit_block_img_pts[-2]
            self.WIDTH = math.sqrt((l_p1[0] - l_p2[0]) ** 2 + (l_p1[1] - l_p2[1]) ** 2)
            print(unit_block_img_pts)

def image_proc(img):
    img_h, img_w = img.shape[:2]
    frame_gb = cv2.GaussianBlur(np.copy(img), (5, 5), 5)
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_RGB2LAB)  # Convert rgb to lab
    blocks = []
    for color_name, color in state.target_colors.items():  # Loop through all selected colors
        frame_mask = cv2.inRange(frame_lab, tuple(color['min']), tuple(color['max']))
        eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
        contour_area = map(lambda c: (c, math.fabs(cv2.contourArea(c))), contours)
        contour_area = list(filter(lambda c: c[1] > 1000, contour_area))  # Eliminate contours that are too small

        if len(contour_area) > 0:
            for contour, area in contour_area:  # Loop through all the contours found
                rect = cv2.minAreaRect(contour)
                center_x, center_y = rect[0]
                box = cv2.boxPoints(rect)  # The four vertices of the minimum-area-rectangle
                box_list = box.tolist()
                box = np.int0(box)

                cv2.drawContours(img, [box], -1, hiwonder.COLORS[color_name.upper()], 2)
                cv2.circle(img, (int(center_x), int(center_y)), 1, hiwonder.COLORS[color_name.upper()], 5)
                angle = rect[2]
                angle = angle - 90 if angle > 45 else angle
                s = "Angle:{:0.2f}deg".format(angle)
                cv2.putText(img, s, (int(center_x), int(center_y)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                print(s)

    cv2.line(img, (int(img_w / 2 - 10), int(img_h / 2)), (int(img_w / 2 + 10), int(img_h / 2)), (0, 255, 255), 2)
    cv2.line(img, (int(img_w / 2), int(img_h / 2 - 10)), (int(img_w / 2), int(img_h / 2 + 10)), (0, 255, 255), 2)

    return img

def image_callback(ros_image):
    image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)
    frame_result = np.copy(image)
    frame_result = image_proc(frame_result)
    image_bgr = cv2.cvtColor(frame_result, cv2.COLOR_RGB2BGR)
    cv2.imshow("result", image_bgr)
    cv2.waitKey(1)


if __name__ == '__main__':
    rospy.init_node(ROS_NODE_NAME, log_level=rospy.DEBUG)
    state = State()
    state.load_camera_params()
    if state.camera_params is None:
        rospy.logerr("Can not load camera params")
        sys.exit(-1)
    jetmax = hiwonder.JetMax()
    jetmax.go_home()
    rospy.sleep(1)
    state.target_colors = rospy.get_param('/lab_config_manager/color_range_list', {})
    del[state.target_colors['white']]
    del[state.target_colors['black']]
    del[state.target_colors['ball']]
    del[state.target_colors['tennis']]
    image_sub = rospy.Subscriber('/usb_cam/image_rect_color', Image, image_callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        sys.exit(0)
