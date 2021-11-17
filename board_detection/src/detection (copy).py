#!/usr/bin/env python

# ----------------------------------------------------------
# detection.py
# subscribed topics: /camera/rgb/image_rect_color, /camera/depth/points
# published topics: /tf_static
# 
# chelsea.starr@onsemi.com
# 
# TODO: 
#   - [X] error check for no contour found
#   - [ ] make detection more robust (varied lighting, play with hsv)
#   - [ ] sometimes pc returns NaN - test if current workaroud is sufficient
#   - [ ] test offset by sending EE to xyz coordinate
# ----------------------------------------------------------

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import imutils
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import tf
import tf.msg
import tf2_ros
import geometry_msgs.msg
import ros_numpy
import math

# ----------------------------------------------------------
# class Board
# hold fruit location info
# ----------------------------------------------------------
class Board():
    def __init__(self):

        # physical coordinates to center point
        self.x = -1
        self.y = -1
        self.z = -1

        # physical orientation
        self.R = -1
        self.P = -1
        self.Y = -1

        # pixel coordinates to center point
        self.u = -1
        self.v = -1

        # orientation in image 
        self.angle = -1

# ----------------------------------------------------------
# class Scene
# class to subscribe to and hold momentary rgbd info
# ----------------------------------------------------------
class Scene(object):

    # ---------------------------------------------------------
    # def __init__
    # ---------------------------------------------------------
    def __init__(self):

        # rgb image and info
        self.image_sub = rospy.Subscriber("/camera/rgb/image_rect_color",Image,self.image_callback)
        self.bridge_object = CvBridge()
        self.image_info = CameraInfo
        self.rgb_image = []

        
        # pointcloud
        self.pointcloud_sub = rospy.Subscriber('/camera/depth/points', PointCloud2, self.pointcloud_callback)
        self.pointcloud = []


    # ----------------------------------------------------------
    # def poitcloud_callback
    # ----------------------------------------------------------
    def pointcloud_callback(self, data):
        assert isinstance(data, PointCloud2)
        self.pointcloud = ros_numpy.point_cloud2.pointcloud2_to_array(data)


    # ----------------------------------------------------------
    # def image_info_callback
    # ----------------------------------------------------------
    def image_info_callback(self,info):
        self.image_info = info


    # ----------------------------------------------------------
    # def image_callback
    # initiates image processing each time new color image is recieved
    # ----------------------------------------------------------
    def image_callback(self,data):
        # print("in image callback")
        try:
            # We select bgr8 because its the OpenCV encoding by default
            self.rgb_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

class HomogeneousBgDetector():
    def __init__(self):
        pass

    def detect_objects(self, frame):
        # Convert Image to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Create a Mask with adaptive threshold
        mask = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 19, 5)

        # Find contours
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)

        #cv2.imshow("mask", mask)
        objects_contours = []

        # cnt = max(contours, key=cv2.contourArea)
        # objects_contours.append(cnt)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 2000:
                #cnt = cv2.approxPolyDP(cnt, 0.03*cv2.arcLength(cnt, True), True)
                objects_contours.append(cnt)

        return objects_contours

    # def get_objects_rect(self):
    #     box = cv2.boxPoints(rect)  # cv2.boxPoints(rect) for OpenCV 3.x
    #     box = np.int0(box)

# ----------------------------------------------------------
# def identify_board_aruco()
# input: RGB image, hsv range
# output: center pixel (u,v), orientation
# ----------------------------------------------------------
def identify_board_aruco(rgb_frame, belt_frame,detector,parameters,aruco_dict):
    
    cX = -1
    cY = -1
    angle = -1

    # Get Aruco marker
    #corners, _, _ = cv2.aruco.detectMarkers(belt_frame, aruco_dict, parameters=parameters)
    # if corners:
    # Draw polygon around the marker
    #int_corners = np.int0(corners)
    #cv2.polylines(belt_frame, int_corners, True, (0, 255, 0), 2)
    
    # # Aruco Perimeter
    # aruco_perimeter = cv2.arcLength(corners[0], True)
    
    # # Pixel to cm ratio
    # pixel_cm_ratio = aruco_perimeter / 20
    
    contours = detector.detect_objects(belt_frame)
    if contours:
    
        # Draw objects boundaries
        for cnt in contours:
            # Get rect
            rect = cv2.minAreaRect(cnt)
            (x, y), (w, h), ang = rect
        
            # Get Width and Height of the Objects by applying the Ratio pixel to cm
            object_width = w #/ pixel_cm_ratio
            object_height = h #/ pixel_cm_ratio

            if round(object_width, 0) in range(70,100):
                if round(object_height, 0) in range(70,100):
        
                    # Display rectangle
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                 
                    cX = int(x)
                    cY = int(y)+210

                    angle = rect[-1]
                
                    cv2.circle(rgb_frame, (cX, cY), 5, (0, 0, 255), -1)
                    cv2.polylines(belt_frame, [box], True, (255, 0, 0), 2)
                    cv2.putText(rgb_frame, "Width {} cm".format(round(object_width, 1)), (int(x - 100), int(y - 20)), cv2.FONT_HERSHEY_PLAIN, 1, (100, 200, 0), 1)
                    cv2.putText(rgb_frame, "Height {} cm".format(round(object_height, 1)), (int(x - 100), int(y + 15)), cv2.FONT_HERSHEY_PLAIN, 1, (100, 200, 0), 1)
        
    cv2.imshow("Belt Frame", belt_frame)
    cv2.imshow("RGB Frame", rgb_frame)
    cv2.waitKey(1)

    return cX, cY, angle
# ----------------------------------------------------------
# def identify_board()
# input: RGB image, hsv range
# output: center pixel (u,v), orientation
# ----------------------------------------------------------
def identify_board(rgb_frame, belt_frame):

    mask = cv2.cvtColor(belt_frame, cv2.COLOR_BGR2GRAY)
    mask = cv2.inRange(mask, 140, 255)
    # mask = object_detector.apply(belt_frame)

    # hsv = cv2.cvtColor(belt_frame, cv2.COLOR_BGR2HSV)
    # mask = cv2.inRange(hsv, (36,25,25),(70,255,255))
    # mask = cv2.inRange(belt_frame, (36,25,25),(185,0,0))


    # find contours in the mask
    contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)

    # return 0 if no contours are found
    if len(contours) == 0:
        return 0

    center = None

    center = max(contours, key=cv2.contourArea)
    x,y,w,h = cv2.boundingRect(center)
    # center = 
    # cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)

    detections = []
    minx =[]
    miny =[]
    for count in contours:
        rect = cv2.minAreaRect(count)
        board_box = cv2.boxPoints(rect)
        board_box = np.int0(board_box)
        cv2.drawContours(rgb_frame, [board_box], -1, (0, 255, 0), 2)
        x,y,w = cv2.minAreaRect(count)
        detections.append ([x,y,w])
        print (detections)

    ## BEGIN - draw rotated rectangle
    # rect_tuple = cv2.minAreaRect(center)
    # p1 = tuple()[0[1+90]]
    # p2 = tuple()[[]]
    # rect = rect_tuple
    # print(type(rect))
    rect = cv2.minAreaRect(center)
    p1 = rect[0]
    p2 = (p1[0],p1[1]+190)
    # p3 = rect[1]
    # p4 = (p3[0],p3[1]+190)
    rect = (p2,rect[1],rect[2])

    # rect = rect[[]]
    board_box = cv2.boxPoints(rect)
    board_box = np.int0(board_box)
    cv2.drawContours(rgb_frame,[board_box],0,(0,191,255),2)

    # compute the center of the contour
    M = cv2.moments(center)
    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"]) + 190
    else: return 0

    # draw the contour and center of the shape on the image
    # cv2.drawContours(rgb_frame, [center], -1, (0, 255, 0), 2)
    cv2.circle(rgb_frame, (cX, cY), 7, (255, 255, 255), -1)
    # cv2.drawContours(rgb_frame, (cX, cY), -1, (0, 255, 0), 2)

    cv2.imshow("Mask", mask)
    cv2.imshow("Belt Frame", belt_frame)
    cv2.imshow("RGB Frame", rgb_frame)

    angle = rect[-1]

    # cv2.waitKey(1)

    return cX, cY, angle


# ----------------------------------------------------------
# def identify_belt()
# input: rgb image
# output: grayscale image showing only belt and items on it
# ----------------------------------------------------------
def identify_belt(cv_image):

    # process image
    belt = cv_image[190:345,0:360]
    cv2.imshow("frame1",belt)


    # gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    # # construct a mask for the color fruit, then perform
    # # a series of dilations and erosions to remove any small
    # # blobs left in the mask
    # mask = cv2.inRange(gray, 0, 90)
    # mask = cv2.erode(mask, None, iterations=2)
    # mask = cv2.dilate(mask, None, iterations=2)


    # # find contours in the mask
    # contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # contours = imutils.grab_contours(contours)

    # # return empty array if no contours are found
    # if len(contours) == 0:
    #     return np.array([])

    # center = None

    # center = max(contours, key=cv2.contourArea)
    # x,y,w,h = cv2.boundingRect(center)
    # # cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)

    # ## BEGIN - draw rotated rectangle
    # rect = cv2.minAreaRect(center)
    # belt_box = cv2.boxPoints(rect)
    # belt_box = np.int0(belt_box)
    # cv2.drawContours(frame,[belt_box],0,(0,191,255),2)
    # ## END - draw rotated rectangle


    # belt_mask = np.zeros(frame.shape, np.uint8)
    # cv2.drawContours(belt_mask, center, -1, (0,255,0),1)
    # cv2.fillPoly(belt_mask, pts =[center], color=(255,255,255))
    # belt_mask = cv2.cvtColor(belt_mask, cv2.COLOR_BGR2GRAY)

    # belt = cv2.bitwise_and(gray,gray,mask=belt_mask)

    cv2.waitKey(1)

    return belt


# ----------------------------------------------------------
# def locate_board()
# input: pointcloud2, u, v, angle
# output: XYZRPY as TF
# ----------------------------------------------------------
def orient_board(board):
    
    return 0,0,0,0,0,0


# ----------------------------------------------------------
# def get_hsv()
# description: for development only. click pixel for HSV val
# ----------------------------------------------------------
def get_hsv(img):

    HSV=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    #Mouse click response event
    def getposHsv(event,x,y,flags,param):
        if event==cv2.EVENT_LBUTTONDOWN:
            print("HSV is",HSV[y,x])
    def getposBgr(event,x,y,flags,param):
        if event==cv2.EVENT_LBUTTONDOWN:
            print("Bgr is",img[y,x])
    #
    cv2.imshow("imageHSV",HSV)
    cv2.imshow('image',img)
    cv2.setMouseCallback("imageHSV",getposHsv) 
    cv2.setMouseCallback("image",getposBgr)
    cv2.waitKey(0)


# ----------------------------------------------------------
# def board_tf_publisher()
# description: publish board location/orientiation as static tf
# input: broadcaster var, board (location data)
# output: ros /tf
# ----------------------------------------------------------
# def board_tf_publisher(broadcaster, board):

#     static_transformStamped = geometry_msgs.msg.TransformStamped()

#     static_transformStamped.header.stamp = rospy.Time.now()
#     static_transformStamped.header.frame_id = "camera_link"
#     static_transformStamped.child_frame_id = "board"

#     static_transformStamped.transform.translation.x = board.x
#     static_transformStamped.transform.translation.y = board.y
#     static_transformStamped.transform.translation.z = board.z

#     quat = tf.transformations.quaternion_from_euler(board.angle*3.14/180,0,0)
#     static_transformStamped.transform.rotation.x = quat[0]
#     static_transformStamped.transform.rotation.y = quat[1]
#     static_transformStamped.transform.rotation.z = quat[2]
#     static_transformStamped.transform.rotation.w = quat[3]

#     broadcaster.sendTransform(static_transformStamped)

# ----------------------------------------------------------
# class to publish fruit xyz as /tf
# ----------------------------------------------------------
class DynamicTFBroadcaster:


    def __init__(self,board):
        self.pub_tf = rospy.Publisher("/tf", tf.msg.tfMessage, queue_size=10)

        change = 0.0

        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = "camera_link"
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = "board"
        t.transform.translation.x = board.x
        t.transform.translation.y = board.y
        t.transform.translation.z = board.z

        quat = tf.transformations.quaternion_from_euler(board.angle*3.14/180,0,0)
        # quat = tf.transformations.quaternion_from_euler(board.angle*3.14/180,-1.42,0)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        tfm = tf.msg.tfMessage([t])
        self.pub_tf.publish(tfm)


# ----------------------------------------------------------
# def main()
# initialization and main while True
# ----------------------------------------------------------
def main():

    rospy.init_node('detect_board', anonymous=True)

    # get rgbd info - continuously updated via subscriber callbacks
    scene = Scene()

    # allow image subscribers to get data
    rospy.sleep(5)

    # initialize tf broadcaster
    # broadcaster = tf2_ros.StaticTransformBroadcaster()
    
    
    # Load Aruco detector
    parameters = cv2.aruco.DetectorParameters_create()
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)

    # Load Object Detector
    detector = HomogeneousBgDetector()
    
    #i = 0
    while True:

        # initialize class to hold board location information
        board = Board()

        #i = i + 1
        #pic_name = "board" + str(i) + ".jpg"
        #cv2.imwrite(pic_name, scene.rgb_image)

        # rospy.sleep(1)

        # identify conveyor belt, else filter out
        output = scene.rgb_image[210:365,0:270]
        #output = identify_belt(scene.rgb_image)
        if output.size == None:
            print("belt not detected")
            continue
        belt_image = output
        cv2.imshow("belt_image",belt_image)
        
        # get board (center) pixel coordinate and orientation
        output = identify_board_aruco(scene.rgb_image, belt_image,detector,parameters,aruco_dict)
        if output[0] == -1: 
            print("board not detected") 
            continue
        board.u,board.v,board.angle = output

        # # get physical coordinates
        board.y,board.z,board.x = scene.pointcloud[board.v] [board.u]
        board.y = -board.y
        board.z = -board.z

        # # FOR DEBUGGING
        # # board.x = 0.3
        # # board.y = 0.05
        # # board.z = 0
        # # board.angle = 45
        # # 1.089,-0.577104,-0.449721

        # # for debugging
        print("u,v: (" + str(board.u) + "," + str(board.v) + ")")
        print("(x,y,z): (" + str(board.x) + "," + str(board.y) + "," + str(board.z) + ")")
        print("angle: " + str(board.angle))
        if (math.isnan(board.x)):
            print("No \TF Published \n")
            continue
        print("\n")


        # publish board transform
        # board_tf_publisher(broadcaster, board)
        tfb = DynamicTFBroadcaster(board)
        
        # rospy.sleep(0.5)

        try:
            continue
        except KeyboardInterrupt:
            cv2.destroyAllWindows()
            break


if __name__ == '__main__':
    main()