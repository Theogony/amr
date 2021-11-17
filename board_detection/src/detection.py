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
#import ros_numpy
import math

#--- Define Tag
id_to_find  = 10
marker_size  = 5 #- [cm]
#--- Get the camera calibration path
calib_path  = ""
#camera_matrix   = np.loadtxt(calib_path+'cameraMatrix_webcam.txt', delimiter=',')
#camera_distortion   = np.loadtxt(calib_path+'cameraDistortion_webcam.txt', delimiter=',')
camera_matrix = np.array([[698.015333, 0.000000, 641.757223],[0.000000, 700.547964, 357.506101],[0.000000, 0.000000, 1.000000]])
camera_distortion = np.array([-0.300847, 0.070926, 0.001054, -0.001241, 0.000000])

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
        relative_name = rospy.get_param('/image_topic_name','/usb_cam/image_raw')
        #relative_name = rospy.get_param('/image_topic_name','/usb_cam/image_rect_color')
       
        self.image_sub = rospy.Subscriber(relative_name,Image,self.image_callback)
        self.bridge_object = CvBridge()
        self.image_info = CameraInfo
        self.rgb_image = []
        print (self.image_info)
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
def identify_aruco(rgb_frame, parameters,aruco_dict):
    
    pixel_cm_ratio =-1
    cm_depth =-1


    
    #-- Find all the aruco markers in the image
    corners, ids, rejected = cv2.aruco.detectMarkers(image=rgb_frame, dictionary=aruco_dict, parameters=parameters)
    
    if ids is not None and ids[0] == id_to_find:
    
        if corners:
            # Draw polygon around the marker
            # int_corners = np.int0(corners)
            # cv2.polylines(rgb_frame, int_corners, True, (0, 255, 0), 2)
        
            #-- ret = [rvec, tvec, ?]
            #-- array of rotation and position of each marker in camera frame
            #-- rvec = [[rvec_1], [rvec_2], ...]    attitude of the marker respect to camera frame
            #-- tvec = [[tvec_1], [tvec_2], ...]    position of the marker in camera frame
            ret = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)

            #-- Unpack the output, get only the first
            rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]

            #-- Draw the detected marker and put a reference frame over it
            cv2.aruco.drawDetectedMarkers(rgb_frame, corners)
            cv2.aruco.drawAxis(rgb_frame, camera_matrix, camera_distortion, rvec, tvec, 10)

            # Aruco Perimeter
            # using a DICT_5X5_50 printed at 50% so its 2.5cmx2.5cm need to compensate for that
            aruco_perimeter = cv2.arcLength(corners[0], True)
            # Pixel to cm ratio
            pixel_cm_ratio = aruco_perimeter / 20 # for a DICT_5X5_50
            cm_depth = tvec[2] - 5 # 5 is a value mesured with the gripper attached
            # Get Width and Height of the Objects by applying the Ratio pixel to cm
            rect = cv2.minAreaRect(corners[0])
            (x, y), (w, h), ang = rect
            object_width = w / pixel_cm_ratio
            object_height = h / pixel_cm_ratio

            #-- Print the tag position in camera frame
            str_position = "MARKER Position x=%4.0f  y=%4.0f  z=%4.0f"%(tvec[0], tvec[1], cm_depth)
            cv2.putText(rgb_frame, str_position, (400, 500), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 0), 1, cv2.LINE_AA)

            cv2.circle(rgb_frame, (int(x), int(y)), 5, (0, 0, 255), -1)
            cv2.putText(rgb_frame, "Width {} cm".format(round(object_width, 2)), (400, 500+20), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 0), 1, cv2.LINE_AA)
            cv2.putText(rgb_frame, "Height {} cm".format(round(object_height, 2)), (400, 500+40), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 0), 1, cv2.LINE_AA)
            cv2.waitKey(1)

    return pixel_cm_ratio,cm_depth

def identify_board2(rgb_frame,detector,pixel_cm_ratio):
    
    cX = -1
    cY = -1
    angle = -1

   
    contours = detector.detect_objects(rgb_frame)
    if contours:
    
        # Draw objects boundaries
        for cnt in contours:
            # Get rect
            rect = cv2.minAreaRect(cnt)
            (x, y), (w, h), ang = rect
        
            # Get Width and Height of the Objects by applying the Ratio pixel to cm
            object_width = w / pixel_cm_ratio
            object_height = h / pixel_cm_ratio

            if round(object_width, 0) in range(6,12):
                if round(object_height, 0) in range(6,12):
        
                    # Display rectangle
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                 
                    cX = int(x)
                    cY = int(y)

                    angle = rect[-1]
                
                    cv2.circle(rgb_frame, (cX, cY), 5, (0, 0, 255), -1)
                    cv2.polylines(rgb_frame, [box], True, (255, 0, 0), 2)
                    cv2.putText(rgb_frame, "Width {} cm".format(round(object_width, 2)), (int(x - 100), int(y - 20)), cv2.FONT_HERSHEY_PLAIN, 1, (100, 200, 0), 1)
                    cv2.putText(rgb_frame, "Height {} cm".format(round(object_height, 2)), (int(x - 100), int(y + 15)), cv2.FONT_HERSHEY_PLAIN, 1, (100, 200, 0), 1)
        
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
    pixel_cm_ratio = -1
    cm_depth = -1 
    #i = 0
    while True:

        # initialize class to hold board location information
        board = Board()

        # recevie a image from image camera ros node
        output = scene.rgb_image
        output = np.uint8(output)
        if output.size == None:
            print ('image is empty')
        else:   

            cropped_image0 = output[80:680, 250:1000]
            cv2.imshow("depth",cropped_image0)
            # cv2.imshow("depth",output)
            cv2.waitKey(1)

            a, b = identify_aruco(scene.rgb_image,parameters,aruco_dict)

            if cm_depth <6 or cm_depth >20:
                # get distance to surface by detecting aruco
                pixel_cm_ratio, cm_depth = identify_aruco(scene.rgb_image,parameters,aruco_dict)
                # cropped_image0 = output[80:680, 250:1000]
                # cv2.imshow("depth",cropped_image0)
                # # cv2.imshow("depth",output)
                # cv2.waitKey(1)
            else:
                #get a list of objects detected in the scene that are ~7.5cmx~7.5cm 
                cX,Cy,angle = identify_board2(scene.rgb_image,detector,pixel_cm_ratio)
                board.u = cX
                board.v = Cy
                board.z = cm_depth
                board.x = 1280-cX
                board.y = 720-Cy
                board.angle = angle
                # cropped_image0 = output[80:680, 250:1000]
                # cv2.imshow("depth",cropped_image1)
                # cv2.imshow("frame1",output)
                # cv2.waitKey(1)

                #output = identify_belt(scene.rgb_image)
                #if output.size == None:
                #    print("belt not detected")
                #    continue
                #belt_image = output
                #cv2.imshow("belt_image",belt_image)
                
                
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

        try:
            continue
        except KeyboardInterrupt:
            cv2.destroyAllWindows()
            break


if __name__ == '__main__':
    main()