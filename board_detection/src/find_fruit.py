#!/usr/bin/env python

# ----------------------------------------------------------
# find_fruit.py
# Detects fruit type and locations. Publishes locatoin as
# ROS transform.
#
# chelsea.starr@onsemi.com
# ----------------------------------------------------------


import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import imutils
import roslib
import tf
import tf.msg
import geometry_msgs.msg
import pyrealsense2
import math

# define detected hsv values
# orange
orangeLower = (12, 130, 120)
orangeUpper = (20, 255, 255)

# banana
bananaLower = (23, 80, 100)
bananaUpper = (35, 255, 255)

# apple
appleLower = (1, 90, 50)
appleUpper = (10, 255, 155)

# green apple
greenAppleLower = (35, 90, 10)
greenAppleUpper = (60, 255, 255)

# ----------------------------------------------------------
# class to hold info associated with individual fruit
# ----------------------------------------------------------
class Fruit():
    def __init__(self,x_pixel,y_pixel, type):
        self.x = -1
        self.y = -1
        self.z = -1

        self.x_pixel = x_pixel
        self.y_pixel = y_pixel

        self.type = type

        self.depth = -1


# ----------------------------------------------------------
# class for everying associated with finding the fruit
# ----------------------------------------------------------
class Finder(object):

    def __init__(self):

        # rgb image and info for that camera
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.image_callback)
        self.image_info_sub = rospy.Subscriber("/usb_cam/camera_info",CameraInfo,self.image_info_callback)
        self.bridge_object = CvBridge()

        # depth image and info for that camera
        self.depth_info_sub = rospy.Subscriber("/camera/depth/camera_info",CameraInfo, self.depth_info_callback)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_raw",Image, self.depth_callback)

        # var to store camera info
        self.depth_info = CameraInfo
        self.image_info = CameraInfo

        self.rgb_image = []
        self.depth_image = []

        # list to store detected fruit
        # self.fruits = []

    # ----------------------------------------------------------
    # image_info_callback
    # ----------------------------------------------------------
    def image_info_callback(self,info):
        self.image_info = info

    # ----------------------------------------------------------
    # image_info_callback
    # ----------------------------------------------------------
    def depth_info_callback(self,info):
        self.depth_info = info

    # ----------------------------------------------------------
    # image_callback
    # initiates image processing each time new color image is recieved
    # ----------------------------------------------------------
    def image_callback(self,data):
        try:
            # We select bgr8 because its the OpenCV encoding by default
            self.rgb_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        # self.fruits = []

        # get fruit location in xy image pixels
        # self.find_fruit_pixel(cv_image, orangeLower, orangeUpper, "ORANGE", self.fruits)
        # self.find_fruit_pixel(cv_image, bananaLower, bananaUpper, "BANANA", self.fruits)
        # self.find_fruit_pixel(cv_image, appleLower, appleUpper, "APPLE", self.fruits)
        # self.find_fruit_pixel(cv_image, greenAppleLower, greenAppleUpper, "APPLE", self.fruits)


    # ----------------------------------------------------------
    # depth_callback
    # runs each time depth image is recieved
    # initiates conversion to xyz physical coordinates
    # ----------------------------------------------------------
    def depth_callback(self,data):
        try:
            # We select bgr8 because its the OpenCV encoding by default
            self.depth_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="32FC1")
        except CvBridgeError as e:
            print(e)

        # # for each fruit, use x_pixel, y_pixel, and depth to find actual xyz coordinates
        # for i in range(0,len(self.fruits)):
        #     self.fruits[i].depth = cv_image[int(round(self.fruits[i].y_pixel))][int(round(self.fruits[i].x_pixel))]/1000
        #     result = self.convert_depth_to_phys_coord_using_realsense(self.depth_info, self.fruits[i])
        #
        # # publish fruit location as /tf
        # tfb = DynamicTFBroadcaster(self.fruits)


# ----------------------------------------------------------
# convert_depth_to_phys_coord_using_realsense
# uses rgb pixel and corresponding depth data to find physical
# location w.r.t cam
# pyrealsense (library for RealSense Intel cam) used for deprojection
# ----------------------------------------------------------
def convert_depth_to_phys_coord_using_realsense(cameraInfo, fruit):
    _intrinsics = pyrealsense2.intrinsics()
    _intrinsics.width = cameraInfo.width
    _intrinsics.height = cameraInfo.height
    _intrinsics.ppx = cameraInfo.K[2]
    _intrinsics.ppy = cameraInfo.K[5]
    _intrinsics.fx = cameraInfo.K[0]
    _intrinsics.fy = cameraInfo.K[4]
    #_intrinsics.model = cameraInfo.distortion_model
    _intrinsics.model  = pyrealsense2.distortion.none
    _intrinsics.coeffs = [i for i in cameraInfo.D]
    result = pyrealsense2.rs2_deproject_pixel_to_point(_intrinsics, [fruit.x_pixel, fruit.y_pixel], fruit.depth)
    #result[0]: right, result[1]: down, result[2]: forward
    fruit.x = result[2]
    fruit.y = -result[0]
    fruit.z = -result[1]

# ----------------------------------------------------------
# find_fruit_pixel()
# use opencv to detect fruit and add to fruits[] array
# ----------------------------------------------------------
def find_fruit_pixel(cv_image, fruitLower, fruitUpper, type, fruits):

    # process image
    frame = cv_image
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # construct a mask for the color fruit, then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, fruitLower, fruitUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None

    # find the largest contour in the mask, then use
    # it to compute the minimum enclosing circle and
    # centroid
    while True:

        # break if no contours are left
        if len(cnts) <= 0:
            break

        # get largest contour
        c = max(cnts, key=cv2.contourArea)

        # remove largest from list of contours
        for j in range(0,len(cnts)):
            if np.all(c == cnts[j]):
                cnts.pop(j)
                break

        # print np.where((cnts == c))
        # cnts = np.delete(cnts, np.argwhere(cnts == c))

        # get pixel coordinates of 'center' of contour
        ((x, y), radius) = cv2.minEnclosingCircle(c)


        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # only proceed if the radius meets a minimum size
        if radius > 20:

            # skip countours that are too close to other contours
            # (AKA avoid detecting the same fruit multiple times)
            too_close_flag = False
            for prev_fruit in fruits:
                dist = math.sqrt(pow(prev_fruit.x_pixel - x, 2) + pow(prev_fruit.y_pixel - y, 2))
                # print(str(prev_fruit.type) + " & " + str(type) + ": " + str(dist))
                if dist < 40:
                    # print "TOO CLOSE!"
                    too_close_flag = True
                    break

            if too_close_flag == False:

                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                # cv2.circle(frame, (int(x), int(y)), int(radius),
                #     (0, 255, 255), 2)
                # cv2.drawContours(frame, c, -1, (0,255,0), 3)


                # print center circle
                if type == "APPLE":
                    cv2.circle(frame, center, 5, (0, 0, 255), -1)
                elif type == "BANANA":
                    cv2.circle(frame, center, 5, (255, 0, 0), -1)
                elif type == "ORANGE":
                    cv2.circle(frame, center, 5, (0, 150, 0), -1)

                # add fruit to list of detected fruits
                fruits.append( Fruit(x, y, type) )

    cnts = []

    # cv2.imshow("Frame", mask)
    # cv2.imshow("Frame", frame)
    if type == "APPLE":

        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(frame, 'APPLE', (550, 425), font, 0.5, (0, 0, 255), 2, cv2.LINE_4)
        cv2.putText(frame, 'BANANA', (550, 450), font, 0.5, (255, 0, 0), 2, cv2.LINE_4)
        cv2.putText(frame, 'ORANGE', (550, 475), font, 0.5, (0, 150, 0), 2, cv2.LINE_4)

        cv2.imshow("Frame", frame)
        # cv2.imshow("Frame", mask)

    cv2.waitKey(1)

# ----------------------------------------------------------
# class to publish fruit xyz as /tf
# ----------------------------------------------------------
class DynamicTFBroadcaster:


    def __init__(self,fruits):
        self.pub_tf = rospy.Publisher("/tf", tf.msg.tfMessage, queue_size=10)

        change = 0.0

        t = []
        apples, oranges, bananas = 0,0,0
        # publish location as /tf for each fruit
        for i in range(0,len(fruits)):
            t.append(geometry_msgs.msg.TransformStamped())
            t[i].header.frame_id = "camera_link"
            t[i].header.stamp = rospy.Time.now()
            if fruits[i].type == "ORANGE":
                t[i].child_frame_id = fruits[i].type + str(oranges)
                oranges = oranges + 1
            elif fruits[i].type == "APPLE":
                t[i].child_frame_id = fruits[i].type + str(apples)
                apples = apples + 1
            elif fruits[i].type == "BANANA":
                t[i].child_frame_id = fruits[i].type + str(bananas)
                bananas = bananas + 1
            t[i].transform.translation.x = fruits[i].y
            t[i].transform.translation.y = fruits[i].z
            t[i].transform.translation.z = fruits[i].x
            t[i].transform.rotation.w = 1.0

            tfm = tf.msg.tfMessage([t[i]])
            self.pub_tf.publish(tfm)

            # print(fruits[i].type)

            change += 0.1

# ----------------------------------------------------------
# main()
# Image subscribers in finder_object get incoming rgb and
# depth data. fruits array is wiped and refilled with each
# while loop iteration
# ----------------------------------------------------------
def main():

    finder_object = Finder()
    rospy.init_node('finder')

    # allow image subscribers to get data
    rospy.sleep(2)

    while True:

        fruits = []

        # find fruit locations in rgb_image
        find_fruit_pixel(finder_object.rgb_image, orangeLower, orangeUpper, "ORANGE", fruits)
        find_fruit_pixel(finder_object.rgb_image, bananaLower, bananaUpper, "BANANA", fruits)
        find_fruit_pixel(finder_object.rgb_image, appleLower, appleUpper, "APPLE", fruits)
        find_fruit_pixel(finder_object.rgb_image, greenAppleLower, greenAppleUpper, "APPLE", fruits)

        # find depth corresponding to RGB pixel and convert tp physical coordinates
        for i in range(0,len(fruits)):
            fruits[i].depth = finder_object.depth_image[int(round(fruits[i].y_pixel))][int(round(fruits[i].x_pixel))]/1000
            result = convert_depth_to_phys_coord_using_realsense(finder_object.depth_info, fruits[i])

        # publish transforms
        tfb = DynamicTFBroadcaster(fruits)

        try:
            continue
        except KeyboardInterrupt:
            cv2.destroyAllWindows()
            break

if __name__ == '__main__':
    main()