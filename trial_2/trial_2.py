#!/usr/bin/env python

import os
import time
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import *
from visualization_msgs.msg import *
import string
import math

cx = None
cy = None
d = None

a = 0
b = 0
c = 0
o = 0
q = 0

local_pos_pub = None
local_vel_pub = None
local_acc_pub = None


class stateMoniter:

    def __init__(self, a=0, b=0, c=0):
        self.state = State()
        self.pos = Pose()
        self.a = a
        self.b = b
        self.c = c

    def stateCb(self, msg):
        # Callback function for topic /mavros/state
        #print("inside state callback")
        self.state = msg

    def poseCb(self, msg):
        global a
        global b
        global c
        global o

        self.pos.position.x = msg.pose.position.x
        self.pos.position.y = msg.pose.position.y
        self.pos.position.z = msg.pose.position.z
        self.pos.orientation.x = msg.pose.orientation.x
        self.pos.orientation.y = msg.pose.orientation.y
        self.pos.orientation.z = msg.pose.orientation.z
        self.pos.orientation.w = msg.pose.orientation.w

        a = self.pos.position.x
        b = self.pos.position.y
        c = self.pos.position.z

    def sendCurrentPosToStateMoniter(self, a, b, c):
        self.a = a
        self.b = b
        self.c = c

    def is_at_position(self, x, y, z):
        global a
        global b
        global c
        global o

        desired = np.array((x, y, z))
        pos = np.array((a, b, c))
        return np.linalg.norm(desired - pos)

    def image_callback(self, data):

        global cx
        global cy
        global d

        try:
            img = CvBridge().imgmsg_to_cv2(data, "bgr8")
            cv2.imshow('kuchbhichalega', img)

            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            red_lower = np.array([0, 170, 50])
            red_upper = np.array([10, 255, 255])

            mask = cv2.inRange(hsv, red_lower, red_upper)
            contours_red, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

            for contour in contours_red:
                M = cv2.moments(contour)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])

                else:
                    cx = 100
                    cy = 100

        except CvBridgeError as e:
            # print(e)
            cx = 100
            cy = 100

        finally:
            # print('cx,cy:',(cx,cy), end = '   ')
            return cx, cy

    def depth_callback(self, data):

        global cx
        global cy
        global d
        global q

        if q > 1:
            try:
                depth = CvBridge().imgmsg_to_cv2(data)
                d = depth[cy][cx]
                if len(d) > 2:
                    d = 1

            except CvBridgeError as e:
                # print(e)
                d = 1.00

            finally:
                # print('depth:',d, end = '   ')
                return d
        q = q+1


class offboard_control:

    def __init__(self):
        # Initialise rosnode
        #rospy.init_node('offboard_control', anonymous=True)
        self.dummy = 0

    def setArm(self):
        # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
        # Waiting untill the service starts
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone
            armService = rospy.ServiceProxy(
                '/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            print("Service arming call failed: %s" % e)

    def DisArm(self):
        # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
        # Waiting untill the service starts
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone
            armService = rospy.ServiceProxy(
                '/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException as e:
            print("Service disarming call failed: %s" % e)

    def offboard_set_mode(self):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            SetMode = rospy.ServiceProxy(
                '/mavros/set_mode', mavros_msgs.srv.SetMode)
            SetMode(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print("Service calling offboard failed: %s" % e)

    def land_set_mode(self):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            SetMode = rospy.ServiceProxy(
                '/mavros/set_mode', mavros_msgs.srv.SetMode)
            SetMode(custom_mode='AUTO.LAND')
        except rospy.ServiceException as e:
            print("Service calling Land failed: %s" % e)


def main():

    offboard = offboard_control()
    statemt = stateMoniter()

    start_time = time.time()

    global cx
    global cy
    global d

    global a
    global b
    global c
    global o
    global local_pos_pub
    global local_vel_pub
    global local_acc_pub

    surya = 0

    rospy.init_node('trial_1_node', anonymous=True)

    # Initialize subscriber
    rospy.Subscriber("mavros/state", State, statemt.stateCb)

    # Similarly initialize other subscribers
    rospy.Subscriber("mavros/local_position/pose", PoseStamped, statemt.poseCb)

    # Initialize publishers
    local_pos_pub = rospy.Publisher(
        'mavros/setpoint_position/local', PoseStamped, queue_size=10)
    local_vel_pub = rospy.Publisher(
        'mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)
    local_acc_pub = rospy.Publisher(
        'mavros/setpoint_accel/cmd_acc', Accel, queue_size=10)

    # Create empty message containers
    pos = PoseStamped()
    pos.pose.position.x = 0
    pos.pose.position.y = 0
    pos.pose.position.z = 0
    pos.pose.orientation.x = 0
    pos.pose.orientation.y = 0
    pos.pose.orientation.z = 0
    pos.pose.orientation.w = 0
    pos.header.frame_id = "1"

    # Set your velocity here
    vel = Twist()
    vel.linear.x = 0
    vel.linear.y = 0
    vel.linear.z = 0

    # Similarly add other containers
    acc = Accel()
    acc.linear.x = 0
    acc.linear.y = 0
    acc.linear.z = 0

    # rospy.init_node('gazebo2cv')
    time.sleep(2)
    image_sub = rospy.Subscriber(
        "depth_cam/rgb/image_raw", Image, statemt.image_callback)
    depth_sub = rospy.Subscriber(
        "depth_cam/depth/image_raw", Image, statemt.depth_callback)

    while not statemt.state.armed:
        offboard.setArm()
    print("Armed")

    while not statemt.state.mode == "OFFBOARD":
        # print("inside while")
        for i in range(100):
            local_pos_pub.publish(pos)
            offboard.offboard_set_mode()
        offboard.offboard_set_mode()
    print("OFFBOARD mode activated")

    o = 0.3
    chakkar = 0

    while not rospy.is_shutdown():
        # print('inside rospy.is_shutdown while loop')

        if d <= 2:
            o = a
        elif d <= 3.4:
            o = a + 0.85
        else:
            o = a + 1.45

        # print('depth: ', end = '')
        # print(d, end='   ')
        # print('a = ', end = '')
        # print(a, end = '    ')
        # print('o = ', end= '')
        # print(o)

        if surya < 3:
            print('takeoff delay')
            pos.pose.position.x = 0
            pos.pose.position.y = 0
            pos.pose.position.z = 2
            local_pos_pub.publish(pos)
            surya = surya + 1
            time.sleep(1)
            continue

        if a < 5 and b < 9.7 and chakkar == 0:
            chakkar = 1
            print('chakkar = 1')
            while a < 4.5:
                if d < 2:
                    o = a
                elif d < 3.4:
                    o = a + 0.85
                else:
                    o = a + 1.45
                pos.pose.position.x = o
                pos.pose.position.y = 0.3
                pos.pose.position.z = 1.8
                pos.pose.orientation.x = 0
                pos.pose.orientation.y = 0
                pos.pose.orientation.z = -0.06
                pos.pose.orientation.w = -1
                local_pos_pub.publish(pos)
                print('depth:',end = ' ')
                print(d)

        elif a > 5 and b < 9.7:
            chakkar = 2
            print('chakkar = 2')
            time.sleep(0.8)
            while b < 10:
                if d < 2:
                    o = b
                elif d < 3.4:
                    o = b + 0.85
                else:
                    o = b + 1.55
                pos.pose.position.x = 5
                pos.pose.position.y = o
                pos.pose.position.z = 1.8
                pos.pose.orientation.x = 0
                pos.pose.orientation.y = 0
                pos.pose.orientation.z = -0.7
                pos.pose.orientation.w = -0.7
                local_pos_pub.publish(pos)
                print('depth:',end = ' ')
                print(d)

        elif a > 5 and b > 10 and chakkar == 2:
            chakkar = 3
            print('chakkar = 3')
            time.sleep(0.8)
            while a > -0.9:
                if d < 2:
                    o = a
                elif d < 3.4:
                    o = a - 0.8
                else:
                    o = a - 1.25
                pos.pose.position.x = o
                pos.pose.position.y = 9.7
                pos.pose.position.z = 1.8
                pos.pose.orientation.x = 0
                pos.pose.orientation.y = 0
                pos.pose.orientation.z = -1
                pos.pose.orientation.w = 0
                local_pos_pub.publish(pos)
                print('depth:',end = ' ')
                print(d)

        elif a < -0.85 and b > 9 and chakkar == 3:
            chakkar = 4
            print('chakkar = 4')
            time.sleep(0.8)
            while b > 0.5:
                if d < 2:
                    o = b
                elif d < 3.4:
                    o = b - 0.85
                else:
                    o = b - 1.55
                pos.pose.position.x = -1.2
                pos.pose.position.y = o
                pos.pose.position.z = 1.8
                pos.pose.orientation.x = 0
                pos.pose.orientation.y = 0
                pos.pose.orientation.z = -0.7
                pos.pose.orientation.w = 0.7
                local_pos_pub.publish(pos)
                print('depth:',end = ' ')
                print(d)

        elif chakkar == 4:
            print('breaking')
            break

    print('outside rospy.is_shutdown while loop')
    while not statemt.state.mode == 'AUTO.LAND':
        offboard.land_set_mode()
        time.sleep(1)
    print("Land Mode")


if __name__ == '__main__':
    main()
    rospy.spin()
