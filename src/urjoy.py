#!/usr/bin/env python
"""
this is a hack from the URX package examples to use a ROS joystick
"""

import time

import rospy
from sensor_msgs.msg import Joy
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output as gripperOutMsg

import math3d as m3d
from math import pi

import urx

class Cmd(object):
    def __init__(self):
        self.reset()

    def reset(self):
        self.axis0 = 0
        self.axis1 = 0
        self.axis2 = 0
        self.axis3 = 0
        self.axis4 = 0
        self.axis5 = 0
        self.btn0 = 0
        self.btn1 = 0
        self.btn2 = 0
        self.btn3 = 0
        self.btn4 = 0
        self.btn5 = 0
        self.btn6 = 0
        self.btn7 = 0
        self.btn8 = 0
        self.btn9 = 0


class Service(object):
    def __init__(self, robot, linear_velocity=0.1, rotational_velocity=0.1, acceleration=0.1):
        rospy.init_node('urjoy')
        self.joystick = None
        self.robot = robot
        #max velocity and acceleration to be send to robot
        self.linear_velocity = linear_velocity
        self.rotational_velocity = rotational_velocity
        self.acceleration = acceleration
        #one button send the robot to a preprogram position defined by this variable in join space
        self.init_pose = [-2.0782002408411593, -1.6628931459654561, 2.067930303382134, -1.9172217394630149, 1.5489023943220621, 0.6783171005488982]

        self.cmd = Cmd()
        self.init_joystick()
        self.gripper_status = None
        self.init_robotiq()
        rate = rospy.Rate(20)
        # rospy.spin()
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

    def init_joystick(self):
        joySub = rospy.Subscriber("joy",Joy,self.joy_callback)


    def joy_callback(self,data):
        self.cmd.reset()
        #get joystick state
        #F710 joystick has 6 axes
        for i in range(0, len(data.axes)):
            val = data.axes[i]

            if abs(val) < 0.05:  #deadband already in ROS
                val = 0
            tmp = "self.cmd.axis" + str(i) + " = " + str(val)
            if val != 0:
                print(tmp)
                exec(tmp)

        #get button state
        #F710 has 12 buttons
        for i in range(0, len(data.buttons)):
            if data.buttons[i] != 0:
                tmp = "self.cmd.btn" + str(i) + " = 1"
                print(tmp)
                exec(tmp)

    def init_robotiq(self):
        self.robotiqPub = rospy.Publisher('/Robotiq2FGripperRobotOutput',gripperOutMsg,queue_size=1)

    def status_cb(self,msg):
        self.gripper_status = msg

    def loop(self):

        if self.cmd.btn9:
            print("moving to init pose")
            self.robot.movej(self.init_pose, acc=1, vel=0.1)

        #initalize speed array to 0
        speeds = [0, 0, 0, 0, 0, 0]

        #get linear speed from joystick
        speeds[0] = 1 * self.cmd.axis0 * self.linear_velocity #Y
        speeds[1] = -1 * self.cmd.axis1 * self.linear_velocity # X
        speeds[2] = -1 * self.cmd.axis3 * self.linear_velocity # Z
        speeds[4] = -1 * self.cmd.axis2 * self.linear_velocity #roll
        if self.cmd.btn7:
             speeds[3]= 1 * self.rotational_velocity #pitch
        if self.cmd.btn6:
             speeds[3]= -1 * self.rotational_velocity
        if self.cmd.btn5:
             speeds[5]= 1 * self.rotational_velocity #yaw
        if self.cmd.btn4:
             speeds[5]= -1 * self.rotational_velocity

        #for some reasons everything is inversed
        speeds = [-i for i in speeds]
        #Now sending to robot. tol by default and base csys if btn2 is on
        if speeds != [0 for _ in speeds]:
            print("Sending ", speeds)
#            if self.cmd.btn7:
#                self.robot.speedl_tool(speeds, acc=self.acceleration, min_time=2)
#            else:
#                self.robot.speedl(speeds, acc=self.acceleration, min_time=2)
        self.robot.speedl(speeds, acc=self.acceleration, min_time=2)
        if self.cmd.btn1: #open gripper
            grippermsg = gripperOutMsg()
            grippermsg.rACT = 1
            grippermsg.rGTO = 1
            grippermsg.rATR = 0
            grippermsg.rPR = 255
            grippermsg.rSP = 255
            grippermsg.rFR = 150
            self.robotiqPub.publish(grippermsg)
        if self.cmd.btn2: #close gripper
            grippermsg = gripperOutMsg()
            grippermsg.rACT = 1
            grippermsg.rGTO = 1
            grippermsg.rATR = 0
            grippermsg.rPR = 0
            grippermsg.rSP = 255
            grippermsg.rFR = 150
            self.robotiqPub.publish(grippermsg)



if __name__ == "__main__":
    robot = urx.Robot("192.168.56.101")
    r = robot

    try:
    #start joystick service with given max speed and acceleration
        service = Service(robot, linear_velocity=0.3, rotational_velocity=0.3, acceleration=0.5)

    finally:
        robot.close()
