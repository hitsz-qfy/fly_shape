#!/usr/bin/env python
# coding=utf-8
import rospy
from  math import *
import numpy as np
from qfy_dynamixel.msg import multi_joint_point
from dynamixel_msgs.msg import JointState

from geometry_msgs.msg import TransformStamped
import tf

class Kinematic(object):
    def __init__(self,l2,l3,l4,l6):
        self.joint_id=['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7']
        # self.joint_value=[0.,0.,0.,0.,0.,0.,np.pi/2.]
        self.joint_value = [0., 0., 0., 0., 0., 0., 0.]
        self.kinematic= np.mat(np.zeros((4,4)))
        self.l2, self.l3 , self.l4, self.l6= l2, l3, l4, l6
        # self.joint3_tmp = 0.

        self.sub1 = rospy.Subscriber('/joint1_controller/state', JointState, self.joint1_callback)
        self.sub2 = rospy.Subscriber('/joint2_controller/state', JointState, self.joint2_callback)
        self.sub3 = rospy.Subscriber('/joint3_controller/state', JointState, self.joint3_callback)
        self.sub4 = rospy.Subscriber('/joint4_controller/state', JointState, self.joint4_callback)
        self.sub5 = rospy.Subscriber('/joint5_controller/state', JointState, self.joint5_callback)
        self.sub6 = rospy.Subscriber('/joint6_controller/state', JointState, self.joint6_callback)
        self.sub7 = rospy.Subscriber('/joint7_controller/state', JointState, self.joint7_callback)
        # self.pub = rospy.Publisher('joint_current_kinematic',TransformStamped,queue_size=10)

    def joint1_callback(self,msg):
        # print("joint1_state: %s"%msg.current_pos)
        #true value (pi/2 ============== 0)
        # self.joint_value[0]=msg.current_pos - np.pi/2.
        self.joint_value[0] = np.pi/2. - msg.current_pos

    def joint2_callback(self,msg):
        # print("joint2_state: %s" % msg.current_pos)
        # self.joint_value[1]= 4.71 - msg.current_pos
        #true value (1.57-4.71 ============== -1.57-1.57)
        #self.joint_value[1] = msg.current_pos - np.pi #origin
        self.joint_value[1] = - msg.current_pos

    def joint3_callback(self,msg):
        # print("joint3_state: %s" % msg.current_pos)
        # self.joint_value[2]=msg.current_pos - 3.1415926
        #true value (1.57-4.71 ============== 3.14-0)
        # self.joint_value[2] = np.pi*1.5 - msg.current_pos #origin
        self.joint_value[2] = np.pi/2. + msg.current_pos
        self.tmp_3 = msg.current_pos
        # print("joint 3 value: %s"%msg.current_pos)
        # print("joint_value[2] value: %s"%self.joint_value[2])

    def joint4_callback(self,msg):
        # print("joint4_state: %s" % msg.current_pos)
        self.joint_value[3]=msg.current_pos

    def joint5_callback(self,msg):
        # print("joint5_state: %s" % msg.current_pos)
        # self.joint_value[4]=msg.current_pos + np.pi/2
        self.joint_value[4] = msg.current_pos

    def joint6_callback(self,msg):
        # print("joint6_state: %s" % msg.current_pos)
        self.joint_value[5]=msg.current_pos

    def joint7_callback(self,msg):
        # print("joint6_state: %s" % msg.current_pos)
        self.joint_value[6]=msg.current_pos

    def cur_joint(self):
    #     return [self.joint_value[0]+np.pi/2, -self.joint_value[1], self.tmp_3,  self.joint_value[3],
    #             self.joint_value[4], self.joint_value[5], self.joint_value[6]]
        return [np.pi/2.-self.joint_value[0], -self.joint_value[1], self.tmp_3,  self.joint_value[3],
                self.joint_value[4], self.joint_value[5], self.joint_value[6]]

    def kinematic_(self, joint_value):
        joint_value[2] += np.pi/2

        T01_ = np.mat([[cos(joint_value[0]), 0, sin(joint_value[0]), -self.l2 * cos(joint_value[0])],
                      [sin(joint_value[0]), 0, -cos(joint_value[0]), -self.l2 * sin(joint_value[0])],
                      [0, 1, 0, 0],
                      [0, 0, 0, 1]
                      ])

        T12_ = np.mat([[cos(joint_value[1]), -sin(joint_value[1]), 0, self.l3 * cos(joint_value[1])],
                      [sin(joint_value[1]), cos(joint_value[1]), 0, self.l3 * sin(joint_value[1])],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]
                      ])

        T23_ = np.mat([[cos(joint_value[2]), 0, sin(joint_value[2]), 0],
                      [sin(joint_value[2]), 0, -cos(joint_value[2]), 0],
                      [0, 1, 0, 0],
                      [0, 0, 0, 1]
                      ])

        T34_ = np.mat([[cos(joint_value[3]), 0, -sin(joint_value[3]), 0],
                      [sin(joint_value[3]), 0, cos(joint_value[3]), 0],
                      [0, -1, 0, self.l4],
                      [0, 0, 0, 1]
                      ])

        T45_ = np.mat([[cos(joint_value[4]), 0, sin(joint_value[4]), 0],
                      [sin(joint_value[4]), 0, -cos(joint_value[4]), 0],
                      [0, 1, 0, 0],
                      [0, 0, 0, 1]
                      ])

        T56_ = np.mat([[cos(joint_value[5]), -sin(joint_value[5]), 0, 0],
                      [sin(joint_value[5]), cos(joint_value[5]), 0, 0],
                      [0, 0, 1, self.l6],
                      [0, 0, 0, 1]
                      ])
        return T01_*T12_*T23_*T34_*T45_*T56_


    def kinematic_calcu(self):
        self.joint_value[2] += np.pi/2
        # print("kinematic : %s"%self.joint_value)

        T01 = np.mat([[cos(self.joint_value[0]), 0,  sin(self.joint_value[0]), -self.l2*cos(self.joint_value[0])],
                      [sin(self.joint_value[0]), 0, -cos(self.joint_value[0]), -self.l2*sin(self.joint_value[0])],
                      [0, 1, 0, 0],
                      [0, 0, 0, 1]
                      ])

        T12 = np.mat([[cos(self.joint_value[1]), -sin(self.joint_value[1]), 0,  self.l3*cos(self.joint_value[1])],
                      [sin(self.joint_value[1]), cos(self.joint_value[1]),  0,  self.l3*sin(self.joint_value[1])],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]
                      ])

        T23 = np.mat([[cos(self.joint_value[2]), 0, sin(self.joint_value[2]),  0],
                      [sin(self.joint_value[2]), 0, -cos(self.joint_value[2]), 0],
                      [0, 1, 0, 0],
                      [0, 0, 0, 1]
                      ])

        T34 = np.mat([[cos(self.joint_value[3]), 0, -sin(self.joint_value[3]), 0],
                      [sin(self.joint_value[3]), 0, cos(self.joint_value[3]), 0],
                      [0, -1, 0, self.l4],
                      [0, 0, 0, 1]
                      ])

        T45 = np.mat([[cos(self.joint_value[4]), 0, sin(self.joint_value[4]), 0],
                      [sin(self.joint_value[4]), 0, -cos(self.joint_value[4]), 0],
                      [0, 1, 0, 0],
                      [0, 0, 0, 1]
                      ])

        T56 = np.mat([[cos(self.joint_value[5]), -sin(self.joint_value[5]), 0, 0],
                      [sin(self.joint_value[5]), cos(self.joint_value[5]), 0, 0],
                      [0, 0, 1, self.l6],
                      [0, 0, 0, 1]
                      ])

        self.kinematic = T01*T12*T23*T34*T45*T56
        # print self.kinematic
        return self.kinematic