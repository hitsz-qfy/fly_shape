#!/usr/bin/env python
# coding=utf-8

import rospy
import track
from control_msgs.msg import FollowJointTrajectoryActionResult
from qfy_dynamixel.srv import *

grasp_flag = False

def result_callback(msg):
    if msg.status.status == 3:
        print("Trajectory End!!!")

def handle_service_call(req):
    global grasp_flag
    rospy.loginfo("Begin to grasp at time : %f"% req.header.stamp.toSec())
    grasp_flag = True
    return True

if __name__ == '__main__':
    rospy.init_node('track_client',anonymous=True)
    sub_trjresult = rospy.Subscriber('/m_arm_controller/follow_joint_trajectory/result', FollowJointTrajectoryActionResult, result_callback)
    grasp_srv = rospy.Service('/begin_grasp', BeginGrasp, handle_service_call)
    tracker = track.Track()
    rate = rospy.Rate(2)
    # rospy.sleep(1.0)
    while not rospy.is_shutdown():
        # tracker.kine_calcu()
        if grasp_flag:
            if tracker.get_current_Tc_cstar():
                tracker.kine_calcu()
                tracker.tf_pub()  #for test
                tracker.desire_trans()
                tracker.publish_goal()
                # if tracker.invekine():
                #     tracker.publish_goal()
                # rospy.spin()
                rate.sleep()
            else:
                rate.sleep()
                continue
        else:
            continue


