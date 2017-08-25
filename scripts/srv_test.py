#!/usr/bin/env python
# coding=utf-8

import rospy
import track
from qfy_dynamixel.msg import multi_joint_point
from qfy_dynamixel.srv import *
from control_msgs.msg import FollowJointTrajectoryActionResult

grasp_flag = False
init_position = False
trajectory_done = False

def result_callback(msg):
    global trajectory_done
    if msg.status.status == 3:
        trajectory_done = True

def handle_init_position(req):
    global init_position
    rospy.loginfo("Initialize the position of manipulator at time: %f"%req.header.stamp.secs)
    init_position = True
    return True

def handle_grasp_call(req):
    global grasp_flag
    time = req.header.stamp
    rospy.loginfo("Begin to grasp at time : %f"%time.secs)
    grasp_flag = True
    return True

if __name__ == '__main__':
    rospy.init_node('track_client',anonymous=True)
    grasp_srv = rospy.Service('/begin_grasp', BeginGrasp, handle_grasp_call)
    init_position_srv = rospy.Service('/init_position', BeginGrasp , handle_init_position)
    sub_trjresult = rospy.Subscriber('m_arm_controller/follow_joint_trajectory/result', FollowJointTrajectoryActionResult, result_callback)
    pub = rospy.Publisher('/joint_goal_point', multi_joint_point, queue_size=10)


    origin = []
    joint_p = multi_joint_point()
    joint_p.id = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']
    rate = rospy.Rate(50)
    tf_cnt = 0

    time0 = rospy.Time()
    tracker = track.Track()

    while not rospy.is_shutdown():
        if init_position:
            joint_p.header.stamp = rospy.Time.now()
            joint_p.data = [1.57, 1.55, 1.2, 0.0, 0.5, 0.0, -0.8]
            pub.publish(joint_p)
            if trajectory_done:
                rospy.loginfo("Initialization Done!!")
                init_position = False
                trajectory_done = False
        # Version 1: move when hovering , but no visual servoing
        # if grasp_flag:
        #     origin.append(rospy.Time.now())
        #     now = rospy.Time.now()
        #     joint_p.header.stamp = now
        #     if now - origin[0] < rospy.Duration(5):
        #         joint_p.data = [1.57, 0.33, 0.49, 0.0, -0.22, 0.0, -0.8]
        #         rospy.loginfo_throttle(60, "Push Forward!")
        #     elif now - origin[0] < rospy.Duration(10):
        #         joint_p.data = [1.57, 1.55, 1.2, 0.0, 0.5, 0.0, -0.8]
        #         rospy.loginfo_throttle(60, "Pull Back!")
        #     else:
        #         rospy.loginfo("Grasp Done!!!")
        #         grasp_flag = False
        #
        #     pub.publish(joint_p)

        #Version 2: visual servoing
        if grasp_flag:
            origin.append(rospy.Time.now())
            now = rospy.Time.now()
            joint_p.header.stamp = now
            if now - origin[0] < rospy.Duration(5):
                joint_p.data = [1.57, 1.0, 0.5, 0.0, 0.4, 0.0, 0.0]
                pub.publish(joint_p)
                rospy.loginfo_throttle(60, "Trying to find Apriltags!!")
            else:
                if tf_cnt <= 100:
                    if tracker.listener.frameExists('/camera') and tracker.listener.frameExists('/target1'):
                        t = tracker.listener.getLatestCommonTime('/camera', '/target1')
                        if rospy.Time.now().to_sec() - t.to_sec() < 0.2:
                            (tracker.trans, tracker.rot) = tracker.listener.lookupTransform('/camera', '/target1', t)
                            tracker.kine_calcu()
                            tracker.tf_pub()
                            tracker.desire_trans()
                            if tracker.invekine():
                                tf_cnt += 1
                                rospy.logwarn_throttle(1, "Now Get target1, counter: %d" % tf_cnt)
                            else:
                                rospy.logerr_throttle(1, "Get target, but no inverse kinematic solution!!!")
                        else:
                            rospy.logerr_throttle(1, "Time difference is : %f" % (rospy.Time.now().to_sec() - t.to_sec()))
                        if tf_cnt == 90:
                            time0.from_sec(t.to_sec())
                else:
                    if tracker.get_current_Tc_cstar(time0):
                        tracker.kine_calcu()
                        tracker.tf_pub()
                        tracker.desire_trans()
                        tracker.publish_goal()
                    else:
                        rospy.logerr_throttle(0.5, "No tf get!!")
                        # tracker.publish_goal()

        rate.sleep()





