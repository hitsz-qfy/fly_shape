#include "ros/ros.h"
#include "ros/time.h"
#include <geometry_msgs/PoseStamped.h>
#include "tf/transform_broadcaster.h"
#include "std_msgs/Bool.h"

#include <math.h>
#include <eigen3/Eigen/Eigen>
#include "qfy_dynamixel/BeginGrasp.h"
#include "qfy_dynamixel/MoveUAV.h"

#include "control_msgs/FollowJointTrajectoryActionResult.h"
#include "control_msgs/FollowJointTrajectoryFeedback.h"

using namespace Eigen;

class QFYUavtrack
{
public:
    QFYUavtrack();


private:
    Vector3d beginpoint_;
    Vector3d endpoint_;

    double   AllowError_d;
    double   AllowError_rad;
    double   line_t;
    double   pre_line_t;
    double   land_t;
    double   loiter_t;
    double   allline_t;  // the time from the begin point to end point of line
    double   allland_t;
    double   lambda;
    ros::Time       t_now;
    ros::Time       t_prev;
    ros::Duration   diff_time;
    ros::NodeHandle l_nh_;
    ros::Publisher  pos_sp_pub_;
    ros::Subscriber local_sub_;
    ros::Subscriber trajectory_sub;
    ros::Subscriber mx_joint_sub;
    ros::Subscriber uav_move_sub;
    ros::Subscriber land_flag_sub;
    ros::Subscriber check_join_sub;

    ros::ServiceClient client_init_manipulator;
    bool first_call_init;
    ros::ServiceClient client_grasp;
    bool first_call_grasp;

    geometry_msgs::PoseStamped pose_pub_;
    geometry_msgs::PoseStamped local_pose;
    geometry_msgs::PoseStamped uav_setpose;
    geometry_msgs::PoseStamped lowpass_pre;

    qfy_dynamixel::BeginGrasp srv_grasp;
    qfy_dynamixel::BeginGrasp srv_init_pos;

    bool in_takeoff;
    bool in_line;
    bool in_land;
    bool joint_trj_done;
    bool init_done;
    bool check_init;
    bool land_flag;

    void init();
    void takeoff(const Vector3d);
    void uav_track(const Vector3d &);
    void land(const geometry_msgs::PoseStamped, double);
    void publish();

    void localcallback(const geometry_msgs::PoseStampedConstPtr& msg);
    void joint_trajectory_callback(const control_msgs::FollowJointTrajectoryActionResult & msg);
    void mx_joint_callback(const control_msgs::FollowJointTrajectoryFeedback & msg);
    void uav_move_callback(const geometry_msgs::PoseStampedConstPtr &);
    void land_signal_callback(const std_msgs::Bool &);
    void check_joint_initialization(const std_msgs::Bool &);

    void low_pass(geometry_msgs::PoseStamped);
};

QFYUavtrack::QFYUavtrack()
    : l_nh_("~"),
      in_takeoff(false),
      in_line(false),
      in_land(false),
      line_t(0.0),
      pre_line_t(0.),
      land_t(0.0),
      joint_trj_done(false),
      first_call_init(false),
      first_call_grasp(false),
      init_done(false),
      check_init(false),
      land_flag(false),
      lambda(0.1)
{
    init();
}

void QFYUavtrack::init()
{
    l_nh_.param<double>("alllint_t", allline_t, 10.0);
    l_nh_.param<double>("allland_t",allland_t, 1.5);
    l_nh_.param<double>("loiter_t",loiter_t, 10.0);

    l_nh_.param<double>("AllowError_d",AllowError_d, 0.1);
    l_nh_.param<double>("AllowError_rad",AllowError_rad, 0.15);
    l_nh_.param<double>("beginpoint/x",beginpoint_(0), 0.0);
    l_nh_.param<double>("beginpoint/y",beginpoint_(1), 0.0);
    l_nh_.param<double>("beginpoint/z",beginpoint_(2), 0.8);


    local_sub_  = l_nh_.subscribe("/mavros/vision_pose/pose",100,&QFYUavtrack::localcallback, this);
    pos_sp_pub_ = l_nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",100);
    client_grasp = l_nh_.serviceClient<qfy_dynamixel::BeginGrasp>("/begin_grasp");
    client_init_manipulator = l_nh_.serviceClient<qfy_dynamixel::BeginGrasp>("/init_position");
//    trajectory_sub = l_nh_.subscribe("/m_arm_controller/follow_joint_trajectory/result", 100, &QFYUavtrack::joint_trajectory_callback,this);
    mx_joint_sub = l_nh_.subscribe("/m_arm_controller/state", 100, &QFYUavtrack::mx_joint_callback, this);
    uav_move_sub = l_nh_.subscribe("/uav_move", 100, &QFYUavtrack::uav_move_callback, this);
    land_flag_sub = l_nh_.subscribe("/land_signal", 100, &QFYUavtrack::land_signal_callback, this);
    check_join_sub = l_nh_.subscribe("/check_joint_trj", 100, &QFYUavtrack::check_joint_initialization, this);

    t_now = ros::Time::now();
    t_prev = t_now;

    lowpass_pre.pose.position.x = beginpoint_(0);
    lowpass_pre.pose.position.y = beginpoint_(1);
    lowpass_pre.pose.position.z = beginpoint_(2);
    lowpass_pre.pose.orientation.x = 0.;
    lowpass_pre.pose.orientation.y = 0.;
    lowpass_pre.pose.orientation.z = 0.;
    lowpass_pre.pose.orientation.w = 1.;
}

void QFYUavtrack::land_signal_callback(const std_msgs::Bool &msg)
{
    if(msg.data)
    {
        land_flag = true;
    }
    else
        land_flag = false;
}

void QFYUavtrack::check_joint_initialization(const std_msgs::Bool & msg)
{
    if(msg.data)
    {
        joint_trj_done = true;
        ROS_INFO("Trajectory callback done!!");
    }
    else
        joint_trj_done = false;

}

/*
//void QFYUavtrack::joint_trajectory_callback(const control_msgs::FollowJointTrajectoryActionResult & msg){
////    ROS_INFO("%d", msg.status.status);
//    if(msg.status.status == 3)
//        joint_trj_done = true;
//        ROS_INFO("Trajectory callback done!!");
//}
*/

void QFYUavtrack::mx_joint_callback(const control_msgs::FollowJointTrajectoryFeedback & msg)
{
    if(!init_done){
        if(fabs(msg.actual.positions[0] - 1.57) < 0.15
                &&fabs(msg.actual.positions[1] - 1.55) < 0.15
                       &&fabs(msg.actual.positions[2] - 1.2) < 0.15){
            init_done = true;
        }
    }
    check_init = true;
}

void QFYUavtrack::uav_move_callback(const geometry_msgs::PoseStampedConstPtr & msg)
{
    uav_setpose.header.stamp = msg->header.stamp;
    uav_setpose.pose.position.x = msg->pose.position.x;
    uav_setpose.pose.position.y = msg->pose.position.y;
    uav_setpose.pose.position.z = msg->pose.position.z;
    uav_setpose.pose.orientation.x = msg->pose.orientation.x;
    uav_setpose.pose.orientation.y = msg->pose.orientation.y;
    uav_setpose.pose.orientation.z = msg->pose.orientation.z;
    uav_setpose.pose.orientation.w = msg->pose.orientation.w;
}


void QFYUavtrack::localcallback(const geometry_msgs::PoseStampedConstPtr & msg)
{
    t_now = ros::Time::now();

    local_pose.header.stamp = msg->header.stamp;
    local_pose.pose.position.x = msg->pose.position.x;
    local_pose.pose.position.y = msg->pose.position.y;
    local_pose.pose.position.z = msg->pose.position.z;
    local_pose.pose.orientation.x = msg->pose.orientation.x;
    local_pose.pose.orientation.y = msg->pose.orientation.y;
    local_pose.pose.orientation.z = msg->pose.orientation.z;
    local_pose.pose.orientation.w = msg->pose.orientation.w;

    diff_time = t_now - t_prev;

    if(fabs(msg->pose.position.z - 0.75) < 0.1
       && !in_line
       && !in_land)
    {
        in_line = true;
        ROS_INFO_ONCE("Begin to Hover!!!");
    }
    if(in_line == false && in_land == false)
    {
        ROS_INFO_ONCE("Fly to beginpoint!!!! ");
        takeoff(beginpoint_);
    }
    if(in_line == true)
    {
        uav_track(beginpoint_);
    }
    if(in_land == true)
    {
        land_t += diff_time.toSec();
        land(local_pose, land_t);
    }

    t_prev = t_now;
    publish();
}

void QFYUavtrack::takeoff(const Vector3d bp)
{

    if(check_init){
        if(!init_done){
            ROS_INFO_ONCE("Init not done");
            if(!joint_trj_done){
                if(!first_call_init){
                    client_init_manipulator.waitForExistence(ros::Duration(1.0));
                    srv_init_pos.request.header.stamp = ros::Time::now();
                    if(client_init_manipulator.call(srv_init_pos))
                    {
                        ROS_INFO_ONCE("Initialize manipulator!!");
                        first_call_init = true;
                    }
                    else
                    {
                        ROS_ERROR("Failed to call /init_position");
                    }
                }
            }
        }
        else{
            ROS_INFO_ONCE("Init done");
            if(!first_call_init)
            {
                //BUG:
                //
                ROS_WARN_ONCE("No need to initialize!");
                ROS_INFO_ONCE("Take off!!!!!!");
            }
            else{
                if(joint_trj_done)
                {
                    ROS_INFO_ONCE("Take off!!!!!!");
                }
            }
            pose_pub_.pose.position.x = bp(0);
            pose_pub_.pose.position.y = bp(1);
            pose_pub_.pose.position.z = bp(2);

            pose_pub_.pose.orientation.x = 0.;
            pose_pub_.pose.orientation.y = 0.;
            pose_pub_.pose.orientation.z = 0.;
            pose_pub_.pose.orientation.w = 1.;

        }
    }
}

void QFYUavtrack::low_pass(geometry_msgs::PoseStamped p)
{
    p.pose.position.x = p.pose.position.x * lambda
                                    + lowpass_pre.pose.position.x * ( 1 - lambda);
    p.pose.position.y = p.pose.position.y * lambda
                                    + lowpass_pre.pose.position.y * ( 1 - lambda);
    p.pose.position.z = p.pose.position.z * lambda
                                    + lowpass_pre.pose.position.z * ( 1 - lambda);
    p.pose.orientation.x = p.pose.orientation.x * lambda
                                    + lowpass_pre.pose.orientation.x * ( 1 - lambda);
    p.pose.orientation.y = p.pose.orientation.y * lambda
                                    + lowpass_pre.pose.orientation.y * ( 1 - lambda);
    p.pose.orientation.z = p.pose.orientation.z * lambda
                                    + lowpass_pre.pose.orientation.z * ( 1 - lambda);
    p.pose.orientation.w = p.pose.orientation.w * lambda
                                    + lowpass_pre.pose.orientation.w * ( 1 - lambda);


    lowpass_pre.pose.position.x = p.pose.position.x;
    lowpass_pre.pose.position.y = p.pose.position.y;
    lowpass_pre.pose.position.z = p.pose.position.z;
    lowpass_pre.pose.orientation.x = p.pose.orientation.x;
    lowpass_pre.pose.orientation.y = p.pose.orientation.y;
    lowpass_pre.pose.orientation.z = p.pose.orientation.z;
    lowpass_pre.pose.orientation.w = p.pose.orientation.w;

}
    
void QFYUavtrack::uav_track(const Vector3d & bp)
{

    if(!first_call_grasp){
        client_grasp.waitForExistence(ros::Duration(1.0));
        srv_grasp.request.header.stamp = ros::Time::now();
        if(client_grasp.call(srv_grasp))
        {
            ROS_INFO_ONCE("Begin to grasp");
            first_call_grasp = true;
        }
    }
    //low-pass filter target pose
    /*
    low_pass(uav_setpose);

    pose_pub_.pose.position.x = uav_setpose.pose.position.x;
    pose_pub_.pose.position.y = uav_setpose.pose.position.y;
    pose_pub_.pose.position.z = uav_setpose.pose.position.z;
    pose_pub_.pose.orientation.x = uav_setpose.pose.orientation.x;
    pose_pub_.pose.orientation.y = uav_setpose.pose.orientation.y;
    pose_pub_.pose.orientation.z = uav_setpose.pose.orientation.z;
    pose_pub_.pose.orientation.w = uav_setpose.pose.orientation.w;
    */
    pose_pub_.pose.position.x = bp(0);
    pose_pub_.pose.position.y = bp(1);
    pose_pub_.pose.position.z = bp(2);

    pose_pub_.pose.orientation.x = 0.;
    pose_pub_.pose.orientation.y = 0.;
    pose_pub_.pose.orientation.z = 0.;
    pose_pub_.pose.orientation.w = 1.;
    //TODO:
    //need interpolation poses

    //TODO:
    //get landing signal and set in_land to true and set in_line to false.
    if(land_flag)
    {
        in_land = true;
        in_line = false;
    }

}

void QFYUavtrack::land(const geometry_msgs::PoseStamped ep, double t)
{
    t = ( t < allland_t ? t: allland_t);

    pose_pub_.pose.position.x = ep.pose.position.x;
    pose_pub_.pose.position.y = ep.pose.position.y;
    pose_pub_.pose.position.z = ep.pose.position.z * ( 1. - 1. / allland_t * t);

    pose_pub_.pose.orientation.x = ep.pose.orientation.x;
    pose_pub_.pose.orientation.y = ep.pose.orientation.y;
    pose_pub_.pose.orientation.z = ep.pose.orientation.z;
    pose_pub_.pose.orientation.w = ep.pose.orientation.w;

}

void QFYUavtrack::publish()
{
    pos_sp_pub_.publish(pose_pub_);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "qfy_uav_track");
    QFYUavtrack fl;
    ros::spin();
    return 0;
}
