#include "../include/fly_shape/flyline.h"

FlyLine::FlyLine()
    : l_nh_("~"),
      in_takeoff(false),
      in_line(false),
      in_land(false),
      line_t(0.0),
      land_t(0.0),
      allline_t(5.0),
      allland_t(3.0),
      AllowError_d(0.05),
      AllowError_rad(0.15)
{
    beginpoint_ << 0.0, 0.0, 1.0;
    endpoint_   << 1.0, 0.0, 1.0;
    local_sub_  = l_nh_.subscribe("/mavros/local_position/local",100,&FlyLine::localCallBack, this);
    pos_sp_pub_ = l_nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",100);
//    att_sp_pub_ = l_nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_attitude/attitude",100);
}

void FlyLine::localCallBack(const geometry_msgs::PoseStampedConstPtr &msg)
{
    double roll, pitch, yaw;
    tf::Transform transform;
    tf::poseMsgToTF(msg->pose, transform);
    if(fabs(msg->pose.position.x)                       < AllowError_d
       && fabs(msg->pose.position.y)                    < AllowError_d
       && fabs(msg->pose.position.z - beginpoint_(2))   < AllowError_d
       && fabs(yaw)                                     < AllowError_rad
       && !in_line
       && !in_land)
    {
        in_line = true;
        ROS_INFO("Begin to fly line!!!");
    }
    if(in_line == false && in_land == false)
    {
        ROS_INFO("Taking off to beginpoint,  waiting line!!!! ");
        takeoff(beginpoint_);
    }
    if(in_line == true)
    {
        ROS_INFO("Fly line!!!");
        t_now = ros::Time::now();
        double dt = (t_now.toNSec() - t_prev.toNSec())/1e+9;
        dt = dt < 0.1 ? dt : 0;
        line_t += dt;
        line(beginpoint_, line_t, endpoint_);
        if(line_t >= allline_t)
        {
            ROS_INFO("line end!!!");
            in_land = true;
            in_line = false;
        }
    }
    if(in_land == true)
    {
        ROS_INFO("it is landing!!!");
        t_now = ros::Time::now();
        double dt = (t_now.toNSec() - t_prev.toNSec())/1e+9;
        dt = dt < 0.1 ? dt : 0;
        land_t += dt;
        land(endpoint_, land_t);
    }
    t_prev = t_now;
    publish();
}

void FlyLine::takeoff(Vector3d bp)
{
    geometry_msgs::PoseStamped takeoff_sp;
    takeoff_sp.pose.position.x = bp(0);
    takeoff_sp.pose.position.y = bp(1);
    takeoff_sp.pose.position.z = bp(2) ;
    takeoff_sp.pose.orientation.w = 1.0;
    takeoff_sp.pose.orientation.x = 0.0;
    takeoff_sp.pose.orientation.y = 0.0;
    takeoff_sp.pose.orientation.z = 0.0;

    pose_pub_ = takeoff_sp;
}

void FlyLine::land(Vector3d ep, double t)
{
    t = (t <= allland_t ? t : allland_t);
    geometry_msgs::PoseStamped land_sp;
    land_sp.pose.position.x = ep(0);
    land_sp.pose.position.y = ep(1);
    land_sp.pose.position.z = ep(2) - ep(2) / allland_t * t;
    land_sp.pose.orientation.w = 1.0;
    land_sp.pose.orientation.x = 0.0;
    land_sp.pose.orientation.y = 0.0;
    land_sp.pose.orientation.z = 0.0;

    pose_pub_ = land_sp;
}

void FlyLine::line(Vector3d bp, double t, Vector3d ep)
{
    t = (t <= allline_t ? t : allline_t);
    geometry_msgs::PoseStamped line_sp;
    Vector3d d = ep - bp;
    line_sp.pose.position.x = bp(0) + d(0) / allline_t * t;
    line_sp.pose.position.y = ep(1) + d(1) / allline_t * t;
    line_sp.pose.position.z = bp(2) + d(2) / allline_t * t;
//    line_sp.pose.orientation.w = 1.0;
//    line_sp.pose.orientation.x = 0.0;
//    line_sp.pose.orientation.y = 0.0;
//    line_sp.pose.orientation.z = 0.0;

    pose_pub_ = line_sp;
}

void FlyLine::publish()
{
    pos_sp_pub_.publish(pose_pub_);
}
