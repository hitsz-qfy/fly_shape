// begin_point and end_point need to be the same

#include "../include/fly_shape/qfy_hover.h"

QFYHover::QFYHover()
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
      check_init(false)
{
    init();
}

void QFYHover::joint_trajectory_callback(const control_msgs::FollowJointTrajectoryActionResult & msg){
//    ROS_INFO("%d", msg.status.status);
    if(msg.status.status == 3)
        joint_trj_done = true;
        ROS_INFO("Trajectory callback done!!");
}

void QFYHover::mx_joint_callback(const control_msgs::FollowJointTrajectoryFeedback & msg){
    if(!init_done){
        if(fabs(msg.actual.positions[0] - 1.57) < 0.15
                &&fabs(msg.actual.positions[1] - 1.55) < 0.15
                       &&fabs(msg.actual.positions[2] - 1.2) < 0.15){
            init_done = true;
        }
    }
    check_init = true;
}

void QFYHover::localcallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    double roll, pitch, yaw;
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->pose.orientation, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    if(fabs(msg->pose.position.x)                       < AllowError_d
       && fabs(msg->pose.position.y)                    < AllowError_d
       && fabs(msg->pose.position.z - beginpoint_(2))   < AllowError_d
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

        t_now = ros::Time::now();
        double dt = (t_now.toNSec() - t_prev.toNSec())/1e+9;
        dt = dt < 0.1 ? dt : 0;
        line_t += dt;
        if(line_t < loiter_t)
        {
            ROS_INFO_ONCE("Hovering for %f seconds!!!", loiter_t);
            if(int(line_t) != int(pre_line_t))
                ROS_INFO("%d minutes left!!", int(loiter_t + 1 - line_t));
            loiter(beginpoint_);
            pre_line_t = line_t;
        }
        else
        {
            in_land = true;
            in_line = false;
        }
    }
    if(in_land == true)
    {
        ROS_INFO_ONCE("Landing!!!");
        t_now = ros::Time::now();
        double dt = (t_now.toNSec() - t_prev.toNSec())/1e+9;
        dt = dt < 0.1 ? dt : 0;
        land_t += dt;
        land(endpoint_, land_t);
    }
    t_prev = t_now;
    publish();
}

void QFYHover::init()
{
    l_nh_.param<double>("alllint_t", allline_t, 10.0);
    l_nh_.param<double>("allland_t",allland_t, 1.5);
    l_nh_.param<double>("loiter_t",loiter_t, 10.0);

    l_nh_.param<double>("AllowError_d",AllowError_d, 0.1);
    l_nh_.param<double>("AllowError_rad",AllowError_rad, 0.15);
    l_nh_.param<double>("beginpoint/x",beginpoint_(0), 0.0);
    l_nh_.param<double>("beginpoint/y",beginpoint_(1), 0.0);
    l_nh_.param<double>("beginpoint/z",beginpoint_(2), 0.8);
    //l_nh_.param<double>("endpoint/x",endpoint_(0), 0.0);
    //l_nh_.param<double>("endpoint/y",endpoint_(1), -2.0);
    //l_nh_.param<double>("endpoint/z",endpoint_(2), 1.2);//fly along with -y axis


    l_nh_.param<double>("endpoint/x",endpoint_(0), 0.0);
    l_nh_.param<double>("endpoint/y",endpoint_(1), 0);
    l_nh_.param<double>("endpoint/z",endpoint_(2), 0.8);//fly along with +x axis

    local_sub_  = l_nh_.subscribe("/mavros/vision_pose/pose",100,&QFYHover::localcallback, this);
    pos_sp_pub_ = l_nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",100);
    client_grasp = l_nh_.serviceClient<qfy_dynamixel::BeginGrasp>("/begin_grasp");
    client_init_manipulator = l_nh_.serviceClient<qfy_dynamixel::BeginGrasp>("/init_position");
    trajectory_sub = l_nh_.subscribe("/m_arm_controller/follow_joint_trajectory/result", 100, &QFYHover::joint_trajectory_callback,this);
    mx_joint_sub = l_nh_.subscribe("/m_arm_controller/state", 100, &QFYHover::mx_joint_callback, this);
    //-------------need to add service client -------------------//

    if(l_nh_.getParam("loiter_t", loiter_t))
        ROS_WARN_ONCE("Get loiter time: %f", loiter_t);

}

void QFYHover::takeoff(Vector3d bp)
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
                return;
            }
        }
        else{
            ROS_INFO_ONCE("Init done");
            if(!first_call_init)
            {
                //-------------bug------------
                ROS_WARN_ONCE("No need to initialize!");
                ROS_INFO_ONCE("Take off!!!!!!");
                geometry_msgs::PoseStamped takeoff_sp;
                takeoff_sp.pose.position.x = bp(0);
                takeoff_sp.pose.position.y = bp(1);
                takeoff_sp.pose.position.z = bp(2) ;
                takeoff_sp.pose.orientation.x = 0.0;
                takeoff_sp.pose.orientation.y = 0.0;
                takeoff_sp.pose.orientation.z = 0.0;
                takeoff_sp.pose.orientation.w = 1.0;

                pose_pub_ = takeoff_sp;
            }
            else{
                if(joint_trj_done)
                {
                    ROS_INFO_ONCE("Take off!!!!!!");
                    geometry_msgs::PoseStamped takeoff_sp;
                    takeoff_sp.pose.position.x = bp(0);
                    takeoff_sp.pose.position.y = bp(1);
                    takeoff_sp.pose.position.z = bp(2) ;
                    takeoff_sp.pose.orientation.x = 0.0;
                    takeoff_sp.pose.orientation.y = 0.0;
                    takeoff_sp.pose.orientation.z = 0.0;
                    takeoff_sp.pose.orientation.w = 1.0;

                    pose_pub_ = takeoff_sp;
                }
            }

        }
    }
}

void QFYHover::loiter(Vector3d p)
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
    geometry_msgs::PoseStamped loiter_sp;
    loiter_sp.pose.position.x = p(0);
    loiter_sp.pose.position.y = p(1);
    loiter_sp.pose.position.z = p(2);
    loiter_sp.pose.orientation.w = 1.0;
    loiter_sp.pose.orientation.x = 0.0;
    loiter_sp.pose.orientation.y = 0.0;
    loiter_sp.pose.orientation.z = 0.0;

    pose_pub_ = loiter_sp;
}

void QFYHover::land(Vector3d ep, double t)
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

void QFYHover::publish()
{
    pos_sp_pub_.publish(pose_pub_);
}
