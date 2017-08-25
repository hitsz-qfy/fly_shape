#include "ros/ros.h"
#include "ros/time.h"
#include <geometry_msgs/PoseStamped.h>
#include "tf/transform_broadcaster.h"

#include <math.h>
#include <eigen3/Eigen/Eigen>
#include "qfy_dynamixel/BeginGrasp.h"
#include "control_msgs/FollowJointTrajectoryActionResult.h"
#include "control_msgs/FollowJointTrajectoryFeedback.h"

using namespace Eigen;

class QFYTango
{
public:
    QFYTango();
    void localcallback(const geometry_msgs::PoseStampedConstPtr& msg);

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
    ros::Time       t_now;
    ros::Time       t_prev;
    ros::NodeHandle l_nh_;
    ros::Subscriber local_sub_;
    ros::Subscriber trajectory_sub;
    ros::Subscriber mx_joint_sub;
    ros::ServiceClient client_init_manipulator;
    bool first_call_init;
    ros::ServiceClient client_grasp;
    bool first_call_grasp;

    geometry_msgs::PoseStamped pose_pub_;

    qfy_dynamixel::BeginGrasp srv_grasp;
    qfy_dynamixel::BeginGrasp srv_init_pos;

    bool    in_takeoff;
    bool    in_line;
    bool    in_land;
    bool    joint_trj_done;
    bool    init_done;
    bool    check_init;

    void init();
    void takeoff();
    void loiter();

    void joint_trajectory_callback(const control_msgs::FollowJointTrajectoryActionResult & msg);
    void mx_joint_callback(const control_msgs::FollowJointTrajectoryFeedback & msg);
};

QFYTango::QFYTango()
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

void QFYTango::init()
{
    local_sub_  = l_nh_.subscribe("/mavros/vision_pose/pose",100,&QFYTango::localcallback, this);
    client_grasp = l_nh_.serviceClient<qfy_dynamixel::BeginGrasp>("/begin_grasp");
    client_init_manipulator = l_nh_.serviceClient<qfy_dynamixel::BeginGrasp>("/init_position");
    trajectory_sub = l_nh_.subscribe("/m_arm_controller/follow_joint_trajectory/result", 100, &QFYTango::joint_trajectory_callback,this);
    mx_joint_sub = l_nh_.subscribe("/m_arm_controller/state", 100, &QFYTango::mx_joint_callback, this);

}

void QFYTango::joint_trajectory_callback(const control_msgs::FollowJointTrajectoryActionResult & msg){
//    ROS_INFO("%d", msg.status.status);
    if(msg.status.status == 3)
        joint_trj_done = true;
        ROS_INFO("Trajectory callback done!!");
}

void QFYTango::mx_joint_callback(const control_msgs::FollowJointTrajectoryFeedback & msg){
    if(!init_done){
        if(fabs(msg.actual.positions[0] - 1.57) < 0.15
                &&fabs(msg.actual.positions[1] - 1.55) < 0.15
                       &&fabs(msg.actual.positions[2] - 1.2) < 0.15){
            init_done = true;
        }
    }
    check_init = true;
}


void QFYTango::localcallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    if(fabs(msg->pose.position.z - 0.7)              < 0.1
       && !in_line
       && !in_land)
    {
        in_line = true;
        ROS_INFO_ONCE("Begin to Hover!!!");
    }
    if(in_line == false && in_land == false)
    {
        ROS_INFO_ONCE("Fly to beginpoint!!!! ");
        takeoff();
    }
    if(in_line == true)
    {

        loiter();
    }

}

void QFYTango::takeoff()
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
            }
            else{
                if(joint_trj_done)
                {
                    ROS_INFO_ONCE("Take off!!!!!!");

                }
            }

        }
    }
}
    
void QFYTango::loiter()
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

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "qfy_hover_tango");
    QFYTango fl;
    ros::spin();
    return 0;
}
