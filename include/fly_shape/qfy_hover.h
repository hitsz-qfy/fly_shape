#ifndef QFYHOVER_H
#define QFYHOVER_H

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

class QFYHover
{
public:
    QFYHover();
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
    ros::Publisher  pos_sp_pub_;
    ros::Publisher  att_sp_pub_;
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
    void takeoff(Vector3d bp);
    void loiter(Vector3d p);
    void land(Vector3d ep, double t);
    void publish();

    void joint_trajectory_callback(const control_msgs::FollowJointTrajectoryActionResult & msg);
    void mx_joint_callback(const control_msgs::FollowJointTrajectoryFeedback & msg);
};

#endif // QFYHOVER_H
