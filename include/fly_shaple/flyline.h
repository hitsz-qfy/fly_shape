#ifndef FLYLINE_H
#define FLYLINE_H

#include "ros/ros.h"
#include "ros/time.h"
#include <geometry_msgs/PoseStamped.h>
#include "tf/transform_broadcaster.h"

#include <math.h>
#include <eigen3/Eigen/Eigen>

using namespace Eigen;

class FlyLine
{
public:
    FlyLine();
    void localCallBack(const geometry_msgs::PoseStampedConstPtr& msg);

private:
    Vector3d beginpoint_;
    Vector3d endpoint_;

    double   AllowError;
    double   line_t;
    double   allline_t;  // the time from the begin point to end point of line
    ros::Time       t_now;
    ros::Time       t_prev;
    ros::NodeHandle l_nh_;
    ros::Subscriber local_sub_;
    ros::Publisher  pos_sp_pub_;
    ros::Publisher  att_sp_pub_;

    geometry_msgs::PoseStamped pose_pub_;

    bool    in_takeoff;
    bool    in_line;
    bool    in_land;

    void takeoff(Vector3d bp);
    void land(Vector3d ep);
    void line(Vector3d bp, double t, Vector3d ep);
    void publish();
};

#endif // FLYLINE_H
