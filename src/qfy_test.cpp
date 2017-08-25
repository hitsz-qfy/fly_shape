#include "ros/ros.h"
#include "qfy_dynamixel/BeginGrasp.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "qfy_test_srv");
    ros::NodeHandle nh;
    ros::ServiceClient client_grasp = nh.serviceClient<qfy_dynamixel::BeginGrasp>("begin_grasp");
    ros::ServiceClient client_init = nh.serviceClient<qfy_dynamixel::BeginGrasp>("init_position");

    qfy_dynamixel::BeginGrasp srv_grasp, srv_init;
    srv_grasp.request.header.stamp = ros::Time::now();
    srv_init.request.header.stamp = ros::Time::now();

    ros::Duration wait_time = ros::Duration(0.);
    ros::Duration pre_wait_time;

    client_grasp.waitForExistence(ros::Duration(1.0));
    client_init.waitForExistence(ros::Duration(1.0));

    if(client_init.call(srv_init))
    {
        ROS_INFO("Initialize manipulator!!!");
    }
    else{
        ROS_ERROR("Failed to call /init_position");
        return 1;
    }
    ros::Time start_init =ros::Time::now();
    while(wait_time.toSec() < 5.0)
    {
        wait_time = ros::Time::now() - start_init;
        if(int(wait_time.toSec()) != int(pre_wait_time.toSec()))
            ROS_INFO("Wait for 5 minutes, %d minutes left", int(6.0 - wait_time.toSec()));
        pre_wait_time = wait_time;
    }

    ros::Time grasp_init_time = ros::Time::now();
    wait_time = ros::Duration(0.);
    pre_wait_time = ros::Duration(0.);
    while(wait_time.toSec() < 5.0)
    {
        wait_time = ros::Time::now() - grasp_init_time;
        if(int(wait_time.toSec()) != int(pre_wait_time.toSec()))
            ROS_INFO("Wait for 5 minutes, %d minutes left", int(6.0 - wait_time.toSec()));
        pre_wait_time = wait_time;
    }

    if(client_grasp.call(srv_grasp))
    {
        ROS_INFO("Begin to Grasp!!!");
    }
    else{
        ROS_ERROR("Failed to call service /begin_grasp");
        return 1;
    }

    ros::spin();
    return 0;
}
