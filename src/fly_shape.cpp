#include "ros/ros.h"
#include "../include/fly_shaple/flyline.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fly_shape");
    FlyLine fl;
    ros::spin();
    return 0;
}
