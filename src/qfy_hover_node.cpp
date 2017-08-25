#include "ros/ros.h"
#include "../include/fly_shape/qfy_hover.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "qfy_hover");
    QFYHover fl;
    ros::spin();
    return 0;
}
