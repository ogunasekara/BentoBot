#include "ros/ros.h"
#include <eigen3/Eigen/Dense>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;

    Eigen::MatrixXd m(2,2);
    m(0,0) = 3;
    m(1,0) = 2.5;
    m(0,1) = -1;
    m(1,1) = m(1,0) + m(0,1);

    ROS_INFO_STREAM("msg: " << m << "\n");

    return 0;
}