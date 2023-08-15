#include "ros/ros.h"
#include "bentobot_odom/KalmanFilter.h"
#include "bentobot_mcu_bridge/MCUInfo.h"
#include "nav_msgs/Odometry.h"

#include <eigen3/Eigen/Dense>

class BentobotOdomNode 
{
public:
    BentobotOdomNode():
        nh_{},
        mcu_info_sub_{nh_.subscribe("mcu_info", 1, &BentobotOdomNode::mcu_info_callback, this)},
        odom_pub_{nh_.advertise<nav_msgs::Odometry>("odom", 5)}
    {
    }

    void mcu_info_callback(const bentobot_mcu_bridge::MCUInfo &msg)
    {
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber mcu_info_sub_;
    ros::Publisher odom_pub_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bentobot_odom");

    BentobotOdomNode node;

    // Eigen::MatrixXd m(2,2);
    // m(0,0) = 3;
    // m(1,0) = 2.5;
    // m(0,1) = -1;
    // m(1,1) = m(1,0) + m(0,1);
    // ROS_INFO_STREAM("msg: " << m << "\n");

    ros::spin();

    return 0;
}