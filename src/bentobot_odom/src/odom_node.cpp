#include <ros/ros.h>

#include <bentobot_odom/KalmanFilter.h>
#include <bentobot_mcu_bridge/MCUInfo.h>
#include <nav_msgs/Odometry.h>

#include <Eigen/Dense>

class BentobotOdomNode 
{
public:
    BentobotOdomNode(
        Eigen::Matrix<double, 5, 5> process_noise_cov,
        Eigen::Matrix<double, 2, 2> odom_noise_cov,
        Eigen::Matrix<double, 1, 1> imu_noise_cov):
        nh_{},
        mcu_info_sub_{nh_.subscribe("mcu_info", 1, &BentobotOdomNode::mcu_info_callback, this)},
        odom_pub_{nh_.advertise<nav_msgs::Odometry>("odom", 5)},
        kf_{new Bentobot::KalmanFilter(process_noise_cov, odom_noise_cov, imu_noise_cov)}
    {
    }

    void mcu_info_callback(const bentobot_mcu_bridge::MCUInfo &msg)
    {

    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber mcu_info_sub_;
    ros::Publisher odom_pub_;
    Bentobot::KalmanFilter *kf_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bentobot_odom");

    Eigen::Matrix<double, 5, 5> process_noise_cov;
    process_noise_cov.setIdentity();

    Eigen::Matrix<double, 2, 2> odom_noise_cov;
    odom_noise_cov.setIdentity();

    Eigen::Matrix<double, 1, 1> imu_noise_cov;
    imu_noise_cov.setIdentity();

    BentobotOdomNode node(process_noise_cov, odom_noise_cov, imu_noise_cov);

    ros::spin();

    return 0;
}