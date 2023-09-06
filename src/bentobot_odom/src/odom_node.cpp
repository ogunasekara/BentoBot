#include <ros/ros.h>

#include <bentobot_odom/KalmanFilter.h>
#include <bentobot_mcu_bridge/MCUInfo.h>
#include <nav_msgs/Odometry.h>

#include <Eigen/Dense>

class BentobotOdomNode 
{
private:
    ros::Subscriber mMcuInfoSub;
    ros::Publisher mOdomPub;
    Bentobot::KalmanFilter mKf;
    ros::Time mPrevTime;

    const double WHEEL_BASE_LENGTH = 0.16;

public:
    BentobotOdomNode(
        ros::NodeHandle *nh,
        Eigen::Matrix<double, 5, 5> processNoiseCov,
        Eigen::Matrix<double, 2, 2> odomNoiseCov,
        Eigen::Matrix<double, 1, 1> imuNoiseCov):
        mKf{Bentobot::KalmanFilter(processNoiseCov, odomNoiseCov, imuNoiseCov)}
    {
        nh->subscribe("mcu_info", 1, &BentobotOdomNode::mcuInfoCallback, this);
        mOdomPub = nh->advertise<nav_msgs::Odometry>("odom", 5);
        mPrevTime = ros::Time::now();
    }

    void mcuInfoCallback(const bentobot_mcu_bridge::MCUInfo &msg)
    {
        // find dt
        ros::Duration dt = msg.header.stamp - mPrevTime;
        mPrevTime = msg.header.stamp;

        // state transition update
        mKf.stateTransitionUpdate(dt.toSec());

        // find v, w
        double lin_vel = (msg.right_vel + msg.left_vel) / 2.0;
        double ang_vel = (msg.right_vel - msg.left_vel) / WHEEL_BASE_LENGTH;

        // measurement update with odom
        mKf.measurementOdomUpdate(lin_vel, ang_vel);

        // measurement update with imu
        mKf.measurementIMUUpdate(msg.imu_ang_vel);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bentobot_odom");
    ros::NodeHandle nh;

    Eigen::Matrix<double, 5, 5> process_noise_cov;
    process_noise_cov.setIdentity();

    Eigen::Matrix<double, 2, 2> odom_noise_cov;
    odom_noise_cov.setIdentity();

    Eigen::Matrix<double, 1, 1> imu_noise_cov;
    imu_noise_cov.setIdentity();

    BentobotOdomNode node(&nh, process_noise_cov, odom_noise_cov, imu_noise_cov);

    ros::spin();

    return 0;
}