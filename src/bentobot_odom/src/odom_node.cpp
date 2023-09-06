#include <ros/ros.h>

#include <bentobot_odom/KalmanFilter.h>
#include <bentobot_mcu_bridge/MCUInfo.h>
#include <nav_msgs/Odometry.h>

#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>

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
        mMcuInfoSub = nh->subscribe("/mcu_info", 1, &BentobotOdomNode::mcuInfoCallback, this);
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

        // publish odom message
        Eigen::Matrix<double, 5, 1> state = mKf.getState();
        tf2::Quaternion quat;
        quat.setRPY(0, 0, state(2));
        quat.normalize();

        nav_msgs::Odometry odom_msg;
        odom_msg.pose.pose.position.x = state(0);
        odom_msg.pose.pose.position.y = state(1);
        odom_msg.pose.pose.orientation.x = quat.getX();
        odom_msg.pose.pose.orientation.y = quat.getY();
        odom_msg.pose.pose.orientation.z = quat.getZ();
        odom_msg.pose.pose.orientation.w = quat.getW();

        mOdomPub.publish(odom_msg);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bentobot_odom");
    ros::NodeHandle nh;

    Eigen::Matrix<double, 5, 5> process_noise_cov;
    process_noise_cov.setIdentity();
    process_noise_cov(0, 0) = 0.01;
    process_noise_cov(1, 1) = 0.01;
    process_noise_cov(2, 2) = 0.09;
    process_noise_cov(3, 3) = 0.01;
    process_noise_cov(4, 4) = 0.09;

    Eigen::Matrix<double, 2, 2> odom_noise_cov;
    odom_noise_cov.setIdentity();
    odom_noise_cov(0, 0) = 0.0001;
    odom_noise_cov(1, 1) = 0.0081;

    Eigen::Matrix<double, 1, 1> imu_noise_cov;
    imu_noise_cov.setIdentity();
    imu_noise_cov(0, 0) = 0.0005;

    BentobotOdomNode node(&nh, process_noise_cov, odom_noise_cov, imu_noise_cov);

    ros::spin();

    return 0;
}