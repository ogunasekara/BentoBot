#include <rclcpp/rclcpp.hpp>

#include <bentobot_odom/KalmanFilter.h>
#include <bentobot_interfaces/msg/mcu_info.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <Eigen/Dense>
#include <memory>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

class BentobotOdomNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<bentobot_interfaces::msg::MCUInfo>::SharedPtr mMcuInfoSub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr mOdomPub;
    std::unique_ptr<tf2_ros::TransformBroadcaster> mTfBroadcaster;

    std::unique_ptr<Bentobot::KalmanFilter> mKf;
    rclcpp::Time mPrevTime;

    const double WHEEL_BASE_LENGTH = 0.16;

public:
    BentobotOdomNode(): Node("bentobot_odom")
    {
        double pn_x = declare_parameter("process_noise_x", 0.01);
        double pn_y = declare_parameter("process_noise_y", 0.01);
        double pn_th = declare_parameter("process_noise_th", 0.09);
        double pn_v = declare_parameter("process_noise_v", 0.01);
        double pn_w = declare_parameter("process_noise_w", 0.09);
        double ms_odom_v = declare_parameter("measurement_noise_odom_v", 0.0001);
        double ms_odom_w = declare_parameter("measurement_noise_odom_w", 0.0081);
        double ms_imu_w = declare_parameter("measurement_noise_imu_w", 0.0005);

        Eigen::Matrix<double, 5, 5> process_noise_cov;
        process_noise_cov.setIdentity();
        process_noise_cov(0, 0) = pn_x;
        process_noise_cov(1, 1) = pn_y;
        process_noise_cov(2, 2) = pn_th;
        process_noise_cov(3, 3) = pn_v;
        process_noise_cov(4, 4) = pn_w;

        Eigen::Matrix<double, 2, 2> odom_noise_cov;
        odom_noise_cov.setIdentity();
        odom_noise_cov(0, 0) = ms_odom_v;
        odom_noise_cov(1, 1) = ms_odom_w;

        Eigen::Matrix<double, 1, 1> imu_noise_cov;
        imu_noise_cov.setIdentity();
        imu_noise_cov(0, 0) = ms_imu_w;

        mKf = std::make_unique<Bentobot::KalmanFilter>(process_noise_cov, odom_noise_cov, imu_noise_cov);

        mMcuInfoSub = create_subscription<bentobot_interfaces::msg::MCUInfo>(
            "/mcu_info", 1, std::bind(&BentobotOdomNode::mcuInfoCallback, this, std::placeholders::_1));
        mOdomPub = create_publisher<nav_msgs::msg::Odometry>("odom", 5);
        mTfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        mPrevTime = now();
    }

    void mcuInfoCallback(const bentobot_interfaces::msg::MCUInfo::SharedPtr msg)
    {
        // find dt
        rclcpp::Time stamp(msg->header.stamp, mPrevTime.get_clock_type());
        rclcpp::Duration dt = stamp - mPrevTime;
        mPrevTime = stamp;

        // state transition update
        mKf->stateTransitionUpdate(dt.seconds());

        // find v, w
        double lin_vel = (msg->right_vel + msg->left_vel) / 2.0;
        double ang_vel = (msg->right_vel - msg->left_vel) / WHEEL_BASE_LENGTH;
        double imu_ang_vel = msg->imu_ang_vel;

        // measurement update with odom
        mKf->measurementOdomUpdate(lin_vel, ang_vel);

        // measurement update with imu
        mKf->measurementIMUUpdate(imu_ang_vel);

        // publish odom message
        Eigen::Matrix<double, 5, 1> state = mKf->getState();

        publishOdom(state, stamp);
        publishTf(state);
    }
private:
    void publishOdom(Eigen::Matrix<double, 5, 1> state, const rclcpp::Time &stamp)
    {
        tf2::Quaternion quat;
        quat.setRPY(0, 0, state(2));
        quat.normalize();

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = stamp;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
        odom_msg.pose.pose.position.x = state(0);
        odom_msg.pose.pose.position.y = state(1);
        odom_msg.pose.pose.orientation.x = quat.getX();
        odom_msg.pose.pose.orientation.y = quat.getY();
        odom_msg.pose.pose.orientation.z = quat.getZ();
        odom_msg.pose.pose.orientation.w = quat.getW();

        mOdomPub->publish(odom_msg);
    }

    void publishTf(Eigen::Matrix<double, 5, 1> state)
    {
        geometry_msgs::msg::TransformStamped tf;

        tf.header.stamp = now();
        tf.header.frame_id = "odom";
        tf.child_frame_id = "base_link";

        tf.transform.translation.x = state(0);
        tf.transform.translation.y = state(1);
        tf.transform.translation.z = 0;

        tf2::Quaternion q;
        q.setRPY(0, 0, state(2));

        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf.transform.rotation.w = q.w();

        mTfBroadcaster->sendTransform(tf);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BentobotOdomNode>());
    rclcpp::shutdown();

    return 0;
}
