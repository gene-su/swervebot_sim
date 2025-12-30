#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class SwervebotDriver : public rclcpp::Node {
  public:
    SwervebotDriver() : Node("swervebot_driver") {
        this->declare_parameter("front_dist", 0.5);
        this->declare_parameter("rear_dist", 0.5);
        this->declare_parameter("left_dist", 0.5);
        this->declare_parameter("right_dist", 0.5);
        this->declare_parameter("wheel_radius", 0.2);

        front_dist_ = this->get_parameter("front_dist").as_double();
        rear_dist_ = this->get_parameter("rear_dist").as_double();
        left_dist_ = this->get_parameter("left_dist").as_double();
        right_dist_ = this->get_parameter("right_dist").as_double();
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&SwervebotDriver::CmdVelCallback, this,
                      std::placeholders::_1));

        joint_state_sub_ =
            this->create_subscription<sensor_msgs::msg::JointState>(
                "joint_states", 10,
                std::bind(&SwervebotDriver::JointStateCallback, this,
                          std::placeholders::_1));

        steering_pub_ =
            this->create_publisher<std_msgs::msg::Float64MultiArray>(
                "steering_controller/commands", 10);
        velocity_pub_ =
            this->create_publisher<std_msgs::msg::Float64MultiArray>(
                "velocity_controller/commands", 10);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        last_time_ = this->now();
        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;
    }

  private:
    void CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        double vx = msg->linear.x;
        double vy = msg->linear.y;
        double omega = msg->angular.z;

        std_msgs::msg::Float64MultiArray steering_msg;
        std_msgs::msg::Float64MultiArray velocity_msg;
        steering_msg.data.resize(4);
        velocity_msg.data.resize(4);

        // calc steering angles and wheel velocities for each wheel
        double fl_vx = vx - omega * left_dist_;
        double fl_vy = vy + omega * front_dist_;
        steering_msg.data[0] = std::atan2(fl_vy, fl_vx);
        velocity_msg.data[0] =
            std::sqrt(fl_vx * fl_vx + fl_vy * fl_vy) / wheel_radius_;

        double fr_vx = vx + omega * right_dist_;
        double fr_vy = vy + omega * front_dist_;
        steering_msg.data[1] = std::atan2(fr_vy, fr_vx);
        velocity_msg.data[1] =
            std::sqrt(fr_vx * fr_vx + fr_vy * fr_vy) / wheel_radius_;

        double rl_vx = vx - omega * left_dist_;
        double rl_vy = vy - omega * rear_dist_;
        steering_msg.data[2] = std::atan2(rl_vy, rl_vx);
        velocity_msg.data[2] =
            std::sqrt(rl_vx * rl_vx + rl_vy * rl_vy) / wheel_radius_;

        double rr_vx = vx + omega * right_dist_;
        double rr_vy = vy - omega * rear_dist_;
        steering_msg.data[3] = std::atan2(rr_vy, rr_vx);
        velocity_msg.data[3] =
            std::sqrt(rr_vx * rr_vx + rr_vy * rr_vy) / wheel_radius_;

        // normalize steering angles to [-pi/2, pi/2] and flip velocity
        for (size_t i = 0; i < 4; i++) {
            if (steering_msg.data[i] > M_PI_2) {
                steering_msg.data[i] -= M_PI;
                velocity_msg.data[i] = -velocity_msg.data[i];
            } else if (steering_msg.data[i] < -M_PI_2) {
                steering_msg.data[i] += M_PI;
                velocity_msg.data[i] = -velocity_msg.data[i];
            }
        }

        steering_pub_->publish(steering_msg);
        velocity_pub_->publish(velocity_msg);
    }

    void JointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        auto current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        auto find_index = [&](const std::string& name) -> int {
            auto it = std::find(msg->name.begin(), msg->name.end(), name);
            return it != msg->name.end() ? std::distance(msg->name.begin(), it)
                                         : -1;
        };

        // get wheel vels and steering angles
        double fl_vel = msg->velocity[find_index("front_left_wheel_joint")];
        double fr_vel = msg->velocity[find_index("front_right_wheel_joint")];
        double rl_vel = msg->velocity[find_index("rear_left_wheel_joint")];
        double rr_vel = msg->velocity[find_index("rear_right_wheel_joint")];
        double fl_angle =
            msg->position[find_index("front_left_steering_joint")];
        double fr_angle =
            msg->position[find_index("front_right_steering_joint")];
        double rl_angle = msg->position[find_index("rear_left_steering_joint")];
        double rr_angle =
            msg->position[find_index("rear_right_steering_joint")];

        // calc vx, vy
        double fl_vx = fl_vel * wheel_radius_ * std::cos(fl_angle);
        double fl_vy = fl_vel * wheel_radius_ * std::sin(fl_angle);
        double fr_vx = fr_vel * wheel_radius_ * std::cos(fr_angle);
        double fr_vy = fr_vel * wheel_radius_ * std::sin(fr_angle);
        double rl_vx = rl_vel * wheel_radius_ * std::cos(rl_angle);
        double rl_vy = rl_vel * wheel_radius_ * std::sin(rl_angle);
        double rr_vx = rr_vel * wheel_radius_ * std::cos(rr_angle);
        double rr_vy = rr_vel * wheel_radius_ * std::sin(rr_angle);

        double vx = (fl_vx + fr_vx + rl_vx + rr_vx) / 4.0;
        double vy = (fl_vy + fr_vy + rl_vy + rr_vy) / 4.0;

        // calc omega
        double fl_x = front_dist_;
        double fl_y = left_dist_;
        double fr_x = front_dist_;
        double fr_y = -right_dist_;
        double rl_x = -rear_dist_;
        double rl_y = left_dist_;
        double rr_x = -rear_dist_;
        double rr_y = -right_dist_;

        double omega_numerator =
            (fl_vy * fl_x - fl_vx * fl_y) + (fr_vy * fr_x - fr_vx * fr_y) +
            (rl_vy * rl_x - rl_vx * rl_y) + (rr_vy * rr_x - rr_vx * rr_y);
        double omega_denominator =
            (fl_x * fl_x + fl_y * fl_y) + (fr_x * fr_x + fr_y * fr_y) +
            (rl_x * rl_x + rl_y * rl_y) + (rr_x * rr_x + rr_y * rr_y);

        double omega = omega_numerator / omega_denominator;

        // calc odom
        double delta_x = (vx * std::cos(theta_) - vy * std::sin(theta_)) * dt;
        double delta_y = (vx * std::sin(theta_) + vy * std::cos(theta_)) * dt;
        double delta_theta = omega * dt;
        x_ += delta_x;
        y_ += delta_y;
        theta_ += delta_theta;

        // publish odom
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_footprint";
        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom_msg.pose.pose.orientation = tf2::toMsg(q);
        odom_msg.twist.twist.linear.x = vx;
        odom_msg.twist.twist.linear.y = vy;
        odom_msg.twist.twist.angular.z = omega;
        odom_pub_->publish(odom_msg);

        // publish tf
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = current_time;
        tf.header.frame_id = "odom";
        tf.child_frame_id = "base_footprint";
        tf.transform.translation.x = x_;
        tf.transform.translation.y = y_;
        tf.transform.translation.z = 0.0;
        tf.transform.rotation = tf2::toMsg(q);
        tf_broadcaster_->sendTransform(tf);
    }

    double front_dist_;
    double rear_dist_;
    double left_dist_;
    double right_dist_;
    double wheel_radius_;

    rclcpp::Time last_time_;
    double x_, y_, theta_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
        joint_state_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
        steering_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
        velocity_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SwervebotDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
