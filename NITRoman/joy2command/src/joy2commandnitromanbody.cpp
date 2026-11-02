#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <ps5_msgs/msg/ps5_joy.hpp>
#include <std_msgs/msg/bool.hpp>


class Joy2Commandnitromanbody : public rclcpp::Node {
public:
    Joy2Commandnitromanbody() : Node("ps5_to_ros_command") {
        loadParameters();
        initPublishers();
        ps5_sub_ = this->create_subscription<ps5_msgs::msg::PS5Joy>(
            "/ps5_joy", 1, std::bind(&Joy2Commandnitromanbody::ps5Callback, this, std::placeholders::_1));

    }

    void initPublishers() {
        body_twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/body_twist", 1);
        head_twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/head_twist", 1);
    }
    
    void loadParameters() {
        // Load parameters for linear and angular movements

        this->declare_parameter("body.max_linear_velocity", 0.1);
        this->declare_parameter("body.max_angular_velocity", 0.1);

        this->get_parameter("body.max_linear_velocity", body_max_linear_velocity_);
        this->get_parameter("body.max_angular_velocity", body_max_angular_velocity_);

        this->declare_parameter("head.max_linear_velocity", 0.1);
        this->declare_parameter("head.max_angular_velocity", 0.1);

        this->get_parameter("head.max_linear_velocity", head_max_linear_velocity_);
        this->get_parameter("head.max_angular_velocity", head_max_angular_velocity_);

    }


    void ps5Callback(const ps5_msgs::msg::PS5Joy::SharedPtr msg) {
        bodyCommandPublisher(msg);
        old_msg_ = *msg;
    }

    void bodyCommandPublisher(const ps5_msgs::msg::PS5Joy::SharedPtr &msg) {
        auto body_twist_msg = geometry_msgs::msg::Twist();
        auto head_twist_msg = geometry_msgs::msg::Twist();

        //body_twist_msg.linear.x = msg->right_joystick_y*body_max_linear_velocity_;
        //body_twist_msg.linear.y = msg->right_joystick_x*body_max_linear_velocity_;
        body_twist_msg.linear.z = msg->cross_up*0.5*body_max_linear_velocity_;
        body_twist_msg.angular.z = msg->cross_left*body_max_angular_velocity_;
        body_twist_pub_->publish(body_twist_msg);

        head_twist_msg.angular.z = msg->left_joystick_x*head_max_angular_velocity_;
        head_twist_msg.angular.x = msg->left_joystick_y*head_max_angular_velocity_;
        head_twist_pub_->publish(head_twist_msg);

    }


    // Parameters
    double body_max_linear_velocity_ = 0.0;
    double body_max_angular_velocity_ = 0.0;
    double head_max_linear_velocity_ = 0.0;
    double head_max_angular_velocity_ = 0.0;

    private:
    rclcpp::Subscription<ps5_msgs::msg::PS5Joy>::SharedPtr ps5_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr body_twist_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr head_twist_pub_;
    ps5_msgs::msg::PS5Joy old_msg_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    rclcpp::spin(std::make_shared<Joy2Commandnitromanbody>());
    rclcpp::shutdown();
    return 0;
}
