#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ps5_msgs/msg/ps5_joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
class Joy2Command : public rclcpp::Node {
public:
    Joy2Command() : Node("ps5_to_ros_command") {
        loadParameters();
        initPublishers();
        ps5_sub_ = this->create_subscription<ps5_msgs::msg::PS5Joy>(
            "/ps5_joy", 1, std::bind(&Joy2Command::ps5Callback, this, std::placeholders::_1));
        
    }

    void initPublishers() {
        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    }
    
    void loadParameters() {
        this->declare_parameter("drone.max_linear_velocity", 10.0);
        this->declare_parameter("drone.max_angular_velocity", 6.28);

        this->get_parameter("drone.max_linear_velocity", drone_max_linear_velocity_);
        this->get_parameter("drone.max_angular_velocity", drone_max_angular_velocity_);
    }

    void ps5Callback(const ps5_msgs::msg::PS5Joy::SharedPtr msg) {
        createTwistMsg(msg);
        old_msg_ = *msg;
    }

    void createTwistMsg(const ps5_msgs::msg::PS5Joy::SharedPtr &msg) {
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = msg->left_joystick_y * drone_max_linear_velocity_;
        twist_msg.linear.y = msg->left_joystick_x * drone_max_linear_velocity_;
        twist_msg.linear.z = msg->right_joystick_y * drone_max_linear_velocity_;
        twist_msg.angular.z = msg->right_joystick_x * drone_max_angular_velocity_;
        twist_pub_->publish(twist_msg);
    }

    // Parameters
    double drone_max_linear_velocity_ = 0.0;    
    double drone_max_angular_velocity_ = 0.0;

private:
    rclcpp::Subscription<ps5_msgs::msg::PS5Joy>::SharedPtr ps5_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    bool is_arm_mode_ = false;
    bool is_back_mode = false;
    ps5_msgs::msg::PS5Joy old_msg_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Joy2Command>());
    rclcpp::shutdown();
    return 0;
}
