#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <arm_msgs/msg/arm_command.hpp>
#include <ps5_msgs/msg/ps5_joy.hpp>
#include <std_msgs/msg/bool.hpp>

class openarm_joy2command : public rclcpp::Node {
public:
    openarm_joy2command() : Node("ps5_to_ros_command") {
        loadParameters();
        initPublishers();
        ps5_sub_ = this->create_subscription<ps5_msgs::msg::PS5Joy>(
            "/ps5_joy", 1, std::bind(&openarm_joy2command::ps5Callback, this, std::placeholders::_1));

    }

    void initPublishers() {
        arm_command_pub_ = this->create_publisher<arm_msgs::msg::ArmCommand>("/arm_command/right", 1);
        arm_twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/arm_twist/right", 1);
        joy_mode_pub_ = this->create_publisher<std_msgs::msg::String>("/joy_mode_str/right", 1);

        pub_gripper_ = this->create_publisher<sensor_msgs::msg::JointState>("/gripper_target/right", 1);
        pub_flag_ = this->create_publisher<std_msgs::msg::Bool>("/flag/right", 1);
    }
    
    void loadParameters() {
        // Load parameters for linear and angular movements
        

        this->declare_parameter("manipulator.max_linear_velocity", 0.1);
        this->declare_parameter("manipulator.max_angular_velocity", 0.1);

        this->get_parameter("manipulator.max_linear_velocity", manipulator_max_linear_velocity_);
        this->get_parameter("manipulator.max_angular_velocity", manipulator_max_angular_velocity_);

        
    }

    void ps5Callback(const ps5_msgs::msg::PS5Joy::SharedPtr msg) {
        if (msg->command_l) {
            toggleArmMode(msg);
        }

        if (msg->joy_l && !old_msg_.joy_l) {
            is_back_mode = !is_back_mode;
            RCLCPP_INFO(this->get_logger(), "Back mode %s", is_back_mode ? "enabled" : "disabled");
        }

        if (is_arm_mode_) {
            armCommandPublisher(msg);
        } 
        old_msg_ = *msg;
    }

    void toggleArmMode(const ps5_msgs::msg::PS5Joy::SharedPtr &msg) {
        if (msg->command_l && !old_msg_.command_l) {
            is_arm_mode_ = !is_arm_mode_;
            joy_mode_str_.data = is_arm_mode_ ? "[Manipulator]" : "[Flipper]";
            joy_mode_pub_->publish(joy_mode_str_);
            RCLCPP_INFO(this->get_logger(), "%s_MODE", is_arm_mode_ ? "ARM" : "FLIPPER");
        }
    }


    void armCommandPublisher(const ps5_msgs::msg::PS5Joy::SharedPtr &msg) {
        auto arm_twist_msg = geometry_msgs::msg::Twist();
        arm_twist_msg.linear.x = msg->right_joystick_y*manipulator_max_linear_velocity_;
        arm_twist_msg.linear.y = msg->right_joystick_x*manipulator_max_linear_velocity_;
        arm_twist_msg.linear.z = (msg->l1 ? 1.0 : (msg->r1 ? -1.0 : 0.0))*0.5*manipulator_max_linear_velocity_;
        arm_twist_msg.angular.x = (msg->l2 ? 1.0 : (msg->r2 ? -1.0 : 0.0))*manipulator_max_angular_velocity_;
        arm_twist_msg.angular.y = msg->cross_up*manipulator_max_angular_velocity_;
        arm_twist_msg.angular.z = msg->cross_left*manipulator_max_angular_velocity_;
        arm_twist_pub_->publish(arm_twist_msg);

        auto arm_command_msg = arm_msgs::msg::ArmCommand();
        static bool gripper_state = false;
        if (msg->circle && !old_msg_.circle) {
            gripper_state = !gripper_state;
        }

        gripper_target_.header.stamp = this->now();
        gripper_target_.name = {"openarm_right_finger_joint1", "openarm_right_finger_joint2"};
        gripper_target_.position = {gripper_state ? 0.044 : 0.0, gripper_state ? 0.044 : 0.0};
        gripper_target_.velocity = {0.0 , 0.0};
        gripper_target_.effort = {0.0, 0.0};
        pub_gripper_->publish(gripper_target_);
        arm_command_msg.gripper_close = gripper_state;
        arm_command_msg.gripper_open = !gripper_state;
        arm_command_msg.reset_home_position = msg->home;
        arm_command_msg.reset_end_posture = msg->capture;
        arm_command_pub_->publish(arm_command_msg);
        
    }



    // Parameters
    
    double manipulator_max_linear_velocity_ = 0.0;
    double manipulator_max_angular_velocity_ = 0.0;

    private:
    rclcpp::Subscription<ps5_msgs::msg::PS5Joy>::SharedPtr ps5_sub_;
    rclcpp::Publisher<arm_msgs::msg::ArmCommand>::SharedPtr arm_command_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr joy_mode_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_gripper_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_flag_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr arm_twist_pub_;
    std_msgs::msg::String joy_mode_str_;
    sensor_msgs::msg::JointState gripper_target_;
    bool is_arm_mode_ = false;
    bool is_back_mode = false;

    ps5_msgs::msg::PS5Joy old_msg_;

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<openarm_joy2command>());
    rclcpp::shutdown();
    return 0;
}
