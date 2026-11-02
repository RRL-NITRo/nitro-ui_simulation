#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <ps5_msgs/msg/ps5_joy.hpp>
#include <yaml-cpp/yaml.h>
#include <unordered_map>
#include <filesystem>
#include <cmath>
#include <stdexcept>

// Custom conversion function for std::unordered_map<std::string, int>
namespace YAML {
template<>
struct convert<std::unordered_map<std::string, int>> {
    static Node encode(const std::unordered_map<std::string, int>& rhs) {
        Node node;
        for (const auto& pair : rhs) {
            node[pair.first] = pair.second;
        }
        return node;
    }

    static bool decode(const Node& node, std::unordered_map<std::string, int>& rhs) {
        if (!node.IsMap()) {
            return false;
        }
        for (const auto& pair : node) {
            rhs[pair.first.as<std::string>()] = pair.second.as<int>();
        }
        return true;
    }
};
}

class PS5JoyPublisher : public rclcpp::Node {
public:
    PS5JoyPublisher(const std::string &yaml_file_path) : Node("ps5_joy_publisher") {
        const float DEFAULT_DEADZONE = 0.1f;

        // Debug log to print the YAML file path
        RCLCPP_DEBUG(this->get_logger(), "YAML file path: %s", yaml_file_path.c_str());

        // Check if the YAML file exists
        if (!std::filesystem::exists(yaml_file_path)) {
            RCLCPP_ERROR(this->get_logger(), "YAML file not found: %s", yaml_file_path.c_str());
            throw std::runtime_error("YAML file not found");
        }

        // Load the YAML file
        YAML::Node config = YAML::LoadFile(yaml_file_path);
        YAML::Node parameters = config["ps5_joy_publisher"]["ros__parameters"];

        // Load deadzone settings
        left_joystick_deadzone_ = config["deadzones"]["left_joystick"].as<float>(DEFAULT_DEADZONE);
        right_joystick_deadzone_ = config["deadzones"]["right_joystick"].as<float>(DEFAULT_DEADZONE);

        // Log deadzone settings
        RCLCPP_INFO(this->get_logger(), "Left joystick deadzone set to: %f", left_joystick_deadzone_);
        RCLCPP_INFO(this->get_logger(), "Right joystick deadzone set to: %f", right_joystick_deadzone_);

        // Load button and axis mappings
        button_mapping_ = parameters["buttons"].as<std::unordered_map<std::string, int>>();
        axis_mapping_ = parameters["axes"].as<std::unordered_map<std::string, int>>();

        // Log warnings if mappings are empty
        if (button_mapping_.empty()) {
            RCLCPP_WARN(this->get_logger(), "Button mapping not specified; using empty mapping");
        }

        if (axis_mapping_.empty()) {
            RCLCPP_WARN(this->get_logger(), "Axis mapping not specified; using empty mapping");
        }

        // Set up the subscriber
        joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&PS5JoyPublisher::joy_callback, this, std::placeholders::_1));

        // Set up the publisher
        ps5_joy_publisher_ = this->create_publisher<ps5_msgs::msg::PS5Joy>("ps5_joy", 10);

        // Create the PS5Joy message
        ps5_joy_msg_ = std::make_shared<ps5_msgs::msg::PS5Joy>();
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {

float left_joystick_x = 0.0f;
float left_joystick_y = 0.0f;
if (axis_mapping_.count("left_joystick_x") && axis_mapping_.count("left_joystick_y")) {
    left_joystick_x = joy_msg->axes[axis_mapping_["left_joystick_x"]];
    left_joystick_y = joy_msg->axes[axis_mapping_["left_joystick_y"]];
    if ((left_joystick_x * left_joystick_x + left_joystick_y * left_joystick_y) < (left_joystick_deadzone_ * left_joystick_deadzone_)) {
        left_joystick_x = 0.0f;
        left_joystick_y = 0.0f;
    }
}

// Process right joystick deadzone
float right_joystick_x = 0.0f;
float right_joystick_y = 0.0f;
if (axis_mapping_.count("right_joystick_x") && axis_mapping_.count("right_joystick_y")) {
    right_joystick_x = joy_msg->axes[axis_mapping_["right_joystick_x"]];
    right_joystick_y = joy_msg->axes[axis_mapping_["right_joystick_y"]];
    if ((right_joystick_x * right_joystick_x + right_joystick_y * right_joystick_y) < (right_joystick_deadzone_ * right_joystick_deadzone_)) {
        right_joystick_x = 0.0f;
        right_joystick_y = 0.0f;
    }
}

ps5_joy_msg_->left_joystick_deadzone = left_joystick_deadzone_;
ps5_joy_msg_->right_joystick_deadzone = right_joystick_deadzone_;

if (axis_mapping_.count("cross_up")) {
    ps5_joy_msg_->cross_up = joy_msg->axes[axis_mapping_["cross_up"]];
}
if (axis_mapping_.count("cross_left")) {
    ps5_joy_msg_->cross_left = joy_msg->axes[axis_mapping_["cross_left"]];
}

if (axis_mapping_.count("left_trigger")) {
    ps5_joy_msg_->left_trigger = joy_msg->axes[axis_mapping_["left_trigger"]];
}
if (axis_mapping_.count("right_trigger")) {
    ps5_joy_msg_->right_trigger = joy_msg->axes[axis_mapping_["right_trigger"]];
}

        // Set the joystick values in the message
        ps5_joy_msg_->left_joystick_x = left_joystick_x;
        ps5_joy_msg_->left_joystick_y = left_joystick_y;
        ps5_joy_msg_->right_joystick_x = right_joystick_x;
        ps5_joy_msg_->right_joystick_y = right_joystick_y;

        // Map all button values
        for (const auto& button : button_mapping_) {
            bool button_state = joy_msg->buttons[button.second]; // Get the button state
            // Update the corresponding button field in the message
            if (button.first == "square") ps5_joy_msg_->square = button_state;
            else if (button.first == "cross") ps5_joy_msg_->cross = button_state;
            else if (button.first == "circle") ps5_joy_msg_->circle = button_state;
            else if (button.first == "triangle") ps5_joy_msg_->triangle = button_state;
            else if (button.first == "l1") ps5_joy_msg_->l1 = button_state;
            else if (button.first == "r1") ps5_joy_msg_->r1 = button_state;
            else if (button.first == "l2") ps5_joy_msg_->l2 = button_state;
            else if (button.first == "r2") ps5_joy_msg_->r2 = button_state;
            else if (button.first == "command_l") ps5_joy_msg_->command_l = button_state;
            else if (button.first == "command_r") ps5_joy_msg_->command_r = button_state;
            else if (button.first == "joy_l") ps5_joy_msg_->joy_l = button_state;
            else if (button.first == "joy_r") ps5_joy_msg_->joy_r = button_state;
            else if (button.first == "home") ps5_joy_msg_->home = button_state;
            else if (button.first == "capture") ps5_joy_msg_->capture = button_state;
        }

        // Publish the message
        ps5_joy_publisher_->publish(*ps5_joy_msg_);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    rclcpp::Publisher<ps5_msgs::msg::PS5Joy>::SharedPtr ps5_joy_publisher_;
    std::unordered_map<std::string, int> button_mapping_;  // Button mapping
    std::unordered_map<std::string, int> axis_mapping_;    // Axis mapping
    float left_joystick_deadzone_;
    float right_joystick_deadzone_;
    std::shared_ptr<ps5_msgs::msg::PS5Joy> ps5_joy_msg_;  // Reusable PS5Joy message
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);

    // Process command line arguments
    std::string yaml_file_path;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--params-file" && i + 1 < argc) {
            yaml_file_path = argv[++i];
        }
    }

    // Confirm YAML file path
    if (yaml_file_path.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "YAML file path not provided");
        return 1;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "YAML file path: %s", yaml_file_path.c_str());

    rclcpp::spin(std::make_shared<PS5JoyPublisher>(yaml_file_path));

    rclcpp::shutdown();
    return 0;
}
