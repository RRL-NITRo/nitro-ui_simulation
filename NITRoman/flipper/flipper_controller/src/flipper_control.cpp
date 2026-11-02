#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "flipper_msgs/msg/flipper.hpp"

class FlipperController : public rclcpp::Node
{
public:
    FlipperController()
        : Node("flipper_controller")
    {
        // Flipper control subscriber
        flipper_sub_ = this->create_subscription<flipper_msgs::msg::Flipper>(
            "/flipper_command", 10, std::bind(&FlipperController::flipperCallback, this, std::placeholders::_1));

        // Joint state publisher
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/target/joint_states_flipper", 10);

        // Initialize joint positions
        joint_positions_["joint_left_front"] = 0.0;
        joint_positions_["joint_left_rear"] = 0.0;
        joint_positions_["joint_right_front"] = 0.0;
        joint_positions_["joint_right_rear"] = 0.0;
    }

private:
    void flipperCallback(const flipper_msgs::msg::Flipper::SharedPtr msg)
    {
        auto joint_state_msg = sensor_msgs::msg::JointState();
        joint_state_msg.header.stamp = this->get_clock()->now();
        joint_state_msg.name = msg->name;
        joint_state_msg.position.resize(msg->name.size());

        // Joint positionsを更新
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            joint_state_msg.position[i] = static_cast<double>(joint_positions_[msg->name[i]]);
        }

        if (msg->use_position)
        {
            // floatからdoubleに変換しつつpositionをコピー
            joint_state_msg.position.resize(4);
            joint_state_msg.velocity.resize(4);
            for (size_t i = 0; i < 4; ++i)
            {
                if (i < msg->position.size())
                {
                    double target_position = static_cast<double>(msg->position[i]);
                    double current_position = joint_positions_[msg->name[i]];
                    double step = 0.02 * (target_position - current_position); // Proportional control step
                    joint_state_msg.position[i] = current_position + step;
                    joint_positions_[msg->name[i]] = joint_state_msg.position[i];
                }
                else
                {
                    joint_state_msg.position[i] = joint_positions_[msg->name[i]];
                }
            }
        }
        else if (msg->use_velocity)
        {
            joint_state_msg.velocity.resize(msg->velocity.size());
            for (size_t i = 0; i < msg->velocity.size(); ++i)
            {
                joint_state_msg.velocity[i] = static_cast<double>(msg->velocity[i]);
                joint_positions_[msg->name[i]] += joint_state_msg.velocity[i] * 0.05; // Assuming a time step of 0.05 seconds
            }

            joint_state_msg.position.resize(msg->name.size());
            for (size_t i = 0; i < msg->name.size(); ++i)
            {
                joint_state_msg.position[i] = joint_positions_[msg->name[i]];
            }
        }

        // reset_home_positionが押された場合、positionをすべて0にリセット
        if (msg->reset_home_position)
        {
            for (auto &position : joint_state_msg.position)
            {
                position = 0.0;
            }

            joint_positions_["joint_left_front"] = 0.0;
            joint_positions_["joint_left_rear"] = 0.0;
            joint_positions_["joint_right_front"] = 0.0;
            joint_positions_["joint_right_rear"] = 0.0;
        }

        // Ensure joint positions are within the range [-2PI, 2PI]
        for (auto &position : joint_state_msg.position)
        {
            if (position > 2 * M_PI)
            {
                position = 2 * M_PI;
            }
            else if (position < -2 * M_PI)
            {
                position = -2 * M_PI;
            }
        }

        // Round joint positions to 4 decimal places
        for (auto &position : joint_state_msg.position)
        {
            position = std::round(position * 10000.0) / 10000.0;
        }
        joint_pub_->publish(joint_state_msg);
    }

    rclcpp::Subscription<flipper_msgs::msg::Flipper>::SharedPtr flipper_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;

    std::map<std::string, double> joint_positions_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FlipperController>());
    rclcpp::shutdown();
    return 0;
}

