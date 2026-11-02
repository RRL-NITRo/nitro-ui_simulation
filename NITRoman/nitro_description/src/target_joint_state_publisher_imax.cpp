#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class JointStatePublisher : public rclcpp::Node
{
public:
    JointStatePublisher() : Node("joint_state_publisher")
    {
        // サブスクライバーを作成
        arm_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/target/joint_states_arm", 10,
            std::bind(&JointStatePublisher::arm_callback, this, std::placeholders::_1));

        flipper_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/target/joint_states_flipper", 10,
            std::bind(&JointStatePublisher::flipper_callback, this, std::placeholders::_1));

        

        // パブリッシャーを作成
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/target/joint_states", 10);

        // ジョイント名を登録し、すべての位置に0を初期値として設定
        joint_state_.name = {
            "yaw_joint", "first_link_joint", "second_link_joint", "third_link_joint",
            "hand_pitch", "hand_roll", "joint_left_front", "joint_left_rear",
            "joint_right_front", "joint_right_rear"
        };
        joint_state_.position.resize(joint_state_.name.size(), 0.0);  // 全てのジョイントに対して初期値を0に設定

        // 30Hzでパブリッシュ
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33), // 約30Hz (1000ms / 30 = 33ms)
            std::bind(&JointStatePublisher::publish_joint_states, this));
    }

private:
    void arm_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // ジョイント名に基づいて値を代入
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "yaw_joint") {
                joint_state_.position[0] = msg->position[i];
            } else if (msg->name[i] == "first_link_joint") {
                joint_state_.position[1] = msg->position[i];
            } else if (msg->name[i] == "second_link_joint") {
                joint_state_.position[2] = msg->position[i];
            } else if (msg->name[i] == "third_link_joint") {
                joint_state_.position[3] = msg->position[i];
            } else if (msg->name[i] == "hand_pitch") {
                joint_state_.position[4] = msg->position[i];
            } else if (msg->name[i] == "hand_roll") {
                joint_state_.position[5] = msg->position[i];
            }
        }
    }

    void flipper_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // フリッパージョイントの値を代入
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "joint_left_front") {
                joint_state_.position[6] = msg->position[i];
            } else if (msg->name[i] == "joint_left_rear") {
                joint_state_.position[7] = msg->position[i];
            } else if (msg->name[i] == "joint_right_front") {
                joint_state_.position[8] = msg->position[i];
            } else if (msg->name[i] == "joint_right_rear") {
                joint_state_.position[9] = msg->position[i];
            }
        }
    }

    void publish_joint_states()
    {
        joint_state_.header.stamp = this->get_clock()->now();

        // 統合したジョイントステートをパブリッシュ
        joint_state_publisher_->publish(joint_state_);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arm_subscriber_,flipper_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    sensor_msgs::msg::JointState joint_state_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStatePublisher>());
    rclcpp::shutdown();
    return 0;
}
