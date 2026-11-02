#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class JointStatePublisher : public rclcpp::Node {
public:
    JointStatePublisher() : Node("joint_state_publisher")
    {
        // サブスクライバー
        arm_left_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
            "/target/joint_states/arm_left", 10,
            std::bind(&JointStatePublisher::arm_callback, this, std::placeholders::_1));
        arm_right_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/target/joint_states/arm_right", 10,
            std::bind(&JointStatePublisher::arm_callback, this, std::placeholders::_1));
        arm_body_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/target/joint_states/body", 10,
            std::bind(&JointStatePublisher::arm_callback, this, std::placeholders::_1));
        gripper_left_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/target/joint_states/gripper_left", 10,
            std::bind(&JointStatePublisher::arm_callback, this, std::placeholders::_1));
        gripper_right_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/target/joint_states/gripper_right", 10,
            std::bind(&JointStatePublisher::arm_callback, this, std::placeholders::_1));
        flipper_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/target/joint_states_flipper", 10,
            std::bind(&JointStatePublisher::flipper_callback, this, std::placeholders::_1));

        // パブリッシャー
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/target/joint_states", 10);

        // URDFに基づくジョイント名（左アーム＋右アーム＋ハンド＋フリッパ＋ボディ系）
        joint_state_.name = {
            // body/head
            "body_link_yaw_joint",
            "body_first_link_joint",
            "body_second_link_joint",
            "body_third_link_joint",
            "head_pitch_joint",
            "head_yaw_joint",

            // left arm
            "openarm_left_joint1",
            "openarm_left_joint2",
            "openarm_left_joint3",
            "openarm_left_joint4",
            "openarm_left_joint5",
            "openarm_left_joint6",
            "openarm_left_joint7",
            "openarm_left_finger_joint1",

            // right arm
            "openarm_right_joint1",
            "openarm_right_joint2",
            "openarm_right_joint3",
            "openarm_right_joint4",
            "openarm_right_joint5",
            "openarm_right_joint6",
            "openarm_right_joint7",
            "openarm_right_finger_joint1",

            // flippers
            "joint_left_front",
            "joint_left_rear",
            "joint_right_front",
            "joint_right_rear"
        };
        joint_state_.position.resize(joint_state_.name.size(), 0.0);

        // 30Hzでpublish
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&JointStatePublisher::publish_joint_states, this));
    }

private:
    void arm_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            for (size_t j = 0; j < joint_state_.name.size(); ++j) {
                if (msg->name[i] == joint_state_.name[j]) {
                    joint_state_.position[j] = msg->position[i];
                }
            }
        }
    }

    void flipper_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            for (size_t j = 0; j < joint_state_.name.size(); ++j) {
                if (msg->name[i] == joint_state_.name[j]) {
                    joint_state_.position[j] = msg->position[i];
                }
            }
        }
    }

    void publish_joint_states()
    {
        joint_state_.header.stamp = this->get_clock()->now();
        joint_state_publisher_->publish(joint_state_);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
        arm_left_subscriber, arm_right_subscriber_, arm_body_subscriber_,
        gripper_left_subscriber_, gripper_right_subscriber_, flipper_subscriber_;
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
