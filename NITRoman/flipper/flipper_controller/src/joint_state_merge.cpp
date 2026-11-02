#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class JointStatePublisher : public rclcpp::Node
{
public:
    JointStatePublisher() : Node("joint_state_publisher")
    {
        // サブスクライバーを作成
        left_arm_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/target/joint_states_arm/left", 10,
            std::bind(&JointStatePublisher::leftarm_callback, this, std::placeholders::_1));

        right_arm_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/target/joint_states_arm/right", 10,
            std::bind(&JointStatePublisher::rightarm_callback, this, std::placeholders::_1));

        flipper_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/target/joint_states_flipper", 10,
            std::bind(&JointStatePublisher::flipper_callback, this, std::placeholders::_1));

        

        // パブリッシャーを作成
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/target/joint_states", 10);

        // ジョイント名を登録し、すべての位置に0を初期値として設定
        joint_state_.name = {
            "openarm_left_joint1", "openarm_left_joint2", "openarm_left_joint3", "openarm_left_joint4",
            "openarm_left_joint5", "openarm_left_joint6", "openarm_left_joint7", "left_openarm_hand_joint",
            "openarm_right_joint1", "openarm_right_joint2", "openarm_right_joint3", "openarm_right_joint4",
            "openarm_right_joint5", "openarm_right_joint6", "openarm_right_joint7", "right_openarm_hand_joint",
            "joint_left_front", "joint_left_rear", "joint_right_front", "joint_right_rear"
        };
        joint_state_.position.resize(joint_state_.name.size(), 0.0);  // 全てのジョイントに対して初期値を0に設定

        // 30Hzでパブリッシュ
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33), // 約30Hz (1000ms / 30 = 33ms)
            std::bind(&JointStatePublisher::publish_joint_states, this));
    }

private:
    // void arm_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    // {
    //     // ジョイント名に基づいて値を代入
    //     for (size_t i = 0; i < msg->name.size(); ++i) {
    //         if (msg->name[i] == "yaw_joint") {
    //             joint_state_.position[0] = msg->position[i];
    //         } else if (msg->name[i] == "first_link_joint") {
    //             joint_state_.position[1] = msg->position[i];
    //         } else if (msg->name[i] == "second_link_joint") {
    //             joint_state_.position[2] = msg->position[i];
    //         } else if (msg->name[i] == "third_link_joint") {
    //             joint_state_.position[3] = msg->position[i];
    //         } else if (msg->name[i] == "hand_pitch") {
    //             joint_state_.position[4] = msg->position[i];
    //         } else if (msg->name[i] == "hand_roll") {
    //             joint_state_.position[5] = msg->position[i];
    //         }
    //     }
    // }

    void leftarm_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // ジョイント名に基づいて値を代入
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "openarm_left_joint1") {
                joint_state_.position[0] = msg->position[i];
            } else if (msg->name[i] == "openarm_left_joint2") {
                joint_state_.position[1] = msg->position[i];
            } else if (msg->name[i] == "openarm_left_joint3") {
                joint_state_.position[2] = msg->position[i];
            } else if (msg->name[i] == "openarm_left_joint4") {
                joint_state_.position[3] = msg->position[i];
            } else if (msg->name[i] == "openarm_left_joint5") {
                joint_state_.position[4] = msg->position[i];
            } else if (msg->name[i] == "openarm_left_joint6") {
                joint_state_.position[5] = msg->position[i];
            } else if (msg->name[i] == "openarm_left_joint7") {
                joint_state_.position[6] = msg->position[i];
            } else if (msg->name[i] == "left_openarm_hand_joint") {
                joint_state_.position[7] = msg->position[i];
            }
        }
    }

    void rightarm_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // ジョイント名に基づいて値を代入
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "openarm_right_joint1") {
                joint_state_.position[8] = msg->position[i];
            } else if (msg->name[i] == "openarm_right_joint2") {
                joint_state_.position[9] = msg->position[i];
            } else if (msg->name[i] == "openarm_right_joint3") {
                joint_state_.position[10] = msg->position[i];
            } else if (msg->name[i] == "openarm_right_joint4") {
                joint_state_.position[11] = msg->position[i];
            } else if (msg->name[i] == "openarm_right_joint5") {
                joint_state_.position[12] = msg->position[i];
            } else if (msg->name[i] == "openarm_right_joint6") {
                joint_state_.position[13] = msg->position[i];
            } else if (msg->name[i] == "openarm_right_joint7") {
                joint_state_.position[14] = msg->position[i];
            } else if (msg->name[i] == "right_openarm_hand_joint") {
                joint_state_.position[15] = msg->position[i];
            }
        }
    }

    void flipper_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // フリッパージョイントの値を代入
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "joint_left_front") {
                joint_state_.position[16] = msg->position[i];
            } else if (msg->name[i] == "joint_left_rear") {
                joint_state_.position[17] = msg->position[i];
            } else if (msg->name[i] == "joint_right_front") {
                joint_state_.position[18] = msg->position[i];
            } else if (msg->name[i] == "joint_right_rear") {
                joint_state_.position[19] = msg->position[i];
            }
        }
    }

    void publish_joint_states()
    {
        joint_state_.header.stamp = this->get_clock()->now();

        // 統合したジョイントステートをパブリッシュ
        joint_state_publisher_->publish(joint_state_);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr left_arm_subscriber_, right_arm_subscriber_, flipper_subscriber_;
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
