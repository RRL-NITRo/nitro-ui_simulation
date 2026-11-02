#include <cnoid/SimpleController>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using namespace std;
using namespace cnoid;

class ImaxJointStatePublisher : public SimpleController
{
    BodyPtr ioBody;
    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatePublisher;
    sensor_msgs::msg::JointState jointState;
    double time;
    double timeStep;
    double cycleTime;
    double timeCounter;
    std::vector<std::pair<std::string, std::string>> joint_pairs = {
        {"link_yaw", "yaw_joint"}, {"link_first_link", "first_link_joint"}, {"link_second_link", "second_link_joint"},
        {"link3_roll", "third_link_joint"}, {"link_hand_pitch", "hand_pitch"}, {"link_hand_roll", "hand_roll"},
        {"left_finger", "left_finger"}, {"right_finger", "right_finger"},
        {"left_front", "joint_left_front"}, {"right_front", "joint_right_front"}, {"right_rear", "joint_right_rear"}, {"left_rear", "joint_left_rear"}
    };
    vector<int> jointIndices;

public:
    virtual bool configure(SimpleControllerConfig* config) override
    {
        node = rclcpp::Node::make_shared(config->controllerName());
        jointStatePublisher = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
        return true;
    }

    virtual bool initialize(SimpleControllerIO* io) override
    {
        ioBody = io->body();

        // joint_pairsのリンク名（first）を使ってjointIdを取得し、見つかった場合はjointIndicesに追加
        for (const auto& pair : joint_pairs) {
            auto link = ioBody->link(pair.first);  // firstがリンク名
            if (link && link->jointId() >= 0) {
                jointIndices.push_back(link->jointId());
            } else {
                RCLCPP_WARN(node->get_logger(), "Joint %s not found", pair.first.c_str());
            }
        }

        // jointStateのサイズを設定
        jointState.name.resize(jointIndices.size());
        jointState.position.resize(jointIndices.size());
        jointState.velocity.resize(jointIndices.size());
        jointState.effort.resize(jointIndices.size());

        // jointState.nameにsecondを使用して名前を設定
        for (size_t i = 0; i < jointIndices.size(); ++i) {
            jointState.name[i] = joint_pairs[i].second;  // secondがパブリッシュ時の名前
            auto joint = ioBody->joint(jointIndices[i]);
            io->enableInput(joint, JointDisplacement | JointVelocity | JointEffort);
        }

        time = 0.0;
        timeStep = io->timeStep();
        double frequency = 50.0;
        cycleTime = 1.0 / frequency;
        timeCounter = 0.0;

        return true;
    }

    virtual bool control() override
    {
        time += timeStep;
        timeCounter += timeStep;

        if (timeCounter >= cycleTime) {

            jointState.header.stamp = node->now();

            for (int i = 0; i < jointIndices.size(); ++i) {
                auto joint = ioBody->joint(jointIndices[i]);
                jointState.position[i] = joint->q();
                jointState.velocity[i] = joint->dq();
                jointState.effort[i] = joint->u();
            }

            jointStatePublisher->publish(jointState);

            timeCounter -= cycleTime;
        }

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(ImaxJointStatePublisher)
