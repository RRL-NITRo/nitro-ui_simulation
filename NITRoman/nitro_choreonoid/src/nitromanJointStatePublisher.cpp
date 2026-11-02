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
    std::vector<std::pair<std::string, std::string>> joint_pairs = {//{cnoidのリンク名, パブリッシュ時の名前}
        {"body_link_yaw", "body_link_yaw_joint"}, 
        {"body_link_first_link", "body_first_link_joint"}, 
        {"body_link_second_link", "body_second_link_joint"}, 
        {"body_link_third_link", "body_third_link_joint"}, 
        {"head_pitch", "head_pitch_joint"},
        {"head_yaw", "head_yaw_joint"}, 
        {"openarm_left_link1", "openarm_left_joint1"}, 
        {"openarm_left_link2", "openarm_left_joint2"}, 
        {"openarm_left_link3", "openarm_left_joint3"},
        {"openarm_left_link4", "openarm_left_joint4"}, 
        {"openarm_left_link5", "openarm_left_joint5"}, 
        {"openarm_left_link6", "openarm_left_joint6"},
        {"openarm_left_link7", "openarm_left_joint7"}, 
        {"openarm_left_left_finger", "openarm_left_finger_joint2"}, 
        {"openarm_left_right_finger", "openarm_left_finger_joint1"},
        {"openarm_right_link1", "openarm_right_joint1"}, 
        {"openarm_right_link2", "openarm_right_joint2"}, 
        {"openarm_right_link3", "openarm_right_joint3"},
        {"openarm_right_link4", "openarm_right_joint4"}, 
        {"openarm_right_link5", "openarm_right_joint5"}, 
        {"openarm_right_link6", "openarm_right_joint6"},
        {"openarm_right_link7", "openarm_right_joint7"}, 
        {"openarm_right_left_finger", "openarm_right_finger_joint2"}, 
        {"openarm_right_right_finger", "openarm_right_finger_joint1"},
        {"left_front", "joint_left_front"}, 
        {"right_front", "joint_right_front"}, 
        {"left_rear", "joint_left_rear"},
        {"right_rear", "joint_right_rear"}
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
                std::ostringstream ss;
                ss << "Joint " << pair.first << " not found";
                io->os() << ss.str() << std::endl;
                return false; // リンクが見つからなかった場合は初期化失敗
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
                jointState.position[i] = std::round(joint->q() * 10000.0) / 10000.0;
                //jointState.velocity[i] = std::round(joint->dq() * 10000.0) / 10000.0;
               // jointState.effort[i] = std::round(joint->u() * 10000.0) / 10000.0;
            }

            jointStatePublisher->publish(jointState);

            timeCounter -= cycleTime;
        }

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(ImaxJointStatePublisher)
