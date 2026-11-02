#include <cnoid/SimpleController>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cnoid/BodyItem>
#include <cnoid/SimulatorItem>
#include <mutex>
#include <vector>
#include <algorithm>
#include <thread>

using namespace cnoid;

class openarm_JointControllerleft : public SimpleController
{
    std::shared_ptr<rclcpp::Node> node;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_gripper;
    
    std::vector<Link*> cnoid_joint;
    double dt;
    std::vector<std::pair<std::string, std::string>> joint_pairs = {
        {"openarm_left_link1", "openarm_left_joint1"}, {"openarm_left_link2", "openarm_left_joint2"}, {"openarm_left_link3", "openarm_left_joint3"},
        {"openarm_left_link4", "openarm_left_joint4"}, {"openarm_left_link5", "openarm_left_joint5"}, {"openarm_left_link6", "openarm_left_joint6"},
        {"openarm_left_link7", "openarm_left_joint7"}, {"openarm_left_hand", "left_openarm_hand_joint"},
        {"openarm_left_left_finger", "openarm_left_finger_joint2"}, {"openarm_left_right_finger", "openarm_left_finger_joint1"}
    };

    struct JointGain {
        float P;
        float I;
        float D;
    };

    std::vector<JointGain> joint_gains = {

        // {500, 5, 10}, {500, 10, 10}, {500, 5, 10}, {5, 0.1, 0.1}, {10, 1, 0.1}, {1, 0.1, 0.1},
        // {1, 0.01, 0.01}, {1, 0.01, 0.01},
        // {500, 0, 10}, {500, 0, 10}, {500, 0, 10}, {500, 0, 10}

        {500, 5, 10}, {500, 10, 10}, {500, 5, 10}, {500, 0.1, 0.1}, {500, 1, 0.1}, {100, 0.1, 0.1}, {100, 0.1, 0.1}, {100, 0.1, 0.1},
        {100, 0.01, 0.01}, {100, 0.01, 0.01},
    };

    enum JointMode {
       Effort,
        Position
    };

    std::vector<JointMode> joint_modes = {
        Position, Position, Position, Position, Position, Position, Position,
        Position, Position
    };
    std::vector<float> joint_p_gain, joint_d_gain, joint_i_gain;
    std::vector<double> armJointDiff = std::vector<double>(joint_pairs.size(), 0.0);
    std::mutex commandMutex;

private:
    int armjoint_num = joint_pairs.size();
    std::thread executorThread;
    bool running = false;

public:
    openarm_JointControllerleft() {
        for (const auto& gain : joint_gains) {
            joint_p_gain.push_back(gain.P);
            joint_d_gain.push_back(gain.D);
            joint_i_gain.push_back(gain.I);
        }
        cnoid_joint.resize(joint_pairs.size());
    }

    SimpleControllerIO* io_;
    std_msgs::msg::Float32MultiArray joint_target_q, joint_target_q_before;

public:
    virtual bool configure(SimpleControllerConfig* config) override
    {
        node = rclcpp::Node::make_shared(config->controllerName());
        sub_joint_state = node->create_subscription<sensor_msgs::msg::JointState>(
            "/target/joint_states/arm_left", 1, std::bind(&openarm_JointControllerleft::callback_joint_state, this, std::placeholders::_1)
        );
        sub_gripper = node->create_subscription<sensor_msgs::msg::JointState>(
            "/gripper_target/left", 1, std::bind(&openarm_JointControllerleft::callback_gripper, this, std::placeholders::_1)
        );

        return true;
    }

    virtual bool initialize(SimpleControllerIO* io) override
    {
        io_ = io;
        Body* body = io->body();
        dt = io->timeStep();

        for (int i = 0; i < joint_pairs.size(); ++i) {
            cnoid_joint[i] = body->link(joint_pairs[i].first);
            if (!cnoid_joint[i]) {
                io->os() << "Link " << joint_pairs[i].first << " not found." << std::endl;
                return false;
            }
        }

        for (int i = 0; i < joint_pairs.size(); i++) {
            if (joint_modes[i] == Effort) {
                cnoid_joint[i]->setActuationMode(JointEffort);
                io->enableIO(cnoid_joint[i]);

            } else {
                cnoid_joint[i]->setActuationMode(JointDisplacement);
                //io->enableInput(cnoid_joint[i], JointDisplacement);
                //io->enableOutput(cnoid_joint[i], JointDisplacement);
                io->enableIO(cnoid_joint[i]);
            }
        }

        joint_target_q.data.resize(armjoint_num);
        joint_target_q_before.data.resize(armjoint_num);

        // スレッドを開始
        running = true;
        executorThread = std::thread([this]() {
            rclcpp::executors::SingleThreadedExecutor exec;
            exec.add_node(node);
            while (running) {
                exec.spin_some();
            }
        });

        return true;
    }

    void callback_joint_state(const sensor_msgs::msg::JointState::SharedPtr msgs)
    {
        std::lock_guard<std::mutex> lock(commandMutex);
        for (size_t i = 0; i < msgs->name.size(); ++i) {
            auto it = std::find_if(joint_pairs.begin(), joint_pairs.end(), [&](const std::pair<std::string, std::string>& pair) {
                return pair.second == msgs->name[i];
            });
            if (it != joint_pairs.end()) {
                size_t index = std::distance(joint_pairs.begin(), it);
                if (index < joint_target_q.data.size()) {
                    joint_target_q.data[index] = std::round(msgs->position[i] * 1000) / 1000.0;
                }
            }
        }
    }

    void callback_gripper(const sensor_msgs::msg::JointState::SharedPtr msgs)
    {
        std::lock_guard<std::mutex> lock(commandMutex);
        for (size_t i = 0; i < msgs->name.size(); ++i) {
            auto it = std::find_if(joint_pairs.begin(), joint_pairs.end(), [&](const std::pair<std::string, std::string>& pair) {
                return pair.second == msgs->name[i];
            });
            if (it != joint_pairs.end()) {
                size_t index = std::distance(joint_pairs.begin(), it);
                if (index < joint_target_q.data.size()) {
                    joint_target_q.data[index] = std::round(msgs->position[i] * 1000) / 1000.0;
                }
            }
        }
    }

    virtual bool control() override
    {
        std::lock_guard<std::mutex> lock(commandMutex);
        for (size_t i = 0; i < cnoid_joint.size(); i++) {
            if (joint_modes[i] == Effort) {
                // 力制御の場合
                armJointDiff[i] += (joint_target_q.data[i] - cnoid_joint[i]->q()) * dt;
                float armJoint_speed = (cnoid_joint[i]->q() - joint_target_q_before.data[i]) / dt;
                double control_effort = joint_p_gain[i] * (joint_target_q.data[i] - cnoid_joint[i]->q()) +
                                        joint_d_gain[i] * (0 - armJoint_speed) +
                                        joint_i_gain[i] * armJointDiff[i];
                control_effort = std::clamp(control_effort, -1000.0, 1000.0);
                cnoid_joint[i]->u() = control_effort;
            } else {
                // 位置制御の場合
                cnoid_joint[i]->q_target() = joint_target_q.data[i];
            }
            joint_target_q_before.data[i] = cnoid_joint[i]->q();
        }

        return true;
    }

    virtual void stop() override
    {
        running = false;  // スレッドの終了フラグをセット
        if (executorThread.joinable()) {
            executorThread.join();  // スレッドを終了させる
        }
        if (sub_joint_state) sub_joint_state.reset();
        if (sub_gripper) sub_gripper.reset();
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(openarm_JointControllerleft)
