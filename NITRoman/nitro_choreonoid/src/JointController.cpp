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

class JointController : public SimpleController
{
    std::shared_ptr<rclcpp::Node> node;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_gripper;
    
    std::vector<Link*> cnoid_joint;
    double dt;
    std::vector<std::pair<std::string, std::string>> joint_pairs = {
        {"link_yaw", "yaw_joint"}, {"link_first_link", "first_link_joint"}, {"link_second_link", "second_link_joint"},
        {"link3_roll", "third_link_joint"}, {"link_hand_pitch", "hand_pitch"}, {"link_hand_roll", "hand_roll"},
        {"left_finger", "left_finger"}, {"right_finger", "right_finger"},
        {"left_front", "joint_left_front"}, {"right_front", "joint_right_front"}, {"right_rear", "joint_right_rear"}, {"left_rear", "joint_left_rear"}
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

        {500, 5, 10}, {500, 10, 10}, {500, 5, 10}, {500, 0.1, 0.1}, {500, 1, 0.1}, {100, 0.1, 0.1},
        {100, 1, 1}, {100, 1, 1},
        {500, 0, 10}, {500, 0, 10}, {500, 0, 10}, {500, 0, 10}
    };

    enum JointMode {
       Effort,
        Position
    };

    std::vector<JointMode> joint_modes = {
        Effort, Effort, Effort, Effort, Effort, Effort,
        // Effort, Effort,
        Position, Position,
        //Effort, Effort, Effort, Effort
        Position, Position, Position, Position
    };
    std::vector<float> joint_p_gain, joint_d_gain, joint_i_gain;
    std::vector<double> armJointDiff = std::vector<double>(joint_pairs.size(), 0.0);
    std::mutex commandMutex;

private:
    int armjoint_num = joint_pairs.size();
    std::thread executorThread;
    bool running = false;

public:
    JointController() {
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
            "/target/joint_states", 1, std::bind(&JointController::callback_joint_state, this, std::placeholders::_1)
        );
        sub_gripper = node->create_subscription<sensor_msgs::msg::JointState>(
            "/gripper_target", 1, std::bind(&JointController::callback_gripper, this, std::placeholders::_1)
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

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(JointController)
