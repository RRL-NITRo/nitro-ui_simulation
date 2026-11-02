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

class body_JointController : public SimpleController
{
    std::shared_ptr<rclcpp::Node> node;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state;
    
    std::vector<Link*> cnoid_joint;
    double dt;
    std::vector<std::pair<std::string, std::string>> joint_pairs = {
       {"body_link_yaw", "body_link_yaw_joint"}, {"body_link_first_link", "body_first_link_joint"}, {"body_link_second_link", "body_second_link_joint"}, {"body_link_third_link", "body_third_link_joint"}, {"head_yaw", "head_yaw_joint"}, {"head_pitch", "head_pitch_joint"}
    };

    struct JointGain {
        float P;
        float I;
        float D;
    };

    std::vector<JointGain> joint_gains = {

        {100, 0.1, 1}, {100, 0.1, 10}, {100, 0.1, 1}, {10, 0.01, 0.001}, {100, 0.1, 1}, {100, 0.1, 10}
    };

    enum JointMode {
        Effort,
        Position
    };

    std::vector<JointMode> joint_modes = {
        Position, Position, Position, Position, Position, Position
    };
    std::vector<float> joint_p_gain, joint_d_gain, joint_i_gain;
    std::vector<double> bodyJointDiff = std::vector<double>(joint_pairs.size(), 0.0);
    std::mutex commandMutex;

private:
    int bodyjoint_num = joint_pairs.size();
    std::thread executorThread;
    bool running = false;

public:
    body_JointController() {
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
            "/target/joint_states/body", 1, std::bind(&body_JointController::callback_joint_state, this, std::placeholders::_1)
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

        joint_target_q.data.resize(bodyjoint_num);
        joint_target_q_before.data.resize(bodyjoint_num);

        for (int i = 0; i < bodyjoint_num; ++i) {
            joint_target_q.data[i] = cnoid_joint[i]->q();
            joint_target_q_before.data[i] = cnoid_joint[i]->q();
        }

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

    

    virtual bool control() override
    {
        std::lock_guard<std::mutex> lock(commandMutex);
        for (size_t i = 0; i < cnoid_joint.size(); i++) {
            if (joint_modes[i] == Effort) {
                // 力制御の場合
                bodyJointDiff[i] += (joint_target_q.data[i] - cnoid_joint[i]->q()) * dt;
                float bodyJoint_speed = (cnoid_joint[i]->q() - joint_target_q_before.data[i]) / dt;
                double control_effort = joint_p_gain[i] * (joint_target_q.data[i] - cnoid_joint[i]->q()) +
                                        joint_d_gain[i] * (0 - bodyJoint_speed) +
                                        joint_i_gain[i] * bodyJointDiff[i];
                control_effort = std::clamp(control_effort, -1000.0, 1000.0);
                cnoid_joint[i]->u() = control_effort;
            } else {
                // 位置制御の場合
                //noid_joint[i]->q_target() = joint_target_q.data[i];
                double max_step = 0.1; // 1ステップでの最大変位（必要に応じ調整）
                double desired = joint_target_q.data[i];
                double current = cnoid_joint[i]->q();
                double diff = desired - current;
                if (std::abs(diff) > max_step) {
                    desired = current + std::copysign(max_step, diff);
                }
                cnoid_joint[i]->q_target() = desired;
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
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(body_JointController)
