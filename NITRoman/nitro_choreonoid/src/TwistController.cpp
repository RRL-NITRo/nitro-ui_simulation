#include <cnoid/SimpleController>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cnoid/BodyItem>
#include <cnoid/SimulatorItem>
#include <mutex>
#include <vector>
#include <thread>

using namespace cnoid;

class TwistController : public SimpleController
{
    std::shared_ptr<rclcpp::Node> node;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist;
    std::vector<Link*> cnoid_track;
    std::vector<std::string> left_wheel_names = {
        "left_wheel_front", 
        "left_wheel_center", "left_wheel_back",
        "left_front_wheel_front", "left_front_wheel_center", "left_front_wheel_back",
        "left_rear_wheel_front", "left_rear_wheel_center", "left_rear_wheel_back"
    };

    std::vector<std::string> right_wheel_names = {
        "right_wheel_front", 
        "right_wheel_center", "right_wheel_back",
        "right_front_wheel_front", "right_front_wheel_center", "right_front_wheel_back",
        "right_rear_wheel_front", "right_rear_wheel_center", "right_rear_wheel_back"
    };
    geometry_msgs::msg::Twist command;
    std::mutex commandMutex;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
    std::thread executorThread;

public:
    virtual bool configure(SimpleControllerConfig* config) override
    {
        node = rclcpp::Node::make_shared(config->controllerName());
        sub_twist = node->create_subscription<geometry_msgs::msg::Twist>(
            "/target/cmd_vel", 1, std::bind(&TwistController::callback_twist, this, std::placeholders::_1)
        );

        executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor->add_node(node);
        executorThread = std::thread([this](){ executor->spin(); });

        return true;
    }

    virtual bool initialize(SimpleControllerIO* io) override
    {
        Body* body = io->body();
        for (const auto& name : left_wheel_names) {
            Link* link = body->link(name);
            if (link) {
                cnoid_track.push_back(link);
            }
        }
        for (const auto& name : right_wheel_names) {
            Link* link = body->link(name);
            if (link) {
                cnoid_track.push_back(link);
            }
        }

        for (size_t i = 0; i < left_wheel_names.size(); ++i) {
            cnoid_track[i]->setActuationMode(JointVelocity);
            io->enableInput(cnoid_track[i], JointVelocity);
            io->enableOutput(cnoid_track[i], JointVelocity);
                // cnoid_track[i]->setActuationMode(JointEffort);
                // io->enableInput(cnoid_track[i], JointEffort);
                // io->enableOutput(cnoid_track[i], JointEffort);

        }
        for (size_t i = 0; i < right_wheel_names.size(); ++i) {
            cnoid_track[i + left_wheel_names.size()]->setActuationMode(JointVelocity);
            io->enableInput(cnoid_track[i + left_wheel_names.size()], JointVelocity);
            io->enableOutput(cnoid_track[i + left_wheel_names.size()], JointVelocity);
                // cnoid_track[i + left_wheel_names.size()]->setActuationMode(JointEffort);
                // io->enableInput(cnoid_track[i + left_wheel_names.size()], JointEffort);
                // io->enableOutput(cnoid_track[i + left_wheel_names.size()], JointEffort);
        }

        return true;
    }

    void callback_twist(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(commandMutex);
        command = *msg;
    }

    virtual bool control() override
    {
        constexpr double kd = 0.05;
        double velocity_left, velocity_right;
        double wheel_radius = 0.1;

        {
            std::lock_guard<std::mutex> lock(commandMutex);
            double wheels_width = 0.5; // Simulation value, not actual.
            double linear_velocity = command.linear.x;
            double angular_velocity = command.angular.z;
            velocity_left = (linear_velocity - 5*(angular_velocity * wheels_width / 2));
            velocity_right = (linear_velocity + 5*(angular_velocity * wheels_width / 2));
        }

        // 左ホイールの制御
        for (size_t i = 0; i < left_wheel_names.size(); ++i) {
            Link* joint = cnoid_track[i];
            joint->dq_target()=velocity_left;
           // joint->u() = std::clamp(kd * (velocity_left - joint->dq()), -10.0, 10.0);
        }

        // 右ホイールの制御
        for (size_t i = 0; i < right_wheel_names.size(); ++i) {
            Link* joint = cnoid_track[i + left_wheel_names.size()];
            joint->dq_target()=velocity_right;
          //  joint->u() = std::clamp(kd * (velocity_right - joint->dq()), -10.0, 10.0);
        }

        return true;
    }

    virtual void unconfigure() override
    {
        if (executor) {
            executor->cancel();
            executorThread.join();
            executor->remove_node(node);
            executor.reset();
        }
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(TwistController)