#include <cnoid/SimpleController>
#include <cnoid/Link>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using namespace cnoid;

class PanTiltController : public SimpleController
{
    Link* panJoint;
    Link* tiltJoint;

    double targetPan = 0.0;
    double targetTilt = 0.0;
    double kp = 100.0;
    double kd = 10;
    double dt;

    rclcpp::Node::SharedPtr node;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub;

public:
    bool initialize(SimpleControllerIO* io) override
    {
        panJoint = io->body()->link("link_Pan");
        tiltJoint = io->body()->link("link_Tilt");

        panJoint->setActuationMode(Link::JOINT_ANGLE);
        tiltJoint->setActuationMode(Link::JOINT_ANGLE);
        io->enableOutput(panJoint);
        io->enableOutput(tiltJoint);

        dt = io->timeStep();

        // 初期角度を現在に合わせる
        targetPan = panJoint->q();
        targetTilt = tiltJoint->q();

        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        node = rclcpp::Node::make_shared("PanTiltcontroller");

        sub = node->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/PanTilt_target", 10,
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                if (msg->data.size() >= 2) {
                    targetPan = msg->data[0];
                    targetTilt = msg->data[1];
                }
            });

        return true;
    }

    bool control() override
    {
        rclcpp::spin_some(node);

        // 位置目標を設定（内部PD制御）
        panJoint->q_target() = targetPan;
        tiltJoint->q_target() = targetTilt;

        return true;
    }


    void stop() override
    {
        rclcpp::shutdown();
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(PanTiltController)