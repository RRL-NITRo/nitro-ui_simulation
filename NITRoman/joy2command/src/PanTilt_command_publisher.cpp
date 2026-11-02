#include <rclcpp/rclcpp.hpp>
#include <ps5_msgs/msg/ps5_joy.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class PanTiltCommandPublisher : public rclcpp::Node
{
public:
    PanTiltCommandPublisher()
        : Node("PanTilt_command_publisher"),
          target_pan_(0.0), target_tilt_(0.0)
    {
        ps5_sub_ = this->create_subscription<ps5_msgs::msg::PS5Joy>(
            "/ps5_joy", 10, std::bind(&PanTiltCommandPublisher::ps5Callback, this, std::placeholders::_1));

        pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/PanTilt_target", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),  // 50Hz
            std::bind(&PanTiltCommandPublisher::update, this));
    }

private:
    void ps5Callback(const ps5_msgs::msg::PS5Joy::SharedPtr msg)
    {
        input_pan_ = msg->cross_left;   // -1.0〜1.0
        input_tilt_ = msg->cross_up;    // -1.0〜1.0
        if (msg->l2) {  // ← L2トリガーが押されたら
            target_pan_ = 0.0;
            target_tilt_ = 0.0;
            RCLCPP_INFO(this->get_logger(), "PanTilt angles reset to 0 by L2.");
        }
    }

    void update()
    {
        double dt = 0.02;  // 秒（50Hz想定）

        // 目標角度を更新（相対指令）
        target_pan_ += input_pan_ * pan_speed_ * dt;
        target_tilt_ -= input_tilt_ * tilt_speed_ * dt;

        // 制限角度（安全対策）
        target_pan_ = std::clamp(target_pan_, -max_pan_, max_pan_);
        target_tilt_ = std::clamp(target_tilt_, -max_tilt_, max_tilt_);

        std_msgs::msg::Float64MultiArray cmd;
        cmd.data = {target_pan_, target_tilt_};
        pub_->publish(cmd);
    }

    // 入力
    double input_pan_ = 0.0;
    double input_tilt_ = 0.0;

    // 状態
    double target_pan_;
    double target_tilt_;

    // パラメータ
    const double pan_speed_ = 1.5;    // rad/s
    const double tilt_speed_ = 1.5;   // rad/s
    const double max_pan_ = 1.3;      // rad
    const double max_tilt_ = 1.3;     // rad

    rclcpp::Subscription<ps5_msgs::msg::PS5Joy>::SharedPtr ps5_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PanTiltCommandPublisher>());
    rclcpp::shutdown();
    return 0;
}
