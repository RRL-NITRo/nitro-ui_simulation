#include <memory>
#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "arm_msgs/msg/body_command.hpp"

class TwoDoFArmController : public rclcpp::Node
{
public:
    TwoDoFArmController()
    : Node("two_dof_arm_controller")
    {   
        this->declare_parameter("l1", 0.2);
        this->declare_parameter("l2", 0.2);
        l1_ = this->get_parameter("l1").as_double();
        l2_ = this->get_parameter("l2").as_double();

        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/target/joint_states/body", 10);

        sub_= this->create_subscription<arm_msgs::msg::BodyCommand>(
            "/body_command", 10,
            std::bind(&TwoDoFArmController::bodyCommandCallback, this, std::placeholders::_1)
        );
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TwoDoFArmController::timerCallback, this)
        );

        theta_ << 0.0, 0.0; // アーム初期角度
        theta_body_yaw_ = 0.0;
        theta_neck_yaw_ = 0.0;
        theta_head_yaw_ = 0.0;
        theta_head_pitch_ = 0.0;

        
        joint_angles_calculated.resize(6);
        joint_angles_before.resize(6);
        joint_angles_registered.resize(6);
        registerTargetJointStates();
        joint_angles_before = joint_angles_calculated;
        joint_angles_registered = joint_angles_calculated;
        publishJointState();
    }

private:
    double target_z_ = 0.0; // アームの目標高さ
    double theta_body_yaw_;
    double theta_neck_yaw_;
    double theta_head_yaw_;
    double theta_head_pitch_;

    void bodyCommandCallback(const arm_msgs::msg::BodyCommand::SharedPtr msg)
    {
        if(msg->reset_home_position) {
            setFromFK(0.0, M_PI/6, M_PI/6, 0.0, 0.0, 0.0);
            registerTargetJointStates();
            return;
        }
        if(msg->reset_head_posture) {
            theta_head_yaw_ = 0.0;
            theta_head_pitch_ = 0.0;
            registerTargetJointStates();
            return;
        }
        
        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = msg->twist_for_ik[0]; // x方向
        twist_msg.linear.y = msg->twist_for_ik[1]; // y方向 
        twist_msg.linear.z = msg->twist_for_ik[2]; // z方向
        twist_msg.angular.x = msg->twist_for_ik[3]; // roll
        twist_msg.angular.y = msg->twist_for_ik[4]; // pitch
        twist_msg.angular.z = msg->twist_for_ik[5]; // body yaw
        bodyTwistCallback(std::make_shared<geometry_msgs::msg::Twist>(twist_msg));

        geometry_msgs::msg::Twist head_twist_msg;
        head_twist_msg.angular.x = msg->head_angular_velocity[0]; // head pitch
        head_twist_msg.angular.y = msg->head_angular_velocity[1]; // not used
        head_twist_msg.angular.z = msg->head_angular_velocity[2]; // head yaw
        headTwistCallback(std::make_shared<geometry_msgs::msg::Twist>(head_twist_msg));

    }
    // --- /body_twist の処理 ---
    void bodyTwistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double x_fixed = 0.0;

        // linear.z を高さの変化量として扱う
        double delta_z = msg->linear.z;
        target_z_ += delta_z;

        // 安全に高さを制限
        double max_height = l1_ + l2_;
        if (target_z_ < 0.0) target_z_ = 0.0;
        if (target_z_ > max_height) target_z_ = max_height;

        Eigen::Vector2d target(x_fixed, target_z_);

        // 順運動学
        double t1 = theta_(0), t2 = theta_(1);
        Eigen::Vector2d current;
        current(0) = -l1_*cos(t1) + l2_*cos(t2-t1);
        current(1) =  l1_*sin(t1) + l2_*sin(t2-t1);

        // ヤコビアン
        Eigen::Matrix2d J;
        J(0,0) = l1_*sin(t1) + l2_*sin(t2-t1);
        J(0,1) = -l2_*sin(t2-t1);
        J(1,0) =  l1_*cos(t1) - l2_*cos(t2-t1);
        J(1,1) =  l2_*cos(t2-t1);

        // Δθを計算
        Eigen::Vector2d delta_theta = J.completeOrthogonalDecomposition().pseudoInverse() * (target - current);
        theta_ += delta_theta;

        // アームの関節角度を制限
        for(int i=0; i<2; i++) {
            if(theta_(i) < 0.0) theta_(i) = 0.0;
            if(theta_(0) > M_PI/2) theta_(0) = M_PI/2;
            if(theta_(1) > M_PI) theta_(1) = M_PI;
        }

        // --- body yaw 更新 ---
        theta_body_yaw_ += msg->angular.z;
        if (theta_body_yaw_ > M_PI)  theta_body_yaw_ = M_PI;
        if (theta_body_yaw_ < -M_PI) theta_body_yaw_ = -M_PI;

        // --- neck yaw 更新 ---
        theta_neck_yaw_ = t1-t2;
        if (theta_neck_yaw_ > M_PI/2)  theta_neck_yaw_ = M_PI/2;
        if (theta_neck_yaw_ < -M_PI/2) theta_neck_yaw_ = -M_PI/2;

       // if (target_z_ < max_height*0.1) theta_neck_yaw_ = -M_PI/2; // 低い位置では首を下げる

        registerTargetJointStates();
    }

    // --- /head_twist の処理 ---
    void headTwistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // head yaw 更新
        theta_head_yaw_ += msg->angular.z;
        if (theta_head_yaw_ > M_PI/2)  theta_head_yaw_ = M_PI/2;
        if (theta_head_yaw_ < -M_PI/2) theta_head_yaw_ = -M_PI/2;

        // head pitch 更新
        theta_head_pitch_ += msg->angular.x;
        if (theta_head_pitch_ > M_PI/2)  theta_head_pitch_ = M_PI/2;
        if (theta_head_pitch_ < -M_PI/2) theta_head_pitch_ = -M_PI/2;

        registerTargetJointStates();
    }

    void setFromFK(const float& link_yaw, const float& first_joint, const float& second_joint, const float& neck_yaw, const float& head_yaw, const float& head_pitch)
    {
        theta_body_yaw_ = link_yaw;
        theta_(0) = first_joint;
        theta_(1) = second_joint;
        theta_neck_yaw_ = neck_yaw;
        theta_head_yaw_ = head_yaw;
        theta_head_pitch_ = head_pitch;

        target_z_ = l1_ * sin(theta_(0)) + l2_ * sin(theta_(1) - theta_(0));
      
    }

    // --- JointState をまとめてパブリッシュ ---
    void publishJointState()
    {
        auto joint_msg = sensor_msgs::msg::JointState();
        joint_msg.header.stamp = this->get_clock()->now();
        joint_msg.name = {
            "body_link_yaw_joint",
            "body_first_link_joint",
            "body_second_link_joint",
            "body_third_link_joint",
            "head_yaw_joint",
            "head_pitch_joint"
        };
        joint_msg.position = joint_angles_calculated;
        joint_pub_->publish(joint_msg);
        joint_angles_before = joint_angles_calculated;
    }

    void registerTargetJointStates(){
        joint_angles_registered[0] = theta_body_yaw_;
        joint_angles_registered[1] = theta_(0);
        joint_angles_registered[2] = theta_(1);
        joint_angles_registered[3] = theta_neck_yaw_;
        joint_angles_registered[4] = theta_head_yaw_;
        joint_angles_registered[5] = theta_head_pitch_;
    }

    void onlineJointControl(){
        for(size_t i=0; i<joint_angles_registered.size(); i++){
            double diff = joint_angles_registered[i] - joint_angles_before[i];
            double max_change = joint_vel_limits[i] * 0.1; // 0.1秒周期
            if(fabs(diff) > max_change){
                if(diff > 0) joint_angles_calculated[i] = joint_angles_before[i] + max_change;
                else         joint_angles_calculated[i] = joint_angles_before[i] - max_change;
            } else {
                joint_angles_calculated[i]=joint_angles_registered[i];
            }
        }
        publishJointState();
    }
    void timerCallback()
    {
        onlineJointControl();
    }

    double l1_, l2_;
    Eigen::Vector2d theta_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr body_twist_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr head_twist_sub_;
    rclcpp::Subscription<arm_msgs::msg::BodyCommand>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<double> joint_vel_limits={2.0, 2.0, 2.0, 2.0, 2.0, 2.0}; // rad/s
    std::vector<double> joint_angles_calculated; //target of temporary
    std::vector<double> joint_angles_before;    //temp for computation
    std::vector<double> joint_angles_registered; //target of final


};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TwoDoFArmController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
