#include <rclcpp/rclcpp.hpp>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/convert.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <arm_msgs/msg/arm_command.hpp>
#include <std_msgs/msg/float32.hpp>

class KinematicsControl : public rclcpp::Node
{
public:
   KinematicsControl()
    : Node("kinematics_control"), linear_vel_(0.025), angular_vel_(0.05), use_twist_(true)
{
    joint_angles_before.resize(6);
    joint_angles_calculated.resize(6);

    // Initialize the node parameters and publishers/subscribers
    this->declare_parameter("planning_group_name", "imax");
    this->get_parameter("planning_group_name", planning_group_name_);

    this->declare_parameter("initial_joint_angles", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    this->get_parameter("initial_joint_angles", initial_joint_angles_);

    // Publishers and subscribers initialization
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/calculated_pose", 10);
    twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/arm_twist", 1, std::bind(&KinematicsControl::twistCallback, this, std::placeholders::_1));
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/arm_pose", 1, std::bind(&KinematicsControl::poseCallback, this, std::placeholders::_1));
    command_sub_ = this->create_subscription<arm_msgs::msg::ArmCommand>("/arm_command", 1, std::bind(&KinematicsControl::ArmCommandCallback, this, std::placeholders::_1));
    sub_gripper = this->create_subscription<std_msgs::msg::Float32>("/gripper_target", 10, std::bind(&KinematicsControl::gripperCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / loop_hz)), std::bind(&KinematicsControl::timerCallback_calcik, this));

}

void setupRobotModelLoader()
{
    robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(shared_from_this(), "robot_description");
    kinematic_model_ = robot_model_loader_->getModel();
    joint_model_group_ = kinematic_model_->getJointModelGroup(planning_group_name_);
    kinematic_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model_);
    kinematic_state_->setJointGroupPositions(joint_model_group_, initial_joint_angles_);

    // Get joint limits
    const auto& joint_models = joint_model_group_->getJointModels();
    for (const auto& joint_model : joint_models)
    {
        const auto& bounds = joint_model->getVariableBounds();
        for (const auto& bound : bounds)
        {
            lower_limits.push_back(bound.min_position_);
            upper_limits.push_back(bound.max_position_);
        }
    }
}

    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr twist_msg)
    {
        twist_msg_log = *twist_msg;
    }

    void ArmCommandCallback(const arm_msgs::msg::ArmCommand::SharedPtr arm_msg)
    {
        // Arm command handling code
    }
    void gripperCallback(const std_msgs::msg::Float32::SharedPtr gripper_msg)
    {
        double gripper_target = gripper_msg->data;
        // Assuming the gripper joint is the last joint in the joint model group
        std::vector<double> joint_positions;
        kinematic_state_->copyJointGroupPositions(joint_model_group_, joint_positions);
        joint_positions.back() = gripper_target; // Set the gripper joint position

        // Enforce joint limits
        enforceJointLimits(joint_positions);

        // Update the kinematic state with the new joint positions
        kinematic_state_->setJointGroupPositions(joint_model_group_, joint_positions);

        // Publish the updated joint state
        publishJointState(false);
    }
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg)
    {
        const std::vector<std::string>& joint_names = joint_model_group_->getVariableNames();
        std::vector<double> joint_positions;

        geometry_msgs::msg::Pose arm_base_2_link_gripper = pose_msg->pose;
        arm_base_2_link_gripper.position.z += 0.213; // Adjust for gripper position
        arm_base_2_link_gripper.position.x += 0.22;

        if (computeIKWithJointLimits(arm_base_2_link_gripper, joint_positions))
        {
            kinematic_state_->setJointGroupPositions(joint_model_group_, joint_positions);

            pose_pub_->publish(arm_base_2_link_gripper);
        }
    }

    // Placeholder for IK calculation method
    bool computeIKWithJointLimits(const geometry_msgs::msg::Pose& target_pose, std::vector<double>& joint_positions)
    {
        // IK calculation logic
        return true; // Return true if successful
    }

    void timerCallback_calcik()
    {
        publishJointState(true);
    }

    void enforceJointLimits(std::vector<double>& joint_positions)
    {
        for (size_t i = 0; i < joint_positions.size(); ++i)
        {
            joint_positions[i] = std::max(lower_limits[i], std::min(upper_limits[i], joint_positions[i]));
        }
    }

    void publishJointState(bool use_reset)
    {
        sensor_msgs::msg::JointState joint_state_msg;
        joint_state_msg.header.stamp = this->now();
        joint_state_msg.name = joint_model_group_->getVariableNames();
        kinematic_state_->copyJointGroupPositions(joint_model_group_, joint_angles_calculated);
        joint_state_msg.position = joint_angles_calculated;

        joint_state_pub_->publish(joint_state_msg);
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<arm_msgs::msg::ArmCommand>::SharedPtr command_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_gripper;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
    std::shared_ptr<moveit::core::RobotModel> kinematic_model_;
    std::shared_ptr<moveit::core::RobotState> kinematic_state_;
    const moveit::core::JointModelGroup* joint_model_group_;

    std::vector<double> joint_angles_before;
    std::vector<double> joint_angles_calculated;
    std::vector<double> lower_limits;
    std::vector<double> upper_limits;
    std::vector<double> initial_joint_angles_;
    std::string planning_group_name_;
    double linear_vel_;
    double angular_vel_;
    bool use_twist_;
    geometry_msgs::msg::Twist twist_msg_log;
    arm_msgs::msg::ArmCommand arm_msg_log;
    double loop_hz = 30.0;  // Define the loop frequency
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KinematicsControl>();

    // Call the setup method after the node has been constructed
    node->setupRobotModelLoader();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}