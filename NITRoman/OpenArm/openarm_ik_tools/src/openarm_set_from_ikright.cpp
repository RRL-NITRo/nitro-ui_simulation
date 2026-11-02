#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <memory>
#include <arm_msgs/msg/arm_command.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class IKCalculator : public rclcpp::Node
{
public:
    IKCalculator(rclcpp::Node::SharedPtr node) : rclcpp::Node("ik_calculator"), node_(node)
    {
        robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(node_, "robot_description");  // ← パラメータ名を指定

        kinematic_model_ = robot_model_loader_->getModel();
        robot_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model_);
        robot_state_->setToDefaultValues();
        joint_model_group_ = kinematic_model_->getJointModelGroup("right_arm");
        

        if (!kinematic_model_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load kinematic model");
        }
        if (!joint_model_group_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get joint model group");
        }
        

        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/target/joint_states/arm_right", 10);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/calculated_pose/right", 10);
        twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/arm_twist/right", 1, std::bind(&IKCalculator::twistCallback, this, std::placeholders::_1));
       // pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/arm_pose/right", 1, std::bind(&IKCalculator::poseCallback, this, std::placeholders::_1));
        command_sub_ = this->create_subscription<arm_msgs::msg::ArmCommand>("/arm_command/right", 1, std::bind(&IKCalculator::ArmCommandCallback, this, std::placeholders::_1));
        //sub_gripper = this->create_subscription<std_msgs::msg::Float32>("/gripper_target/right", 10, std::bind(&IKCalculator::gripperCallback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / 10)), std::bind(&IKCalculator::timerCallback_calcik, this));
        printJointValues();
        checkJointLimits();
       
        computeForwardKinematics(initial_joint_angles_);

        Eigen::Isometry3d end_effector_state = getEndEffectorState();
        computeInverseKinematics(end_effector_state, 0.1);
        getJacobian(Eigen::Vector3d(0.0, 0.0, 0.0));
        registerTargetJointStates();
        publishEndEffectorState();

        RCLCPP_INFO(this->get_logger(), "Available groups:");
        for (auto &group_name : kinematic_model_->getJointModelGroupNames()) {
            RCLCPP_INFO(this->get_logger(), "  %s", group_name.c_str());
        }

        RCLCPP_INFO(this->get_logger(), "Links in group %s:", joint_model_group_->getName().c_str());
        for (auto &link_name : joint_model_group_->getLinkModelNames()) {
            RCLCPP_INFO(this->get_logger(), "  %s", link_name.c_str());
        }

        joint_angles_before = initial_joint_angles_;
        joint_angles_registered = initial_joint_angles_;
        joint_angles_calculated = initial_joint_angles_;
        publishJointStates();

    }

    void printJointValues()
    {
        const std::vector<std::string>& joint_names = joint_model_group_->getVariableNames();
        std::vector<double> joint_values;
        robot_state_->copyJointGroupPositions(joint_model_group_, joint_values);
        for (std::size_t i = 0; i < joint_names.size(); ++i)
        {
            RCLCPP_INFO(node_->get_logger(), "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
        }
    }

    void checkJointLimits()
    {
        std::vector<double> joint_values;
        robot_state_->copyJointGroupPositions(joint_model_group_, joint_values);

        joint_values[0] = 5.57;
        robot_state_->setJointGroupPositions(joint_model_group_, joint_values);

        RCLCPP_INFO_STREAM(node_->get_logger(), "Current state is " << (robot_state_->satisfiesBounds() ? "valid" : "not valid"));

        robot_state_->enforceBounds();
        RCLCPP_INFO_STREAM(node_->get_logger(), "Current state is " << (robot_state_->satisfiesBounds() ? "valid" : "not valid"));
    }

    void computeForwardKinematics(const std::vector<double>& joint_angles)
    {
        // Set the joint angles to the robot state
        robot_state_->setJointGroupPositions(joint_model_group_, joint_angles);

        // Update the robot state to apply the joint angles
        robot_state_->update();

        // Get the end effector state
        const Eigen::Isometry3d& end_effector_state = robot_state_->getGlobalLinkTransform("openarm_right_hand_tcp");

        // Log the translation and rotation of the end effector
        RCLCPP_INFO_STREAM(node_->get_logger(), "Translation: \n" << end_effector_state.translation() << "\n");
        RCLCPP_INFO_STREAM(node_->get_logger(), "Rotation: \n" << end_effector_state.rotation() << "\n");
    }

    void computeInverseKinematics(const Eigen::Isometry3d& end_effector_state, double timeout)
    {
        // bool found_ik = robot_state_->setFromIK(joint_model_group_, end_effector_state, timeout);
        bool found_ik = robot_state_->setFromIK(joint_model_group_, end_effector_state, "openarm_right_hand_tcp", 0.1);
        if (found_ik)
        {
            printJointValues();
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "Did not find IK solution");
        }
    }

    void getJacobian(const Eigen::Vector3d& reference_point_position)
    {
        Eigen::MatrixXd jacobian;
        robot_state_->getJacobian(joint_model_group_,
                                  robot_state_->getLinkModel(joint_model_group_->getLinkModelNames().back()),
                                  reference_point_position, jacobian);
        RCLCPP_INFO_STREAM(node_->get_logger(), "Jacobian: \n" << jacobian << "\n");
    }

    Eigen::Isometry3d getEndEffectorState()
    {
        return robot_state_->getGlobalLinkTransform("openarm_right_hand_tcp");
    }

    bool calculateIK(const geometry_msgs::msg::Pose& desired_pose)
    {
        bool found_ik = robot_state_->setFromIK(joint_model_group_, desired_pose, "openarm_right_hand_tcp");


        if (found_ik)
        {
            printJointValues();
            registerTargetJointStates();  // ← ここで必ず出す
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "IK solution not found.");
        }

        return found_ik;
    }
    void publishJointStates()
    {
            sensor_msgs::msg::JointState joint_state_msg;
            joint_state_msg.header.stamp = this->now();
            joint_state_msg.name = joint_model_group_->getVariableNames();
            joint_state_msg.position = joint_angles_calculated;

            // Check if joint values are within limits before publishing
            // for (size_t i = 0; i < joint_state_msg.position.size(); ++i)
            // {
            //     if (joint_state_msg.position[i] < lower_limits[i] || joint_state_msg.position[i] > upper_limits[i])
            //     {
            //     RCLCPP_WARN(this->get_logger(), "Joint %s is out of limits: %f", joint_state_msg.name[i].c_str(), joint_state_msg.position[i]);
            //     return; // Do not publish if any joint is out of limits
            //     }
            // }
            // RCLCPP_INFO(this->get_logger(), "Publishing JointState:");
            // for (size_t i = 0; i < joint_state_msg.name.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "  %s: %f", joint_state_msg.name[i].c_str(), joint_state_msg.position[i]);
            // }
            joint_state_pub_->publish(joint_state_msg);
            joint_angles_before = joint_angles_calculated;
    }
    void publishEndEffectorState()
    {
        geometry_msgs::msg::Pose end_effector_pose;
        Eigen::Isometry3d end_effector_state = getEndEffectorState();

        end_effector_pose.position.x = end_effector_state.translation().x();
        end_effector_pose.position.y = end_effector_state.translation().y();
        end_effector_pose.position.z = end_effector_state.translation().z();

        Eigen::Quaterniond orientation(end_effector_state.rotation());
        end_effector_pose.orientation.x = orientation.x();
        end_effector_pose.orientation.y = orientation.y();
        end_effector_pose.orientation.z = orientation.z();
        end_effector_pose.orientation.w = orientation.w();

        pose_pub_->publish(end_effector_pose);
    }

    
    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr twist_msg)
    {
         RCLCPP_INFO(this->get_logger(), "Received Twist message");
        twist_msg_log = *twist_msg;
        // Get the current end effector state
        Eigen::Isometry3d current_end_effector_state = getEndEffectorState();

        // Extract the translation and rotation from the current end effector state
        Eigen::Vector3d current_translation = current_end_effector_state.translation();
        Eigen::Quaterniond current_rotation(current_end_effector_state.rotation());
        Eigen::Vector3d linear_twist(twist_msg->linear.x, twist_msg->linear.y, twist_msg->linear.z);
        if(end_effector_direction_mode){
            linear_twist.x() = twist_msg->linear.z;
            linear_twist.y() = -twist_msg->linear.y;
            linear_twist.z() = twist_msg->linear.x;
            linear_twist = current_rotation * linear_twist;
        }

        current_translation += linear_twist * loop_time;

        // Update the rotation based on the angular twist
        Eigen::Quaterniond delta_rotation(
            Eigen::AngleAxisd(twist_msg->angular.x, Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(twist_msg->angular.y, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(twist_msg->angular.z, Eigen::Vector3d::UnitZ()));
        Eigen::Quaterniond new_rotation = current_rotation * delta_rotation;

        // Construct the new end effector state
        Eigen::Isometry3d new_end_effector_state;
        new_end_effector_state.translation() = current_translation;
        new_end_effector_state.linear() = new_rotation.toRotationMatrix();
        RCLCPP_INFO(this->get_logger(), "New Position: x=%f, y=%f, z=%f", current_translation.x(), current_translation.y(), current_translation.z());

        // Convert the new end effector state to a geometry_msgs::msg::Pose
        geometry_msgs::msg::Pose new_pose;
        new_pose.position.x = new_end_effector_state.translation().x();
        new_pose.position.y = new_end_effector_state.translation().y();
        new_pose.position.z = new_end_effector_state.translation().z();
        new_pose.orientation.x = new_rotation.x();
        new_pose.orientation.y = new_rotation.y();
        new_pose.orientation.z = new_rotation.z();
        new_pose.orientation.w = new_rotation.w();

        RCLCPP_INFO(this->get_logger(), "New Orientation: x=%f, y=%f, z=%f, w=%f", new_rotation.x(), new_rotation.y(), new_rotation.z(), new_rotation.w());

        // Call the IK function with the new pose
        calculateIK(new_pose);
        registerTargetJointStates();
    }

    bool end_effector_direction_mode = false;
    void ArmCommandCallback(const arm_msgs::msg::ArmCommand::SharedPtr msg)
    {
        std::vector<double> target_rad(7, 0.0);
        std::vector<int> activate_joint(7, 0);
        arm_msg_log = *msg;

        if (msg->gripper_open){
            gripper_open = true;
            gripper_close = false;
        }
        else if (msg->gripper_close){
            gripper_open = false;
            gripper_close = true;
        }

        end_effector_direction_mode = msg->end_effector_direction_mode;
        if (msg->reset_home_position)
        {
            reset_joint_loop_counter++;

            if (reset_joint_loop_counter > 30)
            {
                target_rad = initial_joint_angles_;
                activate_joint = {1, 1, 1, 1, 1, 1, 1};
                setFromFK(target_rad, activate_joint);
            }
            else if (reset_joint_loop_counter > 0)
            {
                target_rad = initial_joint_angles_;
                activate_joint = {1, 1, 1, 1, 1, 1, 1};
                setFromFK(target_rad, activate_joint);
            }
        }
        else if (msg->reset_end_posture)
        {
            reset_end_loop_counter++;

            if (reset_end_loop_counter > 30)
            {
                ResetHandPose();
            }
            else if (reset_end_loop_counter > 0)
            {
                target_rad = {0, 0, 0, 0, 0, 0.0, 0};
                activate_joint = {0, 0, 0, 0, 0, 1, 0};
                setFromFK(target_rad, activate_joint);
            }
        }
        else
        {
            reset_joint_loop_counter = 0;
            reset_end_loop_counter = 0;
        }
    }

    void ResetHandPose()
    {
        geometry_msgs::msg::Pose current_pose;
        const Eigen::Isometry3d& end_effector_transform = robot_state_->getGlobalLinkTransform(joint_model_group_->getLinkModelNames().back());

        current_pose = eigenAffine3dToGeometryPose(end_effector_transform);
        current_pose.orientation.x = 0;
        current_pose.orientation.y = 0;
        current_pose.orientation.z = 0;
        current_pose.orientation.w = 1;

        std::vector<double> joint_positions;
        computeInverseKinematics(end_effector_transform, 0.1);
    }
    geometry_msgs::msg::Pose eigenAffine3dToGeometryPose(const Eigen::Affine3d &eigen_affine)
    {
        geometry_msgs::msg::Pose pose_msg;

        // 変換行列から位置を取得
        pose_msg.position.x = eigen_affine.translation().x();
        pose_msg.position.y = eigen_affine.translation().y();
        pose_msg.position.z = eigen_affine.translation().z();

        // 変換行列から姿勢を取得（クォータニオン形式）
        Eigen::Quaterniond quaternion(eigen_affine.rotation());
        pose_msg.orientation.x = quaternion.x();
        pose_msg.orientation.y = quaternion.y();
        pose_msg.orientation.z = quaternion.z();
        pose_msg.orientation.w = quaternion.w();

        return pose_msg;
    }
    void setFromFK(const std::vector<double>& joint_target_rad, const std::vector<int>& change_joint)
    {
        std::vector<double> original_joint_positions;
        robot_state_->copyJointGroupPositions(joint_model_group_, original_joint_positions);

        std::vector<double> target_joint_angles(7, 0.0);
        for (int i = 0; i < 7; ++i)
        {
            if (change_joint.at(i) == 1)
            {
                target_joint_angles.at(i) = joint_target_rad.at(i);
            }
            else if (change_joint.at(i) == 0)
            {
                target_joint_angles.at(i) = original_joint_positions.at(i);
            }
        }

        robot_state_->setJointGroupPositions(joint_model_group_, target_joint_angles);
        registerTargetJointStates();
    }

    void registerTargetJointStates()
    {
        robot_state_->copyJointGroupPositions(joint_model_group_, joint_angles_registered);
    }

    void onlineJointControl()
    {
        for (size_t i = 0; i < joint_angles_registered.size(); ++i)
        {
            double diff = joint_angles_registered[i] - joint_angles_before[i];
            double max_step = joint_vel_limits[i] * loop_time; // Maximum change based on velocity limit

            if (std::abs(diff) > max_step)
            {
                if (diff > 0)
                    joint_angles_calculated[i] = joint_angles_before[i] + max_step;
                else
                    joint_angles_calculated[i] = joint_angles_before[i] - max_step;
            }
            else
            {
                joint_angles_calculated[i] = joint_angles_registered[i];
            }
        }
        
        publishJointStates();
    }

    void timerCallback_calcik()
    {
        // RCLCPP_INFO(this->get_logger(), "Timer callback");
        publishEndEffectorState();
        onlineJointControl();
     
    }
private:
    rclcpp::Node::SharedPtr node_;
    moveit::core::RobotModelPtr kinematic_model_;
    moveit::core::RobotStatePtr robot_state_;
    const moveit::core::JointModelGroup* joint_model_group_;
    geometry_msgs::msg::Twist twist_msg_log;
    std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<arm_msgs::msg::ArmCommand>::SharedPtr command_sub_;
    // rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_gripper;
    rclcpp::TimerBase::SharedPtr timer_;
    double loop_time = 0.1; 
    std::vector<double> lower_limits = {-M_PI*200/180, -M_PI*190/180, -M_PI/2, 0, -M_PI/2, -M_PI/4, -M_PI/2};
    std::vector<double> upper_limits = {M_PI*80/180, M_PI*10/180, M_PI/2, M_PI*140/180, M_PI/2, M_PI/4, M_PI/2};
    std::vector<double> joint_vel_limits = {3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0}; // rad/s
    arm_msgs::msg::ArmCommand arm_msg_log;
    bool reset_home_position = false;
    bool reset_end_posture = false;
    int reset_joint_loop_counter = 0;
    int reset_end_loop_counter = 0;
    bool gripper_open = false;
    bool gripper_close = false;
    std::vector<double> initial_joint_angles_= {-0.868, 0, 0, 2.443, 0, 0, 0};
    std::vector<double> joint_angles_calculated; //target of temporary
    std::vector<double> joint_angles_before;    //temp for computation
    std::vector<double> joint_angles_registered; //target of final

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("ik_solver", node_options);
    auto robot_manager = std::make_shared<IKCalculator>(node);
    rclcpp::spin(robot_manager);
    rclcpp::shutdown();
    return 0;
}
