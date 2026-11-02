#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <arm_msgs/msg/arm_command.hpp>
#include <arm_msgs/msg/body_command.hpp>
#include <ps5_msgs/msg/ps5_joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <flipper_msgs/msg/flipper.hpp>

class Joy2Command : public rclcpp::Node {
public:
    Joy2Command() : Node("ps5_to_ros_command") {
        loadParameters();
        initPublishers();
        ps5_sub_ = this->create_subscription<ps5_msgs::msg::PS5Joy>(
            "/ps5_joy", 1, std::bind(&Joy2Command::ps5Callback, this, std::placeholders::_1));
        
    }

    void initPublishers() {
        arm_command_left_pub_ = this->create_publisher<arm_msgs::msg::ArmCommand>("/arm_command/left", 1);
        arm_command_right_pub_ = this->create_publisher<arm_msgs::msg::ArmCommand>("/arm_command/right", 1);
        arm_twist_left_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/arm_twist/left", 1);
        arm_twist_right_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/arm_twist/right", 1);
        gripper_left_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/gripper_target/left", 1);
        gripper_right_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/gripper_target/right", 1);

        body_command_pub_ = this->create_publisher<arm_msgs::msg::BodyCommand>("/body_command", 1);

        joy_mode_pub_ = this->create_publisher<std_msgs::msg::String>("/joy_mode_str", 1);
        arm_mode_str_pub_ = this->create_publisher<std_msgs::msg::String>("/arm_mode_str", 1);
        twiststamped_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/operation_info/cmd_vel", 1);
        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/target/cmd_vel", 1);
        flipper_command_pub_ = this->create_publisher<flipper_msgs::msg::Flipper>("/flipper_command", 1);

        pub_flag_ = this->create_publisher<std_msgs::msg::Bool>("/flag", 1);
    }
    
    void loadParameters() {
        this->declare_parameter("crawler.linear.x.max_velocity", 2.0);
        this->declare_parameter("crawler.angular.z.max_velocity", 6.28);

        this->declare_parameter("manipulator.max_linear_velocity", 0.1);
        this->declare_parameter("manipulator.max_angular_velocity", 0.1);

        this->declare_parameter("flipper.max_velocity", 0.1);
        this->declare_parameter("flipper.min_velocity", 0.1);

        this->declare_parameter("body.max_linear_velocity", 0.1);
        this->declare_parameter("body.max_angular_velocity", 0.1);
        this->declare_parameter("head.max_linear_velocity", 0.1);
        this->declare_parameter("head.max_angular_velocity", 0.1);

        this->get_parameter("crawler.linear.x.max_velocity", crawler_linear_x_max_velocity_);
        this->get_parameter("crawler.angular.z.max_velocity", crawler_angular_z_max_velocity_);

        this->get_parameter("manipulator.max_linear_velocity", manipulator_max_linear_velocity_);
        this->get_parameter("manipulator.max_angular_velocity", manipulator_max_angular_velocity_);

        this->get_parameter("flipper.max_velocity", flipper_max_velocity_);
        this->get_parameter("flipper.min_velocity", flipper_min_velocity_);

        this->get_parameter("body.max_linear_velocity", body_max_linear_velocity_);
        this->get_parameter("body.max_angular_velocity", body_max_angular_velocity_);

        this->get_parameter("head.max_linear_velocity", head_max_linear_velocity_);
        this->get_parameter("head.max_angular_velocity", head_max_angular_velocity_);
    }

    void ps5Callback(const ps5_msgs::msg::PS5Joy::SharedPtr msg) {
        createTwistMsg(msg);
        if (msg->command_l) {
            toggleArmMode(msg);
        }

        if (msg->joy_l && !old_msg_.joy_l) {
            is_back_mode = !is_back_mode;
            RCLCPP_INFO(this->get_logger(), "Back mode %s", is_back_mode ? "enabled" : "disabled");
        }

        if (is_arm_mode_) {
            armCommandPublisher(msg);
            armStringPublisher(msg);
        } else {
            flipperCommandPublisher(msg);
            flipperStringPublisher(msg);
        }
        old_msg_ = *msg;
    }

    void toggleArmMode(const ps5_msgs::msg::PS5Joy::SharedPtr &msg) {
        if (msg->command_l && !old_msg_.command_l) {
            is_arm_mode_ = !is_arm_mode_;
            std::string mode_str = (is_arm_mode_ ? "[Manipulator]\n" : "[Flipper]\n");

    
            joy_mode_str_.data = mode_str;
            joy_mode_pub_->publish(joy_mode_str_);
            RCLCPP_INFO(this->get_logger(), "%s_MODE", is_arm_mode_ ? "ARM" : "FLIPPER");
        }
    }

    
    void armStringPublisher(const ps5_msgs::msg::PS5Joy::SharedPtr &msg) {
        std::string arm_mode_str = (end_effector_direction_mode_left
            ? "[End Effector Direction Mode ON]\n"
            : "[End Effector Direction Mode OFF]\n");
        
        if (dexterity_mode_) {
            arm_mode_str += "[Dexterity Mode ON]\n";
        } else {
            arm_mode_str += "[Dexterity Mode OFF]\n";
        }
        
        arm_mode_str_.data = arm_mode_str;
        arm_mode_str_pub_->publish(arm_mode_str_);
    }


    void createTwistMsg(const ps5_msgs::msg::PS5Joy::SharedPtr &msg) {
        auto twist_msg = geometry_msgs::msg::Twist();
        if (is_back_mode) {
            twist_msg.linear.x = -msg->left_joystick_y * crawler_linear_x_max_velocity_;
            twist_msg.angular.z = msg->left_joystick_x * crawler_angular_z_max_velocity_;
        } else {
            twist_msg.linear.x = msg->left_joystick_y * crawler_linear_x_max_velocity_;
            twist_msg.angular.z = msg->left_joystick_x * crawler_angular_z_max_velocity_;
        }
        twist_pub_->publish(twist_msg);

        auto twist_stamped_msg = geometry_msgs::msg::TwistStamped();
        twist_stamped_msg.header.stamp = this->now();
        twist_stamped_msg.twist = twist_msg;
        twiststamped_pub_->publish(twist_stamped_msg);
    }


    void bodyCommandPublisher(const ps5_msgs::msg::PS5Joy::SharedPtr &msg) {
        arm_msgs::msg::BodyCommand body_command_msg;
        body_command_msg.reset_home_position = msg->home;

        // twist_for_ik: std::array<float, 6>
        std::array<float, 6> xyzrpy = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        xyzrpy[2] = msg->cross_up * 0.5f * body_max_linear_velocity_;
        xyzrpy[5] = msg->cross_left * body_max_angular_velocity_;
        body_command_msg.twist_for_ik = xyzrpy;

        // head_angular_velocity: std::array<float, 3>
        std::array<float, 3> head_rpy = {0.0, 0.0, 0.0};
        head_rpy[0] = msg->right_joystick_y * -head_max_angular_velocity_;
        head_rpy[2] = msg->right_joystick_x * head_max_angular_velocity_;
        body_command_msg.head_angular_velocity = head_rpy;
        body_command_pub_->publish(body_command_msg);
        
    }

    
    enum TargetArm {LEFT, RIGHT, BOTH, BODY};
    bool end_effector_direction_mode_left = false;
    bool end_effector_direction_mode_right = false;
    bool dexterity_mode_ = false;

    void armCommandPublisher(const ps5_msgs::msg::PS5Joy::SharedPtr &msg) {

        
        if(msg->command_r && !old_msg_.command_r){
            dexterity_mode_ = !dexterity_mode_;
            RCLCPP_INFO(this->get_logger(), "Dexterity mode %s", dexterity_mode_ ? "enabled" : "disabled");
        }

        if(msg->square && !old_msg_.square){
            end_effector_direction_mode_left = !end_effector_direction_mode_left;
            end_effector_direction_mode_right = !end_effector_direction_mode_right;
            RCLCPP_INFO(this->get_logger(), "Left arm end effector direction mode %s", end_effector_direction_mode_left ? "enabled" : "disabled");
        }

        //L1+R1で両腕、L1で左腕、R1で右腕、どちらも押していなければBODY

        TargetArm target_arm;
        if(msg->l1 && msg->r1){
            target_arm = BOTH;
        }
        else if(msg->l1){
            target_arm = LEFT;
        }
        else if(msg->r1){
            target_arm = RIGHT;
        }
        else{
            bodyCommandPublisher(msg);
            return;
        }

        float dexterty_rate = dexterity_mode_ ? 0.5 : 1.0;

        auto arm_twist_msg = geometry_msgs::msg::Twist();
        arm_twist_msg.linear.x = msg->right_joystick_y*manipulator_max_linear_velocity_*dexterty_rate;
        arm_twist_msg.linear.y = msg->right_joystick_x*manipulator_max_linear_velocity_*dexterty_rate;
        arm_twist_msg.linear.z = (msg->triangle ? 1.0 : (msg->cross ? -1.0 : 0.0))*manipulator_max_linear_velocity_*dexterty_rate;
        arm_twist_msg.angular.x = (msg->l2 ? 1.0 : (msg->r2 ? -1.0 : 0.0))*-manipulator_max_angular_velocity_*dexterty_rate;
        arm_twist_msg.angular.y = msg->cross_up*-manipulator_max_angular_velocity_*dexterty_rate;
        arm_twist_msg.angular.z = msg->cross_left*-manipulator_max_angular_velocity_*dexterty_rate;

        bool HOMEPOSITION = msg->home;
        bool ENDEFFECTOR_RESET = msg->capture;

        auto arm_command_left_msg = arm_msgs::msg::ArmCommand();
        auto arm_command_right_msg = arm_msgs::msg::ArmCommand();
        
        auto arm_twist_left_msg = geometry_msgs::msg::Twist();
        auto arm_twist_right_msg = geometry_msgs::msg::Twist();
        sensor_msgs::msg::JointState gripper_left_msg;
        sensor_msgs::msg::JointState gripper_right_msg;

        
        if (target_arm == LEFT || target_arm == BOTH) {
            arm_twist_left_msg=arm_twist_msg;
            arm_command_left_msg.reset_home_position = HOMEPOSITION;
            arm_command_left_msg.reset_end_posture = ENDEFFECTOR_RESET;
            arm_command_left_msg.end_effector_direction_mode = end_effector_direction_mode_left;
            arm_command_left_pub_->publish(arm_command_left_msg);

            if (msg->circle && !old_msg_.circle) {
                gripper_state_left_ = !gripper_state_left_;
            }
            gripper_left_msg.header.stamp = this->now();
            gripper_left_msg.name =  {"openarm_left_finger_joint1", "openarm_left_finger_joint2"};
            gripper_left_msg.position = {gripper_state_left_ ? 0.044 : 0.0, gripper_state_left_ ? 0.044 : 0.0};
            gripper_left_msg.velocity = {0.0};
            gripper_left_msg.effort = {0.0};
            gripper_left_pub_->publish(gripper_left_msg);
        }
        else{//stop
            arm_twist_left_msg=geometry_msgs::msg::Twist();
        }
        
        
        if (target_arm == RIGHT || target_arm == BOTH) {
            arm_twist_right_msg=arm_twist_msg;
            arm_command_right_msg.reset_home_position = HOMEPOSITION;
            arm_command_right_msg.reset_end_posture = ENDEFFECTOR_RESET;
            arm_command_right_msg.end_effector_direction_mode = end_effector_direction_mode_right;
            arm_command_right_pub_->publish(arm_command_right_msg);

            if (msg->circle && !old_msg_.circle) {
                gripper_state_right_ = !gripper_state_right_;
            }
            gripper_right_msg.header.stamp = this->now();
            gripper_right_msg.name = {"openarm_right_finger_joint1", "openarm_right_finger_joint2"};
            gripper_right_msg.position = {gripper_state_right_ ? 0.044 : 0.0, gripper_state_right_ ? 0.044 : 0.0};
            gripper_right_msg.velocity = {0.0 , 0.0};
            gripper_right_msg.effort = {0.0, 0.0};
            gripper_right_pub_->publish(gripper_right_msg);
        }
        else{//stop
            arm_twist_right_msg=geometry_msgs::msg::Twist();
        }
        
        arm_twist_left_pub_->publish(arm_twist_left_msg);
        arm_twist_right_pub_->publish(arm_twist_right_msg);
    }

void flipperStringPublisher(const ps5_msgs::msg::PS5Joy::SharedPtr &msg) {
    std::string flipper_mode_str = (flipper_use_position_
        ? "[Position Control Mode]\n"
        : "[Velocity Control Mode]\n");
    
    std_msgs::msg::String flipper_mode_str_;
    flipper_mode_str_.data = flipper_mode_str;
    arm_mode_str_pub_->publish(flipper_mode_str_);

}

bool flipper_use_position_ = false;
void flipperCommandPublisher(const ps5_msgs::msg::PS5Joy::SharedPtr &msg) {

    auto flipper_msg = flipper_msgs::msg::Flipper();

  
    
    // スイッチが押されたときにモードを切り替え
    if (msg->command_r && !old_msg_.command_r) {
        flipper_use_position_ = !flipper_use_position_;
        RCLCPP_INFO(this->get_logger(), "Switched to %s mode", flipper_use_position_ ? "position" : "velocity");
    }

    flipper_msg.use_position = flipper_use_position_;
    flipper_msg.use_velocity = !flipper_use_position_;

    flipper_msg.auto_control =msg->joy_r;
    // positionモードの場合
    if (flipper_use_position_) {
        if (msg->circle) {
            flipper_msg.position = {0, 0, 0, 0};
        } else if (msg->square) {
            flipper_msg.position = {-M_PI / 2, -M_PI / 2, -M_PI / 2, -M_PI / 2};
        } else {
            // 位置の値をトランクして小数点以下2桁に制限
            for (size_t i = 0; i < flipper_msg.position.size(); ++i) {
                flipper_msg.position[i] = trunc(flipper_msg.position[i] * 100.0) / 100.0;
            }
        }
    } else { // velocityモードの場合
        // フリッパーの動作を制御
        flipper_msg.velocity = {0, 0, 0, 0};

    auto rotate_direction = [](bool up_key, bool down_key){
        if(up_key)
            return -1;
        else if (down_key)
            return 1;
        else 
            return 0;
    };

    double velocity_scale = 0.12; // 速度のスケールを調整するための係数 
    double velocity_scale_rear = 0.35; // 後ろフリッパーの速度スケールを調整するための係数
    if (!is_back_mode) {
            // 前左上げ下げ
            flipper_msg.velocity[0] = -velocity_scale*rotate_direction(msg->l1, msg->l2);
            // 後左上げ下げ
            flipper_msg.velocity[1] = -velocity_scale_rear*msg->cross_up;
            
            // 前右上げ下げ
            flipper_msg.velocity[2] =  -velocity_scale*rotate_direction(msg->r1, msg->r2);
            // 後右上げ下げ
            flipper_msg.velocity[3] = velocity_scale_rear*rotate_direction(msg->triangle, msg->cross);
            
    } else {
            // is_back_modeの場合のフリッパーの動作を制御
            flipper_msg.velocity[3] = -velocity_scale*rotate_direction(msg->l1, msg->l2);
            flipper_msg.velocity[1] = -velocity_scale*rotate_direction(msg->r1, msg->r2);
            flipper_msg.velocity[2] = velocity_scale*msg->cross_up;
            flipper_msg.velocity[0] = -velocity_scale*rotate_direction(msg->triangle, msg->cross);
    }

        // RStickの上下入力によるフリッパーの動作
        if (msg->right_joystick_y != 0) {
            double sign = (msg->right_joystick_y >= 0) ? velocity_scale*1 : velocity_scale*-1;
            flipper_msg.velocity = {sign, sign, sign, sign};
        }
        if(flipper_msg.auto_control){
            if (msg->right_joystick_y != 0) {
            double sign = (msg->right_joystick_y >= 0) ? velocity_scale*1 : velocity_scale*-1;
            flipper_msg.velocity = {sign, -sign,sign, -sign};
            }
        }
    }

    flipper_msg.name = {"joint_left_front", "joint_left_rear", "joint_right_front", "joint_right_rear"};
    flipper_msg.reset_home_position = msg->home;
    flipper_command_pub_->publish(flipper_msg);
}

    // Parameters
    double crawler_linear_x_max_velocity_ = 0.0;    
    double crawler_angular_z_max_velocity_ = 0.0;
    double manipulator_max_linear_velocity_ = 0.0;
    double manipulator_max_angular_velocity_ = 0.0;
    double flipper_max_velocity_ = 0.0;
    double flipper_min_velocity_ = 0.0;
    double body_max_linear_velocity_ = 0.0;
    double body_max_angular_velocity_ = 0.0;
    double head_max_linear_velocity_ = 0.0;
    double head_max_angular_velocity_ = 0.0;

    private:
    rclcpp::Subscription<ps5_msgs::msg::PS5Joy>::SharedPtr ps5_sub_;

    rclcpp::Publisher<arm_msgs::msg::ArmCommand>::SharedPtr arm_command_left_pub_, arm_command_right_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr arm_twist_left_pub_, arm_twist_right_pub_;
    rclcpp::Publisher<arm_msgs::msg::BodyCommand>::SharedPtr body_command_pub_;
    rclcpp::Publisher<flipper_msgs::msg::Flipper>::SharedPtr flipper_command_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr joy_mode_pub_,arm_mode_str_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twiststamped_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr gripper_left_pub_, gripper_right_pub_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_flag_;
    std_msgs::msg::String joy_mode_str_, arm_mode_str_;
   
    
    bool is_arm_mode_ = false;
    bool is_back_mode = false;

    bool gripper_state_left_ = false;
    bool gripper_state_right_ = false;
    

    ps5_msgs::msg::PS5Joy old_msg_;

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    rclcpp::spin(std::make_shared<Joy2Command>());
    rclcpp::shutdown();
    return 0;
}
