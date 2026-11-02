#include <rclcpp/rclcpp.hpp> 
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <thread>

// We'll just set up parameters here
const std::string JOY_TOPIC = "/joy";
const std::string TWIST_TOPIC = "/servo_node/delta_twist_cmds";
const std::string JOINT_TOPIC = "/servo_node/delta_joint_cmds";
const std::string EEF_FRAME_ID = "link_hand_roll";
const std::string BASE_FRAME_ID = "link_base";

// Enums for button names -> axis/button array index
enum Axis
{
    LEFT_STICK_X = 0,
    LEFT_STICK_Y = 1,
    LEFT_TRIGGER = 2,
    RIGHT_STICK_X = 3,
    RIGHT_STICK_Y = 4,
    RIGHT_TRIGGER = 5,
    D_PAD_X = 6,
    D_PAD_Y = 7
};
enum Button
{
    A = 0,
    B = 1,
    X = 2,
    Y = 3,
    LEFT_BUMPER = 4,
    RIGHT_BUMPER = 5,
    CHANGE_VIEW = 6,
    MENU = 7,
    HOME = 8,
    LEFT_STICK_CLICK = 9,
    RIGHT_STICK_CLICK = 10
};

namespace moveit_servo
{
class JoyToServoPub : public rclcpp::Node
{
public:
    JoyToServoPub()  // コンストラクタ
        : Node("joy_to_twist_publisher"), frame_to_publish_(BASE_FRAME_ID)
    {
        // Setup pub/sub
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
                JOY_TOPIC, rclcpp::SystemDefaultsQoS(),
                [this](const sensor_msgs::msg::Joy::ConstSharedPtr& msg) { return joyCB(msg); });

        twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, rclcpp::SystemDefaultsQoS());
        joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, rclcpp::SystemDefaultsQoS());
        collision_pub_ =
                this->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", rclcpp::SystemDefaultsQoS());

        // Create a service client to start the ServoNode
        servo_start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
        servo_start_client_->wait_for_service(std::chrono::seconds(1));
        servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());

        // Load the collision scene asynchronously
        collision_pub_thread_ = std::thread([this]() {
            rclcpp::sleep_for(std::chrono::seconds(3));
            // Create collision object, in the way of servoing
            moveit_msgs::msg::CollisionObject collision_object;
            collision_object.header.frame_id = "link_base";
            collision_object.id = "box";

            shape_msgs::msg::SolidPrimitive table_1;
            table_1.type = table_1.BOX;
            table_1.dimensions = { 0.4, 0.6, 0.03 };

            geometry_msgs::msg::Pose table_1_pose;
            table_1_pose.position.x = 0.6;
            table_1_pose.position.y = 0.0;
            table_1_pose.position.z = 0.4;

            shape_msgs::msg::SolidPrimitive table_2;
            table_2.type = table_2.BOX;
            table_2.dimensions = { 0.6, 0.4, 0.03 };

            geometry_msgs::msg::Pose table_2_pose;
            table_2_pose.position.x = 0.0;
            table_2_pose.position.y = 0.5;
            table_2_pose.position.z = 0.25;

            collision_object.primitives.push_back(table_1);
            collision_object.primitive_poses.push_back(table_1_pose);
            collision_object.primitives.push_back(table_2);
            collision_object.primitive_poses.push_back(table_2_pose);
            collision_object.operation = collision_object.ADD;

            moveit_msgs::msg::PlanningSceneWorld psw;
            psw.collision_objects.push_back(collision_object);

            auto ps = std::make_unique<moveit_msgs::msg::PlanningScene>();
            ps->world = psw;
            ps->is_diff = true;
            collision_pub_->publish(std::move(ps));
        });
    }

    ~JoyToServoPub() override
    {
        if (collision_pub_thread_.joinable())
            collision_pub_thread_.join();
    }

    void joyCB(const sensor_msgs::msg::Joy::ConstSharedPtr& msg)
    {
        // Create the messages we might publish
        auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
        auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();

        // This call updates the frame for twist commands
        updateCmdFrame(frame_to_publish_, msg->buttons);

        // Convert the joystick message to Twist or JointJog and publish
        if (convertJoyToCmd(msg->axes, msg->buttons, twist_msg, joint_msg))
        {
            // publish the TwistStamped
            twist_msg->header.frame_id = frame_to_publish_;
            twist_msg->header.stamp = this->now();
            twist_pub_->publish(std::move(twist_msg));
        }
        else
        {
            // publish the JointJog
            joint_msg->header.stamp = this->now();
            joint_msg->header.frame_id = "link3_roll";
            joint_pub_->publish(std::move(joint_msg));
        }
    }

private:
    void updateCmdFrame(std::string& frame_to_publish, const std::vector<int>& buttons)
    {
        // ボタンの押下に基づいてフレームを更新
        if (buttons[Button::X] == 1)
        {
            frame_to_publish = "link_base";  // BASE_FRAME_IDに変更
        }
        else if (buttons[Button::Y] == 1)
        {
            frame_to_publish = "link_hand_roll";  // EEF_FRAME_IDに変更
        }
    }

   bool convertJoyToCmd(
    const std::vector<float>& axes, const std::vector<int>& buttons,
    std::unique_ptr<geometry_msgs::msg::TwistStamped>& twist_msg,
    std::unique_ptr<control_msgs::msg::JointJog>& joint_msg)
{
    // 軸入力をTwistに変換
    bool twist_input = false;

    // 左スティック: 平行移動（X軸とY軸）
    if (std::abs(axes[Axis::LEFT_STICK_X]) > 0.1 || std::abs(axes[Axis::LEFT_STICK_Y]) > 0.1)
    {
        twist_msg->twist.linear.x = axes[Axis::LEFT_STICK_X];  // 左スティックX軸
        twist_msg->twist.linear.y = axes[Axis::LEFT_STICK_Y];  // 左スティックY軸
        twist_input = true;
    }

    // 右スティック: Z軸の移動とZ軸の回転
    if (std::abs(axes[Axis::RIGHT_STICK_Y]) > 0.1)
    {
        twist_msg->twist.linear.z = axes[Axis::RIGHT_STICK_Y];  // Z軸の移動
        twist_input = true;
    }

    if (std::abs(axes[Axis::RIGHT_STICK_X]) > 0.1)
    {
        twist_msg->twist.angular.z = axes[Axis::RIGHT_STICK_X];  // Z軸の回転
        twist_input = true;
    }

    // トリガー: ピッチとロールの回転
    if (std::abs(axes[Axis::LEFT_TRIGGER]) > 0.1)
    {
        twist_msg->twist.angular.y = axes[Axis::LEFT_TRIGGER];  // ピッチ
        twist_input = true;
    }

    if (std::abs(axes[Axis::RIGHT_TRIGGER]) > 0.1)
    {
        twist_msg->twist.angular.x = axes[Axis::RIGHT_TRIGGER];  // ロール
        twist_input = true;
    }

    // Twistが入力されていれば、Twistをパブリッシュ
    if (twist_input)
    {
        return true;  // Twistをパブリッシュ
    }

    // Twist入力がない場合は、ジョイントコントロール
    joint_msg->joint_names.push_back("joint_name");  // 操作するジョイント名を指定
    joint_msg->velocities.push_back(axes[Axis::RIGHT_STICK_Y]);  // 右スティックY軸をジョイントに割り当て
    return false;  // JointJogをパブリッシュ
}

    // Members
    std::thread collision_pub_thread_;
    std::string frame_to_publish_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr collision_pub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;
};
}  // namespace moveit_servo

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<moveit_servo::JoyToServoPub>());
    rclcpp::shutdown();
    return 0;
}
