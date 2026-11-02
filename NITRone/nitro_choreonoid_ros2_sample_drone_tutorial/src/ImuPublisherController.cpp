#include <cnoid/SimpleController>
#include <cnoid/RateGyroSensor>
#include <cnoid/AccelerationSensor>
#include <cnoid/Link>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rosgraph_msgs/msg/clock.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include "MadgwickAHRS.h"  // ← 追加

using namespace cnoid;

class ImuPublisherController : public SimpleController
{
    RateGyroSensorPtr gyro;
    AccelerationSensorPtr accel;
    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub;
    SimpleControllerIO* io_ptr = nullptr;

    Madgwick filter;      // ← 姿勢推定フィルタ
    double prev_time = 0; // 前回の時刻（シミュレーション時間）


public:
    virtual bool configure(SimpleControllerConfig* config) override {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }

        // use_sim_time を true にする
        rclcpp::NodeOptions options;
        options.parameter_overrides({{"use_sim_time", rclcpp::ParameterValue(true)}});
        node = rclcpp::Node::make_shared("imu_publisher_node", options);

        imu_pub = node->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
        clock_pub = node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

        
        return true;
    }

    virtual bool initialize(SimpleControllerIO* io) override {
        io_ptr = io;  // control()で使うため保存
        gyro = io->body()->findDevice<RateGyroSensor>("GyroSensor");
        accel = io->body()->findDevice<AccelerationSensor>("AccSensor");

        if (!gyro || !accel) {
            io->os() << "GyroSensor or AccSensor not found." << std::endl;
            return false;
        }

        io->enableInput(gyro);
        io->enableInput(accel);

        prev_time = io->currentTime();
        
        return true;
    }

    virtual bool control() override {
        auto imu_msg = sensor_msgs::msg::Imu();

        // 現在のシミュレーション時間
        double now = io_ptr->currentTime();
        double dt = now - prev_time;
        prev_time = now;

        if (dt <= 0.0) dt = 0.001; // 安全対策（初回用）

        // ✅ ROS時間に基づいたタイムスタンプ
        imu_msg.header.stamp = node->get_clock()->now();


        /*
        // IMUの orientation は、Link::R() から取得
        Eigen::Matrix3d R = io_ptr->body()->rootLink()->R();
        Eigen::Quaterniond q(R);

        imu_msg.orientation.x = q.x();
        imu_msg.orientation.y = q.y();
        imu_msg.orientation.z = q.z();
        imu_msg.orientation.w = q.w();

        imu_msg.orientation_covariance[0] = 0.01;  // Roll
        imu_msg.orientation_covariance[4] = 0.01;  // Pitch
        imu_msg.orientation_covariance[8] = 0.01;  // Yaw
        */

        //追加
        // === センサデータ取得 ===
        auto g = gyro->w();   // 角速度（rad/s）
        auto a = accel->dv(); // 加速度（m/s^2）

        // === Madgwickフィルタで姿勢推定 ===
        filter.begin(dt); // 時間ステップを更新
        filter.updateIMU(g.x(), g.y(), g.z(), a.x(), a.y(), a.z());

        imu_msg.orientation.w = filter.getQ0();
        imu_msg.orientation.x = filter.getQ1();
        imu_msg.orientation.y = filter.getQ2();
        imu_msg.orientation.z = filter.getQ3();

        imu_msg.orientation_covariance[0] = 0.01;
        imu_msg.orientation_covariance[4] = 0.01;
        imu_msg.orientation_covariance[8] = 0.01;
        

        // ✅ 角速度（ジャイロ）
        imu_msg.angular_velocity.x = g.x();
        imu_msg.angular_velocity.y = g.y();
        imu_msg.angular_velocity.z = g.z();
        imu_msg.angular_velocity_covariance[0] = 0.01;
        imu_msg.angular_velocity_covariance[4] = 0.01;
        imu_msg.angular_velocity_covariance[8] = 0.01;

        // ✅ 線形加速度（加速度センサ）
        imu_msg.linear_acceleration.x = a.x();
        imu_msg.linear_acceleration.y = a.y();
        imu_msg.linear_acceleration.z = a.z();
        imu_msg.linear_acceleration_covariance[0] = 0.01;
        imu_msg.linear_acceleration_covariance[4] = 0.01;
        imu_msg.linear_acceleration_covariance[8] = 0.01;

        // publish と spin_some
        imu_pub->publish(imu_msg);

        // /clockパブリッシュ
        if(io_ptr){
            double sim_time = io_ptr->currentTime();
            uint64_t nanosec = static_cast<uint64_t>(sim_time * 1e9);
            rosgraph_msgs::msg::Clock clock_msg;
            clock_msg.clock = rclcpp::Time(nanosec);
            clock_pub->publish(clock_msg);
        }
        rclcpp::spin_some(node);

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(ImuPublisherController)