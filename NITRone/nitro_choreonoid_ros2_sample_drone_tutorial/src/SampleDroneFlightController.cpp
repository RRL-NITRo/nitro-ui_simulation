/**
   Drone Flight Controller
   @author Kenta Suzuki
*/

#include <cnoid/EigenUtil>
#include <cnoid/RateGyroSensor>
#include <cnoid/Rotor>
#include <cnoid/SimpleController>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <memory>
#include <mutex>
#include <thread>

namespace {

const double pgain[] = { 1.000, 0.1, 0.1, 0.010 };
const double dgain[] = { 1.000, 0.1, 0.1, 0.001 };

} // namespace

class SampleDroneFlightController : public cnoid::SimpleController
{
public:
    virtual bool configure(cnoid::SimpleControllerConfig* config) override;
    virtual bool initialize(cnoid::SimpleControllerIO* io) override;
    virtual bool control() override;
    virtual void unconfigure() override;

private:
    cnoid::SimpleControllerIO* io;
    cnoid::BodyPtr ioBody;
    cnoid::DeviceList<cnoid::Rotor> rotors;
    cnoid::RateGyroSensor* gyroSensor;
    cnoid::Vector4 zref, zprev;
    cnoid::Vector4 dzref, dzprev;
    cnoid::Vector2 xref, xprev;
    cnoid::Vector2 dxref, dxprev;
    double timeStep;
    double time;
    double durationn;
    bool is_powered_on;

    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr publisher;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription;
    geometry_msgs::msg::Twist command;
    rclcpp::executors::StaticSingleThreadedExecutor::UniquePtr executor;
    std::thread executorThread;
    std::mutex commandMutex;
    std::mutex batteryMutex;

    cnoid::Vector4 getZRPY();
    cnoid::Vector2 getXY();
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SampleDroneFlightController)

bool SampleDroneFlightController::configure(cnoid::SimpleControllerConfig* config)
{
    node = std::make_shared<rclcpp::Node>(config->controllerName());

    publisher = node->create_publisher<sensor_msgs::msg::BatteryState>("/battery_status", 10);
    subscription = node->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 1, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(commandMutex);
            command = *msg;
        });

    executor = std::make_unique<rclcpp::executors::StaticSingleThreadedExecutor>();
    executor->add_node(node);
    executorThread = std::thread([this]() { executor->spin(); });

    return true;
}

bool SampleDroneFlightController::initialize(cnoid::SimpleControllerIO* io)
{
    this->io = io;
    ioBody = io->body();
    rotors = io->body()->devices();
    gyroSensor = ioBody->findDevice<cnoid::RateGyroSensor>("GyroSensor");
    is_powered_on = true;

    io->enableInput(ioBody->rootLink(), cnoid::Link::LinkPosition);
    io->enableInput(gyroSensor);

    for(auto& rotor : rotors) {
        io->enableInput(rotor);
    }

    zref = zprev = getZRPY();
    dzref = dzprev = cnoid::Vector4::Zero();
    xref = xprev = getXY();
    dxref = dxprev = cnoid::Vector2::Zero();

    timeStep = io->timeStep();
    time = durationn = 60.0 * 40.0;

    return true;
}

bool SampleDroneFlightController::control()
{
    // vel[z, r, p, y]
    static double vel[] = { 2.0, 2.0, 2.0, 1.047 };
    double val[] = { command.linear.z, command.linear.y, command.linear.x, command.angular.z };
    for(int i = 0; i < 4; ++i) {
        if(val[i] > 0.0) {
            val[i] = val[i] > vel[i] ? vel[i] : val[i];
        } else if(val[i] < 0.0) {
            val[i] = val[i] < -vel[i] ? -vel[i] : val[i];
        }
        val[i] = val[i] / vel[i] * -1.0;
    }

    double pos[4];
    for(int i = 0; i < 4; ++i) {
        pos[i] = val[i];
        if(fabs(pos[i]) < 0.2) {
            pos[i] = 0.0;
        }
    }

    cnoid::Vector4 fz = cnoid::Vector4::Zero();
    cnoid::Vector4 z = getZRPY();
    cnoid::Vector4 dz = (z - zprev) / timeStep;
    if(gyroSensor) {
        cnoid::Vector3 w = ioBody->rootLink()->R() * gyroSensor->w();
        dz[3] = w[2];
    }
    cnoid::Vector4 ddz = (dz - dzprev) / timeStep;

    cnoid::Vector2 x = getXY();
    cnoid::Vector2 dx = (x - xprev) / timeStep;
    cnoid::Vector2 ddx = (dx - dxprev) / timeStep;
    cnoid::Vector2 dx_local = Eigen::Rotation2Dd(-z[3]) * dx;
    cnoid::Vector2 ddx_local = Eigen::Rotation2Dd(-z[3]) * ddx;

    double cc = cos(z[1]) * cos(z[2]);
    double gfcoef = ioBody->mass() * 9.80665 / 4.0 / cc;

    if((fabs(cnoid::degree(z[1])) > 45.0) || (fabs(cnoid::degree(z[2])) > 45.0)) {
        is_powered_on = false;
    }

    if(!is_powered_on) {
        zref[0] = 0.0;
        dzref[0] = 0.0;
    }

    static const double P = 1.0;
    static const double D = 1.0;

    for(int i = 0; i < 4; ++i) {
        if(i == 3) {
            dzref[i] = -1.047 * pos[i];
            fz[i] = (dzref[i] - dz[i]) * pgain[i] + (0.0 - ddz[i]) * dgain[i];
        } else {
            if(i == 0) {
                zref[i] += -0.002 * pos[i];
            } else {
                int j = i - 1;
                dxref[j] = -2.0 * pos[i];
                zref[i] = P * (dxref[j] - dx_local[1 - j]) + D * (0.0 - ddx_local[1 - j]);
            }
            if(i == 1) {
                zref[i] *= -1.0;
            }
            fz[i] = (zref[i] - z[i]) * pgain[i] + (0.0 - dz[i]) * dgain[i];
        }
    }
    zprev = z;
    dzprev = dz;
    xprev = x;
    dxprev = dx;

    static const double TD[4][4] = {
        {1.0, -1.0, -1.0, -1.0},
        {1.0,  1.0, -1.0,  1.0},
        {1.0,  1.0,  1.0, -1.0},
        {1.0, -1.0,  1.0,  1.0}
    };
    static const double ATD[] = { -1.0, 1.0, -1.0, 1.0 };

    for(size_t i = 0; i < rotors.size(); ++i) {
        cnoid::Rotor* rotor = rotors[i];
        double ft =
            is_powered_on ? gfcoef + TD[i][0] * fz[0] + TD[i][1] * fz[1] + TD[i][2] * fz[2] + TD[i][3] * fz[3] : 0.0;
        rotor->force() = ft;
        rotor->torque() = ATD[i] * ft;
        rotor->notifyStateChange();
    }

    if(is_powered_on) {
        time -= timeStep;
    }
    double percentage = time / durationn * 100.0;

    {
        std::lock_guard<std::mutex> lock(batteryMutex);
        auto message = sensor_msgs::msg::BatteryState();
        message.voltage = 15.0;
        message.percentage = percentage > 0.0 ? percentage : 0.0;
        publisher->publish(message);
    }

    if(percentage <= 0.0) {
        is_powered_on = false;
    }

    return true;
}

void SampleDroneFlightController::unconfigure()
{
    if(executor) {
        executor->cancel();
        executorThread.join();
        executor->remove_node(node);
        executor.reset();
    }
}

cnoid::Vector4 SampleDroneFlightController::getZRPY()
{
    auto T = ioBody->rootLink()->position();
    double z = T.translation().z();
    cnoid::Vector3 rpy = cnoid::rpyFromRot(T.rotation());
    return cnoid::Vector4(z, rpy[0], rpy[1], rpy[2]);
}

cnoid::Vector2 SampleDroneFlightController::getXY()
{
    auto p = ioBody->rootLink()->translation();
    return cnoid::Vector2(p.x(), p.y());
}