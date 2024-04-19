
#include "Fusion/Fusion.h"
#include "geometry_msgs/msg/pose_stamped.hpp"     //path
#include "geometry_msgs/msg/vector3_stamped.hpp"  //wheel_odom
#include "nav_msgs/msg/odometry.hpp"              //odometry
#include "nav_msgs/msg/path.hpp"                  //path
#include "odometry/ekf.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"                        //general
#include "sensor_msgs/msg/imu.hpp"                  //imu
#include "sensor_msgs/msg/magnetic_field.hpp"       //imu
#include "tf2/LinearMath/Quaternion.h"              //odometry, path and tf
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"  //tf
#include "tf2_ros/transform_broadcaster.h"          //tf

#define UNUSED(x) (void)(x)

using namespace std::chrono_literals;
constexpr double G = 9.7;

struct FusionAHRS
{
    FusionAHRS()
    {
        FusionOffsetInitialise(&offset, 250);
        FusionAhrsInitialise(&ahrs);
        const FusionAhrsSettings settings = {
            .convention     = FusionConventionNwu,
            .gain           = 0.5f,
            .gyroscopeRange = 2000.0f, /* replace this with actual gyroscope
                                          range in degrees/s */
            .accelerationRejection = 10.0f,
            .magneticRejection     = 10.0f,
            .recoveryTriggerPeriod = 5 * 250, /* 5 seconds */
        };
        FusionAhrsSetSettings(&ahrs, &settings);
    }

    auto update(Eigen::Vector3f accel,
                Eigen::Vector3f gyro,
                Eigen::Vector3f mag,
                const float &dt) -> void
    {
        accel                    = accel / G;             // convert to g
        gyro                     = gyro * 180.0f / M_PI;  // convert to deg/s
        FusionVector accelVector = {accel(0), accel(1), accel(2)};
        FusionVector gyroVector  = {gyro(0), gyro(1), gyro(2)};
        FusionVector magVector   = {mag(0), mag(1), mag(2)};
        accelVector              = FusionCalibrationInertial(accelVector,
                                                accelerometerMisalignment,
                                                accelerometerSensitivity,
                                                accelerometerOffset);
        gyroVector               = FusionCalibrationInertial(gyroVector,
                                               gyroscopeMisalignment,
                                               gyroscopeSensitivity,
                                               gyroscopeOffset);
        magVector                = FusionCalibrationMagnetic(
            magVector, softIronMatrix, hardIronOffset);

        gyroVector = FusionOffsetUpdate(&offset, gyroVector);
        FusionAhrsUpdate(&ahrs, gyroVector, accelVector, magVector, dt);
    }

    auto getBodyAccel() -> Eigen::Vector3f
    {
        FusionVector linearAccel = FusionAhrsGetLinearAcceleration(&ahrs);
        Eigen::Vector3f accel;
        accel << linearAccel.axis.x, linearAccel.axis.y, linearAccel.axis.z;
        return accel * G;  // convert to m/s^2
    }

    FusionOffset offset;
    FusionAhrs ahrs;
    const FusionMatrix gyroscopeMisalignment = {
        1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    const FusionVector gyroscopeSensitivity      = {1.0f, 1.0f, 1.0f};
    const FusionVector gyroscopeOffset           = {0.0f, 0.0f, 0.0f};
    const FusionMatrix accelerometerMisalignment = {
        1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
    const FusionVector accelerometerOffset      = {0.0f, 0.0f, 0.0f};
    const FusionMatrix softIronMatrix           = {
        1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    const FusionVector hardIronOffset = {0.0f, 0.0f, 0.0f};
};

// x: x, y, yaw, v, v_dot, w
EKF::ProcessModelFunc processModel = [](Eigen::VectorXd x,
                                        Eigen::VectorXd u,
                                        const double &dt) -> Eigen::VectorXd
{
    Eigen::VectorXd x_next = Eigen::VectorXd::Zero(x.size());
    UNUSED(u);

    x_next(0) = x(0) + x(3) * cos(x(2)) * dt + 0.5 * x(4) * cos(x(2)) * dt * dt;
    x_next(1) = x(1) + x(3) * sin(x(2)) * dt + 0.5 * x(4) * sin(x(2)) * dt * dt;
    x_next(2) = x(2) + x(5) * dt;
    x_next(3) = x(3) + x(4) * dt;
    x_next(4) = x(4);
    x_next(5) = x(5);
    return x_next;
};

EKF::VecMatFunc jacobianF = [](Eigen::VectorXd x,
                               const double &dt) -> Eigen::MatrixXd
{
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(x.size(), x.size());

    // clang-format off
    F << 1, 0, -x(3) * sin(x(2)) * dt, cos(x(2)) * dt, 0, 0,
         0, 1, x(3) * cos(x(2)) * dt, sin(x(2)) * dt, 0, 0,
         0, 0, 1, 0, 0, dt,
         0, 0, 0, 1, dt, 0,
         0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 1;
    // clang-format on

    return F;
};

// IMU measure model: z = [v_dot, w]
EKF::VecMatFunc jacobianImuH = [](Eigen::VectorXd x,
                                  const double &dt) -> Eigen::MatrixXd
{
    UNUSED(dt);
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, x.size());
    H(0, 4)           = 1.0;
    H(1, 5)           = 1.0;
    return H;
};

// Wheel encoder measure model: z = [v, w]
EKF::VecMatFunc jacobianWheelH = [](Eigen::VectorXd x,
                                    const double &dt) -> Eigen::MatrixXd
{
    UNUSED(dt);
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, x.size());
    H(0, 3)           = 1.0;
    H(1, 5)           = 1.0;
    return H;
};

const Eigen::MatrixXd Rimu   = Eigen::MatrixXd::Identity(2, 2) * 0.05;
const Eigen::MatrixXd Rwheel = Eigen::MatrixXd::Identity(2, 2) * 0.1;

class FilterOdom : public rclcpp::Node
{
   public:
    FilterOdom() : Node("filter_odom")
    {
        Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(6, 6) * 0.1;
        Eigen::MatrixXd P = Eigen::MatrixXd::Identity(6, 6) * 0.1;

        ekf = EKF(6, processModel, jacobianF, Q, P);

        rclcpp::SensorDataQoS sensorQos;
        // Subscriptions
        wheel_odom_sub_ =
            this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
                "/wheel_odom",
                10,
                std::bind(
                    &FilterOdom::wheel_callback, this, std::placeholders::_1));
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data_raw",
            sensorQos,
            std::bind(&FilterOdom::imu_callback, this, std::placeholders::_1));
        mag_sub_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
            "/imu/mag",
            sensorQos,
            std::bind(&FilterOdom::mag_callback, this, std::placeholders::_1));
        // Publishers
        odom_pub_ =
            this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        timer_ = this->create_wall_timer(
            100ms, std::bind(&FilterOdom::timer_callback, this));

        ekf.init(Eigen::VectorXd::Zero(6));
        last_time = now().seconds();
    }

   private:
    void timer_callback()
    {
        RCLCPP_INFO(this->get_logger(),
                    "state: %f %f %f %f %f %f",
                    ekf.getState()(0),
                    ekf.getState()(1),
                    ekf.getState()(2),
                    ekf.getState()(3),
                    ekf.getState()(4),
                    ekf.getState()(5));
        RCLCPP_INFO(this->get_logger(),
                    "body accel: %f %f %f",
                    ahrs.getBodyAccel()(0),
                    ahrs.getBodyAccel()(1),
                    ahrs.getBodyAccel()(2));
    }
    // Helper method that creates a odom message and publishes it.
    void publish_odom(const rclcpp::Time &stamp)
    {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp          = stamp;
        odom.header.frame_id       = "odom";
        odom.pose.pose.position.x  = ekf.getState()(0);
        odom.pose.pose.position.y  = ekf.getState()(1);
        odom.pose.pose.position.z  = 0.0;
        odom.twist.twist.linear.x  = ekf.getState()(3);
        odom.twist.twist.angular.z = ekf.getState()(5);
        tf2::Quaternion q =
            tf2::Quaternion(tf2::Vector3(0, 0, 1), ekf.getState()(2));
        odom.pose.pose.orientation = tf2::toMsg(q);

        odom_pub_->publish(odom);
    }

    // Helper method that appends to a path the pose and publishes the path.
    void publish_path(const rclcpp::Time &stamp)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp    = stamp;
        pose.header.frame_id = "odom";
        pose.pose.position.x = ekf.getState()(0);
        pose.pose.position.y = ekf.getState()(1);
        pose.pose.position.z = 0.0;
        tf2::Quaternion q =
            tf2::Quaternion(tf2::Vector3(0, 0, 1), ekf.getState()(2));
        pose.pose.orientation = tf2::toMsg(q);
        path_.header.stamp    = stamp;
        path_.header.frame_id = "odom";
        path_.poses.push_back(pose);
        path_pub_->publish(path_);
    }

    // Helper method that creates a tf: odom -> base_link, and publishes it.
    void broadcast_tf(const rclcpp::Time &stamp)
    {
        geometry_msgs::msg::TransformStamped odom_tf;
        odom_tf.header.stamp            = stamp;
        odom_tf.header.frame_id         = "odom";
        odom_tf.child_frame_id          = "base_link";
        odom_tf.transform.translation.x = ekf.getState()(0);
        odom_tf.transform.translation.y = ekf.getState()(1);
        odom_tf.transform.translation.z = 0.0;
        tf2::Quaternion q =
            tf2::Quaternion(tf2::Vector3(0, 0, 1), ekf.getState()(2));
        odom_tf.transform.rotation = tf2::toMsg(q);
        tf_broadcaster_->sendTransform(odom_tf);
    }

    // Callbacks from wheel enconders, realsense IMU and phidget IMU.
    void wheel_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mtx);
        const double now_time = now().seconds();
        const double dt       = now_time - last_time;
        last_time             = now_time;

        Eigen::VectorXd u = Eigen::VectorXd::Zero(2);
        ekf.predict(u, dt);

        Eigen::VectorXd z = Eigen::VectorXd::Zero(2);
        z(0)              = msg->vector.x;
        z(1)              = msg->vector.y;
        ekf.update(z, jacobianWheelH, Rwheel, dt);

        publish_odom(msg->header.stamp);
        publish_path(msg->header.stamp);
        broadcast_tf(msg->header.stamp);
    }
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        static int init = 0;
        std::lock_guard<std::mutex> lock(mtx);
        const double now_time = now().seconds();
        const double dt       = now_time - last_time;
        last_time             = now_time;

        Eigen::VectorXd gyro = Eigen::VectorXd::Zero(3);
        gyro << msg->angular_velocity.x, msg->angular_velocity.y,
            msg->angular_velocity.z;

        Eigen::VectorXd accel = Eigen::VectorXd::Zero(3);
        accel << msg->linear_acceleration.x, msg->linear_acceleration.y,
            msg->linear_acceleration.z;

        ahrs.update(
            accel.cast<float>(), gyro.cast<float>(), mag.cast<float>(), dt);

        if (init < 500)
        {
            init++;
            return;
        }

        // Eigen::VectorXd u = Eigen::VectorXd::Zero(2);
        // ekf.predict(u, dt);

        // Eigen::VectorXd z = Eigen::VectorXd::Zero(2);
        // // z(0)              = -msg->linear_acceleration.y;
        // z(0) = -ahrs.getBodyAccel()(1);
        // if (std::abs(msg->angular_velocity.z) < 0.01)
        //     z(1) = 0.0;
        // else
        //     z(1) = -msg->angular_velocity.z;
        // ekf.update(z, jacobianImuH, Rimu, dt);
    }

    void mag_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mtx);
        mag << msg->magnetic_field.x, msg->magnetic_field.y,
            msg->magnetic_field.z;
    }

    // Pubs and subs
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr
        wheel_odom_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    FusionAHRS ahrs;

    Eigen::Vector3f mag;

    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Path path_;

    double last_time = 0.0;

    EKF ekf;

    std::mutex mtx;
};

auto main(int argc, char **argv) -> int
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FilterOdom>());
    rclcpp::shutdown();
}
