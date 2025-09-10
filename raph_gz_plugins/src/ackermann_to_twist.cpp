// Copyright 2023 Fictionlab sp. z o.o.
// Adapted for direct ROS 2 subscription.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/twist.pb.h>

#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive.hpp>

#include <cmath>
#include <mutex>
#include <memory>

namespace raph_gz_plugins
{

class AckermannToTwist
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
  gz::transport::Node gz_node_;
  gz::transport::Node::Publisher cmd_vel_pub_;
  std::shared_ptr<rclcpp::Node> ros_node_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr ros_sub_;

  std::string robot_ns_;
  double wheelbase_;
  double max_steering_angle_;

  ackermann_msgs::msg::AckermannDrive last_ackermann_cmd_;
  std::mutex ackermann_mutex_;

  bool configured_{false};
  bool new_cmd_received_{false};

public:
  void Configure(
    const gz::sim::Entity & /*entity*/,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & /*ecm*/,
    gz::sim::EventManager & /*eventMgr*/) override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    robot_ns_ = sdf->Get<std::string>("robot_namespace", "").first;
    if (!robot_ns_.empty() && robot_ns_.back() != '/') {
      robot_ns_ += "/";
    }

    wheelbase_ = sdf->Get<double>("wheelbase", 1.0).first;

    max_steering_angle_ = sdf->Get<double>("max_steering_angle", 0.7854).first;

    this->ros_node_ = std::make_shared<rclcpp::Node>("ackermann_to_twist_plugin");

    std::string ros_ackermann_topic = robot_ns_ + "cmd_ackermann";
    this->ros_sub_ = this->ros_node_->create_subscription<ackermann_msgs::msg::AckermannDrive>(
      ros_ackermann_topic,
      rclcpp::QoS(10),
      [this](const ackermann_msgs::msg::AckermannDrive::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(this->ackermann_mutex_);
        this->last_ackermann_cmd_ = *msg;
        this->new_cmd_received_ = true;
      }
    );

    std::string gz_cmd_vel_topic = robot_ns_ + "cmd_vel";
    cmd_vel_pub_ = gz_node_.Advertise<gz::msgs::Twist>(gz_cmd_vel_topic);
    if (!cmd_vel_pub_) {
      gzerr << "Failed to create Gazebo publisher for topic: " << gz_cmd_vel_topic << std::endl;
      return;
    }

    gzmsg << "AckermannToTwist configured successfully!" << std::endl;
    gzmsg << "Robot namespace: " << (robot_ns_.empty() ? "[default]" : robot_ns_) << std::endl;
    gzmsg << "Wheelbase: " << wheelbase_ << " meters" << std::endl;
    gzmsg << "Max steering angle: " << max_steering_angle_ << " radians" << std::endl;
    gzmsg << "Subscribing to ROS 2 topic: " << ros_ackermann_topic << std::endl;
    gzmsg << "Publishing to Gazebo topic: " << gz_cmd_vel_topic << std::endl;

    configured_ = true;
  }

  void PreUpdate(
    const gz::sim::UpdateInfo & /*info*/,
    gz::sim::EntityComponentManager & /*ecm*/) override
  {
    if (!configured_) {
      return;
    }
    rclcpp::spin_some(this->ros_node_);

    std::lock_guard<std::mutex> lock(ackermann_mutex_);
    
    if (new_cmd_received_) {
      gz::msgs::Twist twist_msg;
      ConvertAckermannToTwist(last_ackermann_cmd_, twist_msg);
      cmd_vel_pub_.Publish(twist_msg);
      new_cmd_received_ = false;
    }
  }

private:
  void ConvertAckermannToTwist(const ackermann_msgs::msg::AckermannDrive & ackermann, gz::msgs::Twist & twist)
  {
    twist.Clear();

    double speed = ackermann.speed;
    double steering_angle = ackermann.steering_angle;

    steering_angle = std::clamp(steering_angle, -max_steering_angle_, max_steering_angle_);

    twist.mutable_linear()->set_x(speed);
    twist.mutable_linear()->set_y(0.0);
    twist.mutable_linear()->set_z(0.0);

    double angular_velocity = 0.0;
    if (std::abs(wheelbase_) > 0) {
        angular_velocity = (speed * std::tan(steering_angle)) / wheelbase_;
    }

    twist.mutable_angular()->set_x(0.0);
    twist.mutable_angular()->set_y(0.0);
    twist.mutable_angular()->set_z(angular_velocity);
  }
};

}  // namespace raph_gz_plugins

GZ_ADD_PLUGIN(
  raph_gz_plugins::AckermannToTwist,
  gz::sim::System,
  raph_gz_plugins::AckermannToTwist::ISystemConfigure,
  raph_gz_plugins::AckermannToTwist::ISystemPreUpdate)