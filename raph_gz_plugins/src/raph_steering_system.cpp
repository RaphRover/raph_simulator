// Copyright 2025 Fictionlab sp. z o.o.
// Plugin to support Raph Rover Ackermann steering and its different steering modes
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
#include <gz/msgs/double.pb.h>

#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <raph_interfaces/msg/steering_mode.hpp>
#include <raph_interfaces/srv/set_steering_mode.hpp>

#include <cmath>
#include <memory>
#include <chrono>

namespace raph_gz_plugins
{

class RaphSteeringSystem
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
private:
  gz::transport::Node gz_node_;
  gz::transport::Node::Publisher left_steering_joint_pub_;
  gz::transport::Node::Publisher right_steering_joint_pub_;
  gz::transport::Node::Publisher left_steering_wheel_pub_;
  gz::transport::Node::Publisher right_steering_wheel_pub_;
  gz::transport::Node::Publisher left_drive_wheel_pub_;
  gz::transport::Node::Publisher right_drive_wheel_pub_;


  std::shared_ptr<rclcpp::Node> ros_node_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr ros_sub_;
  rclcpp::Service<raph_interfaces::srv::SetSteeringMode>::SharedPtr steering_mode_service_;

  std::string robot_ns_;

  double wheelbase_;
  double max_steering_angle_;
  double turn_in_place_angle_;
  double max_wheel_speed_;
  double track_width_;
  double wheel_radius_;

  std::string left_steering_joint_;
  std::string right_steering_joint_;

  std::string left_steering_wheel_joint_;
  std::string right_steering_wheel_joint_;
  std::string left_drive_wheel_joint_;
  std::string right_drive_wheel_joint_;

  bool configured_{false};

  uint8_t current_steering_mode_{raph_interfaces::msg::SteeringMode::ACKERMANN};

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

    wheelbase_ = sdf->Get<double>("wheelbase", 0.378).first;
    max_steering_angle_ = sdf->Get<double>("max_steering_angle", 1.08).first;
    turn_in_place_angle_ = sdf->Get<double>("turn_in_place_angle", 1.10082).first;
    max_wheel_speed_ = sdf->Get<double>("max_wheel_speed", 1.52).first;
    track_width_ = sdf->Get<double>("track_width", 0.384).first;
    wheel_radius_ = sdf->Get<double>("wheel_radius", 0.08).first;

    left_steering_joint_ = sdf->Get<std::string>("left_steering_joint", "front_left_servo_joint").first;
    right_steering_joint_ = sdf->Get<std::string>("right_steering_joint", "front_right_servo_joint").first;
    left_steering_wheel_joint_ = sdf->Get<std::string>("left_steering_wheel_joint", "rear_left_wheel_joint").first;
    right_steering_wheel_joint_ = sdf->Get<std::string>("right_steering_wheel_joint", "rear_right_wheel_joint").first;
    left_drive_wheel_joint_ = sdf->Get<std::string>("left_drive_wheel_joint", "front_left_wheel_joint").first;
    right_drive_wheel_joint_ = sdf->Get<std::string>("right_drive_wheel_joint", "front_right_wheel_joint").first;

    this->ros_node_ = std::make_shared<rclcpp::Node>("raph_gz_steering_system", robot_ns_);
    std::string ros_ackermann_topic = "cmd_ackermann";
    this->ros_sub_ = this->ros_node_->create_subscription<ackermann_msgs::msg::AckermannDrive>(
      ros_ackermann_topic,
      rclcpp::QoS(1),
      [this](const ackermann_msgs::msg::AckermannDrive::SharedPtr msg) {
        if (current_steering_mode_ == raph_interfaces::msg::SteeringMode::ACKERMANN) {
          Drive(msg->speed, msg->steering_angle);
        } else if (current_steering_mode_ == raph_interfaces::msg::SteeringMode::TURN_IN_PLACE) {
          TurnInPlace(*msg);
        }
      }
    );

    std::string steering_mode_service_name = "~/set_steering_mode";
    this->steering_mode_service_ = this->ros_node_->create_service<raph_interfaces::srv::SetSteeringMode>(
      steering_mode_service_name,
      [this](const std::shared_ptr<raph_interfaces::srv::SetSteeringMode::Request> request,
              std::shared_ptr<raph_interfaces::srv::SetSteeringMode::Response> response) {
        this->SteeringModeCallback(request, response);
      }
    );

    std::string left_servo_topic = sdf->Get<std::string>("left_steering_joint_topic", "/model/robot/joint/front_left_servo_joint/cmd_pos").first;
    std::string right_servo_topic = sdf->Get<std::string>("right_steering_joint_topic", "/model/robot/joint/front_right_servo_joint/cmd_pos").first;
    std::string left_steering_wheel_topic = sdf->Get<std::string>("left_steering_wheel_topic", "/model/robot/joint/rear_left_wheel_joint/cmd_vel").first;
    std::string right_steering_wheel_topic = sdf->Get<std::string>("right_steering_wheel_topic", "/model/robot/joint/rear_right_wheel_joint/cmd_vel").first;
    std::string left_drive_wheel_topic = sdf->Get<std::string>("left_drive_wheel_topic", "/model/robot/joint/front_left_wheel_joint/cmd_vel").first;
    std::string right_drive_wheel_topic = sdf->Get<std::string>("right_drive_wheel_topic", "/model/robot/joint/front_right_wheel_joint/cmd_vel").first;

    left_steering_joint_pub_ = gz_node_.Advertise<gz::msgs::Double>(left_servo_topic);
    right_steering_joint_pub_ = gz_node_.Advertise<gz::msgs::Double>(right_servo_topic);
    left_steering_wheel_pub_ = gz_node_.Advertise<gz::msgs::Double>(left_steering_wheel_topic);
    right_steering_wheel_pub_ = gz_node_.Advertise<gz::msgs::Double>(right_steering_wheel_topic);
    left_drive_wheel_pub_ = gz_node_.Advertise<gz::msgs::Double>(left_drive_wheel_topic);
    right_drive_wheel_pub_ = gz_node_.Advertise<gz::msgs::Double>(right_drive_wheel_topic);

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
  }

private:
  void SteeringModeCallback(
    const std::shared_ptr<raph_interfaces::srv::SetSteeringMode::Request> request,
    std::shared_ptr<raph_interfaces::srv::SetSteeringMode::Response> response)
  {
    uint8_t requested_mode = request->steering_mode.data;
    
    if (requested_mode != raph_interfaces::msg::SteeringMode::ACKERMANN && 
        requested_mode != raph_interfaces::msg::SteeringMode::TURN_IN_PLACE) {
      response->success = false;
      response->status_message = "Invalid steering mode. Use 0 for Ackermann or 1 for Turn-in-place.";
      return;
    }

    if (current_steering_mode_ == requested_mode) {
      response->success = true;
      response->status_message = std::string("Steering mode already set to ") + 
        (requested_mode == raph_interfaces::msg::SteeringMode::ACKERMANN ? "Ackermann" : "Turn-in-place");
      return;
    }

    current_steering_mode_ = requested_mode;
    StopMovement();
    SetServoPositions(requested_mode);

    response->success = true;
    response->status_message = "Steering mode change initiated.";
  }

  void StopMovement()
  {
    gz::msgs::Double zero_velocity_msg;
    zero_velocity_msg.set_data(0.0);

    if (left_steering_wheel_pub_ && right_steering_wheel_pub_ && left_drive_wheel_pub_ && right_drive_wheel_pub_) {
      left_steering_wheel_pub_.Publish(zero_velocity_msg);
      right_steering_wheel_pub_.Publish(zero_velocity_msg);
      left_drive_wheel_pub_.Publish(zero_velocity_msg);
      right_drive_wheel_pub_.Publish(zero_velocity_msg);
    }
  }

  void SetServoPositions(uint8_t mode)
  {

    gz::msgs::Double left_position_msg;
    gz::msgs::Double right_position_msg;

    if (mode == raph_interfaces::msg::SteeringMode::ACKERMANN) {
      left_position_msg.set_data(0.0);
      right_position_msg.set_data(0.0);
    } else if (mode == raph_interfaces::msg::SteeringMode::TURN_IN_PLACE) {
      left_position_msg.set_data(-turn_in_place_angle_);
      right_position_msg.set_data(turn_in_place_angle_);
    }

    if (left_steering_joint_pub_ && right_steering_joint_pub_) {
      left_steering_joint_pub_.Publish(left_position_msg);
      right_steering_joint_pub_.Publish(right_position_msg);
    }
  }

  void TurnInPlace(const ackermann_msgs::msg::AckermannDrive & ackermann)
  {
    if (!left_steering_wheel_pub_ || !right_steering_wheel_pub_ || !left_drive_wheel_pub_ || !right_drive_wheel_pub_) {
      return;
    }

    const double angular_velocity = ackermann.speed;
    const double radius_f = track_width_ / 2.0;
    const double radius_r = std::sqrt(std::pow(track_width_ / 2.0, 2.0) + std::pow(wheelbase_, 2.0));

    const double rear_angular_velocity = radius_r / wheel_radius_ * angular_velocity;
    const double front_angular_velocity = rear_angular_velocity * radius_f / radius_r;

    gz::msgs::Double msg;

    msg.set_data(rear_angular_velocity);
    left_steering_wheel_pub_.Publish(msg);

    msg.set_data(-rear_angular_velocity);
    right_steering_wheel_pub_.Publish(msg);

    msg.set_data(front_angular_velocity);
    left_drive_wheel_pub_.Publish(msg);

    msg.set_data(-front_angular_velocity);
    right_drive_wheel_pub_.Publish(msg);

    msg.set_data(-turn_in_place_angle_);
    left_steering_joint_pub_.Publish(msg);

    msg.set_data(turn_in_place_angle_);
    right_steering_joint_pub_.Publish(msg);
  }

  void Drive(double speed, double steering_angle)
  {
    steering_angle = std::clamp(steering_angle, -max_steering_angle_, max_steering_angle_);

    double speed_rl = 0.0;
    double speed_rr = 0.0;
    double speed_fl = 0.0;
    double speed_fr = 0.0;
    double angle_l = 0.0;
    double angle_r = 0.0;

    if (steering_angle == 0.0) {
      speed_rl = speed_rr = speed_fl = speed_fr =
          std::clamp(speed, -max_wheel_speed_, max_wheel_speed_);
    } else {
      const double radius = wheelbase_ / std::tan(steering_angle);

      const double radius_fl = radius - track_width_ / 2.0;
      const double radius_fr = radius + track_width_ / 2.0;
      const double radius_rl =
          std::sqrt(std::pow(radius_fl, 2.0) + std::pow(wheelbase_, 2.0)) *
          (radius_fl < 0.0 ? -1.0 : 1.0);
      const double radius_rr =
          std::sqrt(std::pow(radius_fr, 2.0) + std::pow(wheelbase_, 2.0)) *
          (radius_fr < 0.0 ? -1.0 : 1.0);

      if (radius_fl == 0.0) {
        angle_l = M_PI / 2.0;
      } else {
        angle_l = std::atan(wheelbase_ / radius_fl);
      }

      if (radius_fr == 0.0) {
        angle_r = M_PI / 2.0;
      } else {
        angle_r = std::atan(wheelbase_ / radius_fr);
      }

      if (std::abs(radius_rl) >= std::abs(radius_rr)) {
        speed_rl = std::clamp(speed * (radius_rl / radius), -max_wheel_speed_, max_wheel_speed_);
        speed_rr = speed_rl * (radius_rr / radius_rl);
        speed_fl = speed_rl * (radius_fl / radius_rl);
        speed_fr = speed_rl * (radius_fr / radius_rl);
      } else {
        speed_rr = std::clamp(speed * (radius_rr / radius), -max_wheel_speed_, max_wheel_speed_);
        speed_rl = speed_rr * (radius_rl / radius_rr);
        speed_fr = speed_rr * (radius_fr / radius_rr);
        speed_fl = speed_rr * (radius_fl / radius_rr);
      }
    }

    gz::msgs::Double msg;

    msg.set_data(speed_rl / wheel_radius_);
    left_steering_wheel_pub_.Publish(msg);

    msg.set_data(speed_rr / wheel_radius_);
    right_steering_wheel_pub_.Publish(msg);

    msg.set_data(speed_fl / wheel_radius_);
    left_drive_wheel_pub_.Publish(msg);

    msg.set_data(speed_fr / wheel_radius_);
    right_drive_wheel_pub_.Publish(msg);

    msg.set_data(angle_l);
    left_steering_joint_pub_.Publish(msg);

    msg.set_data(angle_r);
    right_steering_joint_pub_.Publish(msg);
  }
};

}  // namespace raph_gz_plugins

GZ_ADD_PLUGIN(
  raph_gz_plugins::RaphSteeringSystem,
  gz::sim::System,
  raph_gz_plugins::RaphSteeringSystem::ISystemConfigure,
  raph_gz_plugins::RaphSteeringSystem::ISystemPreUpdate)