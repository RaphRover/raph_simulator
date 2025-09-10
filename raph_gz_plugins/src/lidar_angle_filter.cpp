// Copyright 2025 Fictionlab sp. z o.o.
// LiDAR Angle Filter Plugin for Gazebo
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
#include <gz/msgs/laserscan.pb.h>

#include <cmath>
#include <mutex>
#include <memory>
#include <vector>
#include <string>
#include <limits>

namespace raph_gz_plugins
{

struct AngleRange
{
  double min_angle;
  double max_angle;
};

class LidarAngleFilter
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
  gz::transport::Node gz_node_;
  std::function<void(const gz::msgs::LaserScan&)> gz_sub_callback_;
  gz::transport::Node::Publisher filtered_lidar_pub_;

  std::string input_topic_;
  std::string output_topic_;
  std::vector<AngleRange> filter_ranges_;

  gz::msgs::LaserScan last_scan_;
  std::mutex scan_mutex_;

  bool configured_{false};
  bool new_scan_received_{false};

public:
  void Configure(
    const gz::sim::Entity & /*entity*/,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & /*ecm*/,
    gz::sim::EventManager & /*eventMgr*/) override
  {
    input_topic_ = sdf->Get<std::string>("input_topic", "scan_raw").first;
    output_topic_ = sdf->Get<std::string>("output_topic", "scan").first;

    ParseFilterRanges(sdf);

    filtered_lidar_pub_ = gz_node_.Advertise<gz::msgs::LaserScan>(output_topic_);
    if (!filtered_lidar_pub_) {
      gzerr << "Failed to create Gazebo publisher for topic: " << output_topic_ << std::endl;
      return;
    }

    gz_sub_callback_ = [this](const gz::msgs::LaserScan& msg) {
      std::lock_guard<std::mutex> lock(this->scan_mutex_);
      this->last_scan_ = msg;
      this->new_scan_received_ = true;
    };

    if (!gz_node_.Subscribe(input_topic_, gz_sub_callback_)) {
      gzerr << "Failed to subscribe to Gazebo topic: " << input_topic_ << std::endl;
      return;
    }

    gzmsg << "LidarAngleFilter configured successfully!" << std::endl;
    gzmsg << "Input topic: " << input_topic_ << std::endl;
    gzmsg << "Output topic: " << output_topic_ << std::endl;
    gzmsg << "Filter ranges configured: " << filter_ranges_.size() << std::endl;

    for (size_t i = 0; i < filter_ranges_.size(); ++i) {
      gzmsg << "  Range " << i << ": [" << filter_ranges_[i].min_angle 
            << ", " << filter_ranges_[i].max_angle << "] rad" << std::endl;
    }

    configured_ = true;
  }

  void PreUpdate(
    const gz::sim::UpdateInfo & /*info*/,
    gz::sim::EntityComponentManager & /*ecm*/) override
  {
    if (!configured_) {
      return;
    }

    std::lock_guard<std::mutex> lock(scan_mutex_);
    
    if (new_scan_received_) {
      gz::msgs::LaserScan filtered_scan = FilterLidarScan(last_scan_);
      filtered_lidar_pub_.Publish(filtered_scan);
      new_scan_received_ = false;
    }
  }

private:
  void ParseFilterRanges(const std::shared_ptr<const sdf::Element> & sdf)
  {
    filter_ranges_.clear();

    if (sdf->HasElement("filter_ranges")) {
      auto filter_ranges_elem = std::const_pointer_cast<sdf::Element>(sdf)->GetElement("filter_ranges");
      
      if (filter_ranges_elem) {
        auto range_elem = filter_ranges_elem->GetElement("range");
        while (range_elem) {
          double min_angle = range_elem->Get<double>("min", 0.0).first;
          double max_angle = range_elem->Get<double>("max", 0.0).first;
          
          if (min_angle < max_angle) {
            filter_ranges_.push_back({min_angle, max_angle});
            gzmsg << "Added filter range: [" << min_angle << ", " << max_angle << "] rad" << std::endl;
          } else {
            gzerr << "Invalid filter range: min_angle (" << min_angle 
                  << ") must be less than max_angle (" << max_angle << ")" << std::endl;
          }
          
          range_elem = range_elem->GetNextElement("range");
        }
      }
    } else {
      gzmsg << "No filter ranges specified, plugin will not filter any angles." << std::endl;
    }
  }

  gz::msgs::LaserScan FilterLidarScan(const gz::msgs::LaserScan& input_scan)
  {
    gz::msgs::LaserScan filtered_scan = input_scan;
    
    double angle_min = input_scan.angle_min();
    double angle_step = input_scan.angle_step();
    int range_count = input_scan.ranges_size();
    
    for (int i = 0; i < range_count; ++i) {
      double current_angle = angle_min + i * angle_step;
      
      bool should_filter = false;
      for (const auto& range : filter_ranges_) {
        if (current_angle >= range.min_angle && current_angle <= range.max_angle) {
          should_filter = true;
          break;
        }
      }
      
      if (should_filter) {
        filtered_scan.set_ranges(i, std::numeric_limits<double>::infinity());
        if (input_scan.intensities_size() > i) {
          filtered_scan.set_intensities(i, 0.0);
        }
      }
    }
    
    return filtered_scan;
  }
};

}  // namespace raph_gz_plugins

GZ_ADD_PLUGIN(
  raph_gz_plugins::LidarAngleFilter,
  gz::sim::System,
  raph_gz_plugins::LidarAngleFilter::ISystemConfigure,
  raph_gz_plugins::LidarAngleFilter::ISystemPreUpdate)
