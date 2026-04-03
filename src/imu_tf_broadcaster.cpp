/** @file imu_tf_broadcaster.cpp
 *
 * Relay IMU orientation to TF for 3D visualization.
 *
 * Subscribes to a sensor_msgs/Imu topic and publishes a dynamic TF transform
 * so tools like RViz can animate the sensor orientation in 3D space.
 *
 * Parameters:
 *   imu_topic      (string, default "/imu_sensor_broadcaster/imu")
 *   parent_frame   (string, default "world")
 *   child_frame    (string, default "base_link")
 *   translation_x  (double, default 0.0)
 *   translation_y  (double, default 0.0)
 *   translation_z  (double, default 0.0)
 */

#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/transform_broadcaster.h"

class ImuTfBroadcaster : public rclcpp::Node
{
public:
  explicit ImuTfBroadcaster(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("imu_tf_broadcaster", options)
  {
    declare_parameter("imu_topic",     "/imu_sensor_broadcaster/imu");
    declare_parameter("parent_frame",  "world");
    declare_parameter("child_frame",   "base_link");
    declare_parameter("translation_x", 0.0);
    declare_parameter("translation_y", 0.0);
    declare_parameter("translation_z", 0.0);

    imu_topic_    = get_parameter("imu_topic").as_string();
    parent_frame_ = get_parameter("parent_frame").as_string();
    child_frame_  = get_parameter("child_frame").as_string();
    tx_ = get_parameter("translation_x").as_double();
    ty_ = get_parameter("translation_y").as_double();
    tz_ = get_parameter("translation_z").as_double();

    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, 10,
      [this](const sensor_msgs::msg::Imu::ConstSharedPtr & msg) { callback(msg); });

    RCLCPP_INFO(
      get_logger(), "Relaying %s -> /tf (%s -> %s)",
      imu_topic_.c_str(), parent_frame_.c_str(), child_frame_.c_str());
  }

private:
  void callback(const sensor_msgs::msg::Imu::ConstSharedPtr & msg)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp    = msg->header.stamp;
    t.header.frame_id = parent_frame_;
    t.child_frame_id  = child_frame_;
    t.transform.translation.x = tx_;
    t.transform.translation.y = ty_;
    t.transform.translation.z = tz_;
    t.transform.rotation      = msg->orientation;
    broadcaster_->sendTransform(t);
  }

  std::string imu_topic_;
  std::string parent_frame_;
  std::string child_frame_;
  double tx_{}, ty_{}, tz_{};
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuTfBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
