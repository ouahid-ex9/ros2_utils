#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

class TimeRewriteNode : public rclcpp::Node
{
public:
  TimeRewriteNode() : Node("time_rewrite_node")
  {
    using std::placeholders::_1;

    // Publishers
    pub_left_  = this->create_publisher<sensor_msgs::msg::PointCloud2>("/control/carla_ros_bridge/points/lidar_left_fixed", 10);
    pub_rear_  = this->create_publisher<sensor_msgs::msg::PointCloud2>("/control/carla_ros_bridge/points/lidar_rear_fixed", 10);
    pub_right_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/control/carla_ros_bridge/points/lidar_right_fixed", 10);
    pub_tf_    = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 100);

    // Subscriptions
    sub_left_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/control/carla_ros_bridge/points/lidar_left", 10,
      std::bind(&TimeRewriteNode::callbackLeft, this, _1));

    sub_rear_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/control/carla_ros_bridge/points/lidar_rear", 10,
      std::bind(&TimeRewriteNode::callbackRear, this, _1));

    sub_right_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/control/carla_ros_bridge/points/lidar_right", 10,
      std::bind(&TimeRewriteNode::callbackRight, this, _1));

    sub_tf_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_old", 100,
      std::bind(&TimeRewriteNode::callbackTF, this, _1));
  }

private:
  void callbackLeft(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    auto new_msg = *msg;
    new_msg.header.stamp = this->get_clock()->now();
    pub_left_->publish(new_msg);
  }

  void callbackRear(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    auto new_msg = *msg;
    new_msg.header.stamp = this->get_clock()->now();
    pub_rear_->publish(new_msg);
  }

  void callbackRight(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    auto new_msg = *msg;
    new_msg.header.stamp = this->get_clock()->now();
    pub_right_->publish(new_msg);
  }

  void callbackTF(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    auto new_msg = *msg;
    rclcpp::Time now = this->get_clock()->now();

    // Overwrite each transform’s timestamp
    for (auto & transform : new_msg.transforms) {
      transform.header.stamp = now;
    }
    pub_tf_->publish(new_msg);
  }

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_left_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_rear_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_right_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_tf_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_left_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_rear_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_right_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_tf_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TimeRewriteNode>());
  rclcpp::shutdown();
  return 0;
}
