#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include <deque>
#include <string>
#include <unordered_map>
#include <vector>

using std::placeholders::_1;

class LidarSyncNode : public rclcpp::Node
{
public:
  LidarSyncNode()
  : Node("lidar_sync_node")
  {
    // Offset parameter (seconds)
    offset_ = this->declare_parameter<double>("offset", 0.0);

    // List of lidar topics to synchronize
    std::vector<std::string> lidar_topics =
      this->declare_parameter<std::vector<std::string>>("lidar_topics",
        {"/lidar1/points", "/lidar2/points"});

    // Clock subscriber
    clock_sub_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock", 10, std::bind(&LidarSyncNode::clockCallback, this, _1));

    // Create subs and pubs for each lidar topic
    for (const auto &topic : lidar_topics) {
      auto sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic, 10, [this, topic](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          lidarCallback(msg, topic);
        });

      auto pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        topic + "_synced", 10);

      subs_[topic] = sub;
      pubs_[topic] = pub;
      buffers_[topic] = {};
    }

    RCLCPP_INFO(this->get_logger(), "LidarSyncNode started with %zu topics, offset=%.3f",
                lidar_topics.size(), offset_);
  }

private:
  void clockCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg)
  {
    current_time_ = msg->clock;

    // For each lidar buffer, release messages whose timestamp <= current_time + offset
    for (auto &kv : buffers_) {
      auto &buffer = kv.second;
      auto &pub = pubs_[kv.first];

      while (!buffer.empty()) {
        auto &front = buffer.front();
        rclcpp::Time msg_time(front->header.stamp);
        rclcpp::Time threshold_time = current_time_ + rclcpp::Duration::from_seconds(offset_);
        if (msg_time.nanoseconds() <= threshold_time.nanoseconds()) {
          pub->publish(*front);
          buffer.pop_front();
        } else {
          break;
        }
      }
    }
  }

  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg, const std::string &topic)
  {
    buffers_[topic].push_back(msg);
  }

  // Parameters
  double offset_;
  rclcpp::Time current_time_;

  // ROS interfaces
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
  std::unordered_map<std::string, rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subs_;
  std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> pubs_;

  // Buffers per lidar
  std::unordered_map<std::string, std::deque<sensor_msgs::msg::PointCloud2::SharedPtr>> buffers_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarSyncNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
