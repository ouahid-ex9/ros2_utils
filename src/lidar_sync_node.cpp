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

    auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
              .best_effort();

    // Clock subscriber
    clock_sub_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock", qos, std::bind(&LidarSyncNode::clockCallback, this, _1));

    // Create subs and pubs for each lidar topic
    for (const auto &topic : lidar_topics) {
      auto sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic, 10, [this, topic](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          lidarCallback(msg, topic);
        });

      // topic should be /lidar1/synced/points
      const auto output_topic = topic + "/synced";

      auto pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        output_topic, 10);

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
    const rclcpp::Time new_time = msg->clock;
    rclcpp::Time threshold_time = new_time + rclcpp::Duration::from_seconds(offset_);

    // If the clock time (plus offset) has not advanced, do not publish
    if (has_last_threshold_time_ && threshold_time <= last_threshold_time_) {
      return;
    }

    current_time_ = new_time;
    last_threshold_time_ = threshold_time;
    has_last_threshold_time_ = true;

    RCLCPP_INFO(this->get_logger(), "current_time: %lf", current_time_.seconds());

    // For each lidar buffer, publish ONLY the latest message with timestamp <= current_time + offset
    // and discard all earlier messages (including the published one to avoid duplicates).
    for (auto &kv : buffers_) {
      auto &buffer = kv.second;
      auto &pub = pubs_[kv.first];

      sensor_msgs::msg::PointCloud2::SharedPtr last_eligible_msg = nullptr;

      while (!buffer.empty()) {
        auto &front = buffer.front();
        rclcpp::Time msg_time(front->header.stamp);

        if (msg_time <= threshold_time) {
          last_eligible_msg = front;
          buffer.pop_front();
        } else {
          break;
        }
      }

      if (last_eligible_msg) {
        RCLCPP_INFO(this->get_logger(), "publishing latest past message");
        pub->publish(*last_eligible_msg);
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
