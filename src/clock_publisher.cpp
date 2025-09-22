#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

class ClockPublisher : public rclcpp::Node
{
public:
    ClockPublisher() : Node("clock_publisher")
    {
        // Publisher for /clock topic
        publisher_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

        // Timer to publish at 100 Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&ClockPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        // Get current ROS time
        rclcpp::Time now = this->get_clock()->now();

        rosgraph_msgs::msg::Clock msg;
        msg.clock = now;

        publisher_->publish(msg);

        /* RCLCPP_INFO(this->get_logger(),
                    "Published clock: %d.%09d",
                    msg.clock.sec, msg.clock.nanosec); */
    }

    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ClockPublisher>());
    rclcpp::shutdown();
    return 0;
}
