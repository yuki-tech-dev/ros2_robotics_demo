#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>

class LaserPublisher : public rclcpp::Node
{
public:
  LaserPublisher()
  : Node("laser_publisher")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&LaserPublisher::publish_scan, this));
  }

private:
  void publish_scan()
  {
    auto msg = sensor_msgs::msg::LaserScan();

    // ヘッダ
    msg.header.stamp = this->now();
    msg.header.frame_id = "laser_link";

    // 角度設定
    msg.angle_min = -1.57;   // -90度
    msg.angle_max =  1.57;   // +90度
    msg.angle_increment = 0.01; // 0.01 rad ≈ 0.57度
    msg.time_increment = 0.0;
    msg.scan_time = 0.5;

    // 距離範囲
    msg.range_min = 0.0;
    msg.range_max = 10.0;

    // ダミーデータ生成（正弦波っぽい距離）
    int sample_count = static_cast<int>((msg.angle_max - msg.angle_min) / msg.angle_increment);
    msg.ranges.resize(sample_count);
    for (int i = 0; i < sample_count; i++) {
      msg.ranges[i] = 5.0 + std::sin(i * 0.1);  // 5m ±1m
    }

    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published LaserScan with %d points", sample_count);
  }

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserPublisher>());
  rclcpp::shutdown();
  return 0;
}