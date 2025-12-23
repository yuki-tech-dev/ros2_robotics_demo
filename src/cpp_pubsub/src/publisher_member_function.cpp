#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()  //コンストラクタ
  : Node("minimal_publisher"), count_(0)//- Node("minimal_publisher") → 基底クラス rclcpp::Node のコンストラクタを呼び出し、ノード名を "minimal_publisher" に設定。- count_ → メンバ変数 count_ を 0 で初期化。
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
//「rclcpp 名前空間の Publisher クラスを、送信データ型 std_msgs::msg::String でテンプレート指定し、
//その SharedPtr 型（スマートポインタのエイリアス）を、クラスのメンバ変数 publisher_ として定義している」
//「std_msgs::msg::String 型を publish できる Publisher クラス」を生成します
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}