#include "rclcpp/rclcpp.hpp"                          // ROS2の基本ノード機能
#include "trajectory_msgs/msg/joint_trajectory.hpp"   // JointTrajectoryメッセージ
#include "trajectory_msgs/msg/joint_trajectory_point.hpp" // 各ポイントの型

// ノードクラス定義
class SimpleArmTrajectory : public rclcpp::Node
{
public:
  SimpleArmTrajectory() : Node("simple_arm_trajectory")
  {
    // JointTrajectoryをPublishするPublisherを作成
    publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/simple_arm_controller/joint_trajectory", 10);

    // 2秒ごとにTrajectoryを送信するタイマーを作成
    timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&SimpleArmTrajectory::publish_trajectory, this));
  }

private:
  // TrajectoryをPublishする関数
  void publish_trajectory()
  {
    trajectory_msgs::msg::JointTrajectory traj;

    // 制御対象のjoint名を指定（URDFに合わせる）
    traj.joint_names = {"joint1","joint2","joint3","joint4","joint5","joint6"};

    // 1つの動作ポイントを作成
    trajectory_msgs::msg::JointTrajectoryPoint point;
    // 各jointの目標角度（ラジアン）
    point.positions = {0.5, -0.5, 0.5, -0.5, 0.5, -0.5};
    // この動作を完了するまでの時間
    point.time_from_start = rclcpp::Duration::from_seconds(2.0);

    // Trajectoryにポイントを追加
    traj.points.push_back(point);

    // Publish
    publisher_->publish(traj);
    RCLCPP_INFO(this->get_logger(), "Trajectory sent!");
  }

  // PublisherとTimerのメンバ変数
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// main関数
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);                           // ROS2初期化
  auto node = std::make_shared<SimpleArmTrajectory>(); // ノード生成
  rclcpp::spin(node);                                 // ノードを実行
  rclcpp::shutdown();                                 // 終了処理
  return 0;
}