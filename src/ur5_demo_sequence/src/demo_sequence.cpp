#include <rclcpp/rclcpp.hpp>  // ROS2 ノードの基本機能
#include <moveit/move_group_interface/move_group_interface.h>  // MoveIt2 の操作インターフェース

int main(int argc, char** argv)
{
  // ==============================
  // 1. ROS2 ノードの初期化
  // ==============================
  rclcpp::init(argc, argv);

  // NodeOptions を追加して外部パラメータを受け取れるようにする
  auto node = rclcpp::Node::make_shared(
      "ur5_demo_sequence",
      rclcpp::NodeOptions()
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true)
  );

  // ==============================
  // 2. MoveIt2 インターフェースの作成
  // ==============================
  // "ur_manipulator" は SRDF に定義されている UR5 のプランニンググループ名
  moveit::planning_interface::MoveGroupInterface move_group(node, "ur_manipulator");

  // 安全のため速度・加速度を制限（10%）
  move_group.setMaxVelocityScalingFactor(0.1);
  move_group.setMaxAccelerationScalingFactor(0.1);

  // ==============================
  // 3. 呼び出す Named Target のリスト
  // ==============================
  std::vector<std::string> targets = {"home", "ready", "place"};

  // ==============================
  // 4. 順番にターゲットを呼び出すループ
  // ==============================
  for (const auto& target : targets)
  {
    RCLCPP_INFO(node->get_logger(), "Moving to target: %s", target.c_str());
    move_group.setNamedTarget(target);

    auto result = move_group.move();

    if (result == moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(node->get_logger(), "Successfully moved to target: %s", target.c_str());
    }
    else
    {
      RCLCPP_WARN(node->get_logger(), "Failed to move to target: %s", target.c_str());
    }

    rclcpp::sleep_for(std::chrono::seconds(2));
  }

  // ==============================
  // 5. ノードの終了処理
  // ==============================
  rclcpp::shutdown();
  return 0;
}
