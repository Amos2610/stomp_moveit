#pragma once

#include <moveit/planning_interface/planning_interface.h>
#include <stomp_moveit/stomp_moveit_parameters.hpp>
#include "path_reuse_method_interfaces/srv/get_path_seed_trajectory.hpp"


// Forward declaration
namespace stomp
{
class Stomp;
}

namespace stomp_moveit
{
class StompPlanningContext : public planning_interface::PlanningContext
{
public:
  ~StompPlanningContext() noexcept override; // デストラクタ
  StompPlanningContext(
    const std::string& name,
    const std::string& group_name,
    const stomp_moveit::Params& params,
    rclcpp::Node::SharedPtr node
  );
  bool solve(planning_interface::MotionPlanResponse& res) override;

  bool solve(planning_interface::MotionPlanDetailedResponse& res) override;

  bool terminate() override;

  void clear() override;

  void setPathPublisher(std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> path_publisher);
  std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> getPathPublisher();

  // パラメータにアクセスするためのメソッド
  const stomp_moveit::Params& getParams() const { return params_; }

  // カスタム軌道を設定する関数
  bool setCustomTrajectory(const Eigen::MatrixXd& trajectory_data, robot_trajectory::RobotTrajectoryPtr& input_trajectory);
  
  // PathSeedリクエストを処理する関数
  bool GetPathSeed();

  // 軌道をtxtファイルに保存する関数
  void saveTrajectoryToFile(const robot_trajectory::RobotTrajectory& trajectory, const std::vector<std::string>& joint_names, double planning_time);

  // カスタム軌道の状態を確認
  bool hasCustomTrajectory() const { return custom_trajectory_ != nullptr; }
  
  // カスタム軌道を取得
  const robot_trajectory::RobotTrajectoryPtr& getCustomTrajectory() const { return custom_trajectory_; }
  
  // カスタム軌道を設定
  void setCustomTrajectory(const robot_trajectory::RobotTrajectoryPtr& trajectory) { custom_trajectory_ = trajectory; }

  // PathSeedを取得するための定義
  rclcpp::Node::SharedPtr client_node_; // クライアントノードへのポインタ
  rclcpp::CallbackGroup::SharedPtr cbg_; // コールバックグループへのポインタ
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_; // Executorへのポインタ
  std::thread exec_thread_; // Executorを実行するためのスレッド
  rclcpp::Client<path_reuse_method_interfaces::srv::GetPathSeedTrajectory>::SharedPtr get_path_seed_client_; // PathSeedを取得するためのサービスクライアント

private:
  const stomp_moveit::Params params_;
  std::shared_ptr<stomp::Stomp> stomp_;
  std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> path_publisher_;

  rclcpp::Node::SharedPtr node_;  // ROS2ノードへのポインタ

  // PathSeedデータを格納するための変数
  std::vector<double> path_seed_data_;
  size_t path_seed_rows_ = 0;
  size_t path_seed_cols_ = 0;
  
  // カスタム軌道を保存するための変数
  robot_trajectory::RobotTrajectoryPtr custom_trajectory_;
};
}  // namespace stomp_moveit
