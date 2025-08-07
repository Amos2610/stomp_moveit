#pragma once

#include <moveit/planning_interface/planning_interface.h>

#include <stomp_moveit/stomp_moveit_parameters.hpp>

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

  // カスタム軌道を設定する関数
  bool setCustomTrajectory(const Eigen::MatrixXd& trajectory_data, robot_trajectory::RobotTrajectoryPtr& input_trajectory);

private:
  const stomp_moveit::Params params_;
  std::shared_ptr<stomp::Stomp> stomp_;
  std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> path_publisher_;

  rclcpp::Node::SharedPtr node_;  // ROS2ノードへのポインタ
};
}  // namespace stomp_moveit
