#include <rclcpp/rclcpp.hpp>
#include <stomp/stomp.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <stomp_moveit/trajectory_visualization.hpp>
#include <stomp_moveit/filter_functions.hpp>
#include <stomp_moveit/noise_generators.hpp>
#include <stomp_moveit/cost_functions.hpp>
#include <stomp_moveit/stomp_moveit_task.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

using namespace std::chrono_literals;

stomp::StompConfiguration getStompConfiguration(size_t num_dimensions)
{
  stomp::StompConfiguration config;
  config.num_iterations = 1000;
  config.num_iterations_after_valid = 0;
  config.num_timesteps = 40;
  config.num_dimensions = num_dimensions;
  config.delta_t = 0.1;
  config.initialization_method = stomp::TrajectoryInitializations::LINEAR_INTERPOLATION;
  config.exponentiated_cost_sensitivity = 0.5;
  config.num_rollouts = 15;
  config.max_rollouts = 25;
  config.control_cost_weight = 0.1;
  return config;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("stomp_moveit_example", "", node_options);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  auto moveit_cpp = std::make_shared<moveit_cpp::MoveItCpp>(node);
  moveit_cpp->getPlanningSceneMonitorNonConst()->waitForCurrentRobotState(node->now(), 1.0);
  moveit_cpp->getPlanningSceneMonitorNonConst()->updateFrameTransforms();
  moveit_cpp->getPlanningSceneMonitorNonConst()->providePlanningSceneService();
  moveit_visual_tools::MoveItVisualTools visual_tools(node, "link_base", "stomp_moveit",
                                                      moveit_cpp->getPlanningSceneMonitorNonConst());

  auto markers_publisher =
      node->create_publisher<visualization_msgs::msg::MarkerArray>("/stomp_moveit", rclcpp::SystemDefaultsQoS());
  rclcpp::sleep_for(2s);

  // xArm6設定
  const auto robot_model = moveit_cpp->getRobotModel();
  const auto group = robot_model->getJointModelGroup("xarm6");
  const auto joints = group->getActiveJointModels();

  // Fake障害物（任意，なくても可）
  // geometry_msgs::msg::Pose block_pose;
  // block_pose.position.z = 0.3;
  // block_pose.position.y = 0.2;
  // visual_tools.publishCollisionBlock(block_pose, "my_block", 0.15);

  const auto planning_scene =
      planning_scene_monitor::LockedPlanningSceneRO(moveit_cpp->getPlanningSceneMonitorNonConst())->diff();
  planning_scene->decoupleParent();
  planning_scene->getCurrentStateNonConst().update();

  // スタート・ゴール状態
  const auto start_state = planning_scene->getCurrentState();
  auto goal_state = start_state;

  // 全関節を+0.2[rad]ずつ動かす
  std::vector<double> goal_positions;
  for (const auto* joint : joints)
  {
    double pos = *goal_state.getJointPositions(joint);
    goal_positions.push_back(pos + 0.2);  // 適当に動かすだけ
  }
  goal_state.setJointGroupPositions(group, goal_positions);

  // EEFリンク（xarm_gripper_base_linkと仮定）
  const std::string eef_link = "xarm_gripper_base_link";
  const Eigen::Isometry3d eef_transform = planning_scene->getFrameTransform(eef_link);
  geometry_msgs::msg::QuaternionStamped eef_orientation;
  eef_orientation.header.frame_id = "link_base";
  eef_orientation.quaternion = tf2::toMsg(Eigen::Quaterniond(eef_transform.linear()));
  moveit_msgs::msg::Constraints constraints =
      kinematic_constraints::constructGoalConstraints(eef_link, eef_orientation, 0.1);

  // 制約のチェック（失敗しても気にしなくていい）
  if (!planning_scene->isStateConstrained(start_state, constraints, true))
    std::cout << "start state doesn't satisfy constraints" << std::endl;
  if (!planning_scene->isStateConstrained(goal_state, constraints, true))
    std::cout << "goal state doesn't satisfy constraints" << std::endl;

  // STOMP設定
  stomp::StompConfiguration config = getStompConfiguration(joints.size());

  using namespace stomp_moveit;
  std::vector<double> noise_stddev(joints.size(), 0.1);
  auto noise_generator_fn = noise::get_normal_distribution_generator(config.num_timesteps, noise_stddev);
  auto cost_fn = costs::sum({
    costs::get_collision_cost_function(planning_scene, group, 1.0),
    costs::get_constraints_cost_function(planning_scene, group, constraints, 1.0)
  });
  auto filter_fn = filters::chain({
    filters::simple_smoothing_matrix(config.num_timesteps),
    filters::enforce_position_bounds(group)
  });
  auto iteration_callback_fn = visualization::get_iteration_path_publisher(markers_publisher, planning_scene, group);
  auto done_callback_fn = visualization::get_success_trajectory_publisher(markers_publisher, planning_scene, group);
  stomp::TaskPtr task =
      std::make_shared<ComposableTask>(noise_generator_fn, cost_fn, filter_fn, iteration_callback_fn, done_callback_fn);

  while (rclcpp::ok())
  {
    stomp::Stomp stomp(config, task);

    Eigen::MatrixXd trajectory;
    if (stomp.solve(get_positions(start_state, joints), get_positions(goal_state, joints), trajectory))
    {
      std::cout << "STOMP succeeded" << std::endl;
    }
    else
    {
      std::cout << "A valid solution was not found" << std::endl;
      rclcpp::sleep_for(std::chrono::seconds(5));
    }

    visual_tools.deleteAllMarkers();
    visual_tools.trigger();
    rclcpp::sleep_for(std::chrono::seconds(1));
  }

  executor.cancel();

  return 0;
}
