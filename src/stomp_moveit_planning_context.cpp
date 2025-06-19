#include <atomic>
#include <future>

#include <stomp/stomp.h>

#include <stomp_moveit/stomp_moveit_planning_context.hpp>
#include <stomp_moveit/trajectory_visualization.hpp>
#include <stomp_moveit/filter_functions.hpp>
#include <stomp_moveit/noise_generators.hpp>
#include <stomp_moveit/cost_functions.hpp>
#include <stomp_moveit/stomp_moveit_task.hpp>

#include <stomp_moveit/stomp_moveit_parameters.hpp>

#include <moveit/constraint_samplers/constraint_sampler_manager.h>
#include <moveit/robot_state/conversions.h>

namespace stomp_moveit
{
bool solveWithStomp(const std::shared_ptr<stomp::Stomp>& stomp, const moveit::core::RobotState& start_state,
                    const moveit::core::RobotState& goal_state, const moveit::core::JointModelGroup* group,
                    const robot_trajectory::RobotTrajectoryPtr& input_trajectory,
                    robot_trajectory::RobotTrajectoryPtr& trajectory)
{
  Eigen::MatrixXd waypoints;
  const auto& joints = group->getActiveJointModels();
  bool success;
  if (!input_trajectory || input_trajectory->empty()) // input_trajectoryがnullか空でないとき
    success = stomp->solve(get_positions(start_state, joints), get_positions(goal_state, joints), waypoints); // スタートとゴールの線形補間
  else
  {
    auto input = robot_trajectory_to_matrix(*input_trajectory);
    success = stomp->solve(input, waypoints); // 与えられた軌道(input)を起点に探索する
  }
  if (success)
  {
    trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(start_state.getRobotModel(), group);
    fill_robot_trajectory(waypoints, start_state, *trajectory);
  }

  return success;
}

bool extractSeedTrajectory(const planning_interface::MotionPlanRequest& req,     // MotionPlanRequestから受け取ったプランニングの計画
                          const moveit::core::RobotModelConstPtr robot_model,    // ロボットの構造情報
                          robot_trajectory::RobotTrajectoryPtr& seed)            // 関数の中で作成された初期軌道を呼び出し元に出力するための引数
{
  if (req.trajectory_constraints.constraints.empty())
    return false; // 受け取ったMotionPlanRequestに軌道制約がない場合はfalseを返す

  // 関節グループ、関節名、自由度の数を取得
  const auto* joint_group = robot_model->getJointModelGroup(req.group_name);
  const auto& names = joint_group->getActiveJointModelNames();
  const auto dof = names.size();

  trajectory_msgs::msg::JointTrajectory seed_traj; // 初期軌道を格納するための変数を定義
  const auto& constraints = req.trajectory_constraints.constraints;  // alias to keep names short
  // Test the first point to ensure that it has all of the joints required
  for (size_t i = 0; i < constraints.size(); ++i)
  {
    auto n = constraints[i].joint_constraints.size();
    if (n != dof)
    {  // first test to ensure that dimensionality is correct
      RCLCPP_WARN(rclcpp::get_logger("stomp_moveit"),
                  "Seed trajectory index %lu does not have %lu constraints (has %lu instead).", i, dof, n);
      return false;
    }

    trajectory_msgs::msg::JointTrajectoryPoint joint_pt;

    for (size_t j = 0; j < constraints[i].joint_constraints.size(); ++j)
    {
      const auto& c = constraints[i].joint_constraints[j];
      if (c.joint_name != names[j])
      {
        RCLCPP_WARN(rclcpp::get_logger("stomp_moveit"),
                    "Seed trajectory (index %lu, joint %lu) joint name '%s' does not match expected name '%s'", i, j,
                    c.joint_name.c_str(), names[j].c_str());
        return false;
      }
      joint_pt.positions.push_back(c.position);
    }

    seed_traj.points.push_back(joint_pt);
  }
  seed_traj.joint_names = names;

  moveit::core::RobotState robot_state(robot_model);
  moveit::core::robotStateMsgToRobotState(req.start_state, robot_state);
  seed = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model, joint_group);
  seed->setRobotTrajectoryMsg(robot_state, seed_traj);

  return !seed->empty();
}

stomp::TaskPtr createStompTask(const stomp::StompConfiguration& config, StompPlanningContext& context)
{
  const size_t num_timesteps = config.num_timesteps;
  const auto planning_scene = context.getPlanningScene();
  const auto group = planning_scene->getRobotModel()->getJointModelGroup(context.getGroupName());

  // Check if we do have path constraints
  const auto& req = context.getMotionPlanRequest();
  kinematic_constraints::KinematicConstraintSet constraints(planning_scene->getRobotModel());
  constraints.add(req.path_constraints, planning_scene->getTransforms());

  // Create callback functions for STOMP task
  // Cost, noise and filter functions are provided for planning.
  // TODO(henningkayser): parameterize cost penalties
  using namespace stomp_moveit;
  CostFn cost_fn;
  if (!constraints.empty())
  {
    cost_fn = costs::sum({ costs::get_collision_cost_function(planning_scene, group, 1.0 /* collision penalty */),
                          costs::get_constraints_cost_function(planning_scene, group, constraints.getAllConstraints(),
                                                                1.0 /* constraint penalty */) });
  }
  else
  {
    cost_fn = costs::get_collision_cost_function(planning_scene, group, 1.0 /* collision penalty */);
  }

  // TODO(henningkayser): parameterize stddev
  const std::vector<double> stddev(group->getActiveJointModels().size(), 0.1);
  auto noise_generator_fn = noise::get_normal_distribution_generator(num_timesteps, stddev);
  auto filter_fn =
      filters::chain({ filters::simple_smoothing_matrix(num_timesteps), filters::enforce_position_bounds(group) });
  auto iteration_callback_fn =
      visualization::get_iteration_path_publisher(context.getPathPublisher(), planning_scene, group);
  auto done_callback_fn =
      visualization::get_success_trajectory_publisher(context.getPathPublisher(), planning_scene, group);

  // Initialize and return STOMP task
  stomp::TaskPtr task =
      std::make_shared<ComposableTask>(noise_generator_fn, cost_fn, filter_fn, iteration_callback_fn, done_callback_fn);
  return task;
}

stomp::StompConfiguration getStompConfig(const stomp_moveit::Params& params, size_t num_dimensions)
{
  stomp::StompConfiguration config;
  config.num_dimensions = num_dimensions;                                                 // Copied from joint count
  config.initialization_method = stomp::TrajectoryInitializations::LINEAR_INTERPOLATION;  // TODO: set from request
  config.num_iterations = params.num_iterations;
  config.num_iterations_after_valid = params.num_iterations_after_valid;
  config.num_timesteps = params.num_timesteps;
  config.delta_t = params.delta_t;
  config.exponentiated_cost_sensitivity = params.exponentiated_cost_sensitivity;
  config.num_rollouts = params.num_rollouts;
  config.max_rollouts = params.max_rollouts;
  config.control_cost_weight = params.control_cost_weight;
  RCLCPP_INFO(rclcpp::get_logger("stomp_moveit"), "ーーーーーーーーーーーーーーーーーーーーーー");
  RCLCPP_INFO(rclcpp::get_logger("stomp_moveit"), "パラメータ");
  RCLCPP_INFO(rclcpp::get_logger("stomp_moveit"), "STOMP Configuration Parameters:");
  RCLCPP_INFO(rclcpp::get_logger("stomp_moveit"), "Num Iterations: %ld", params.num_iterations);
  RCLCPP_INFO(rclcpp::get_logger("stomp_moveit"), "Num Iterations After Valid: %ld", params.num_iterations_after_valid);
  RCLCPP_INFO(rclcpp::get_logger("stomp_moveit"), "Num Timesteps: %ld", params.num_timesteps);
  RCLCPP_INFO(rclcpp::get_logger("stomp_moveit"), "Delta T: %f", params.delta_t);
  RCLCPP_INFO(rclcpp::get_logger("stomp_moveit"), "Exponentiated Cost Sensitivity: %f", params.exponentiated_cost_sensitivity);
  RCLCPP_INFO(rclcpp::get_logger("stomp_moveit"), "Num Rollouts: %ld", params.num_rollouts);
  RCLCPP_INFO(rclcpp::get_logger("stomp_moveit"), "Max Rollouts: %ld", params.max_rollouts);
  RCLCPP_INFO(rclcpp::get_logger("stomp_moveit"), "Control Cost Weight: %f", params.control_cost_weight);
  RCLCPP_INFO(rclcpp::get_logger("stomp_moveit"), "ーーーーーーーーーーーーーーーーーーーーーー");

  return config;
}

StompPlanningContext::StompPlanningContext(const std::string& name, const std::string& group,
                                          const stomp_moveit::Params& params)
  : planning_interface::PlanningContext(name, group), params_(params)
{
}

bool StompPlanningContext::solve(planning_interface::MotionPlanResponse& res)
{
  // Start time
  auto time_start = std::chrono::steady_clock::now();

  // カスタム軌道データの定義
  double trajectory_array[] = {
      0.0253615, 0.00990554, -0.111783, 0.0776433, -0.0137638, -0.0539208,
      0.0396319, 0.017227, -0.223884, 0.155411, 0.0388611, -0.0250857,
      0.0484995, 0.0252098, -0.340112, 0.235977, 0.0883874, 0.00642416,
      0.0554031, 0.0334833, -0.45877, 0.318118, 0.136096, 0.0386754,
      0.0627734, 0.0419326, -0.578633, 0.400865, 0.183029, 0.0706392,
      0.0723662, 0.050548, -0.698785, 0.483438, 0.230075, 0.101815,
      0.0853988, 0.0593422, -0.818512, 0.565222, 0.277977, 0.131995,
      0.102631, 0.0683212, -0.937229, 0.645735, 0.327326, 0.161114,
      0.12449, 0.0774762, -1.05446, 0.724607, 0.378575, 0.189185,
      0.151086, 0.086788, -1.16982, 0.801574, 0.432032, 0.216239,
      0.182247, 0.096219, -1.28293, 0.87646, 0.487864, 0.242318,
      0.217579, 0.1057, -1.39351, 0.949183, 0.546108, 0.26746,
      0.292423, 0.130939, -1.44187, 0.95035, 0.502895, 0.220266,
      0.370319, 0.15599, -1.48729, 0.949429, 0.461884, 0.172187,
      0.415713, 0.186603, -1.517, 0.910886, 0.368479, 0.104757,
      0.463058, 0.216735, -1.5437, 0.870546, 0.276875, 0.036368,
      0.51189, 0.246246, -1.56748, 0.828604, 0.186849, -0.0330795,
      0.561809, 0.275, -1.58851, 0.782218, 0.149087, -0.0483358,
      0.612459, 0.302867, -1.60699, 0.728217, 0.220051, 0.0521096,
      0.663531, 0.329733, -1.62318, 0.673355, 0.291924, 0.151049,
      0.714756, 0.355505, -1.63736, 0.617901, 0.364492, 0.248319,
      0.765899, 0.380119, -1.64984, 0.562127, 0.437552, 0.343762,
      0.816742, 0.403533, -1.66095, 0.506308, 0.510899, 0.437233,
      0.867073, 0.425726, -1.67109, 0.450721, 0.584325, 0.528611,
      0.91667, 0.446703, -1.68066, 0.39564, 0.657601, 0.617803,
      0.965311, 0.466488, -1.69008, 0.341336, 0.730482, 0.70476,
      1.01277, 0.48514, -1.6998, 0.288063, 0.802703, 0.789489,
      1.05882, 0.502764, -1.71028, 0.236054, 0.873976, 0.872076,
      1.10323, 0.519519, -1.72195, 0.185507, 0.943981, 0.952717,
      1.14576, 0.535635, -1.73525, 0.136572, 1.01238, 1.03177,
      1.18619, 0.55145, -1.75061, 0.0893161, 1.07885, 1.1099,
      1.22443, 0.567554, -1.76845, 0.0436238, 1.14327, 1.18841,
      1.26089, 0.585129, -1.78918, -0.00100569, 1.20602, 1.27003
  };
  // 配列の要素数を計算
  size_t trajectory_array_size = sizeof(trajectory_array) / sizeof(trajectory_array[0]);
  // 関節数を定義
  const size_t num_joints = 6;
  // タイムステップ数を計算
  size_t num_timesteps = trajectory_array_size / num_joints;
  
  // まず元の形式（タイムステップ×関節）でマッピングする
  Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> 
      temp_data(trajectory_array, num_timesteps, num_joints);
      
  // 行列を転置して関節×タイムステップの形式にする
  Eigen::MatrixXd trajectory_data = temp_data.transpose();

  // Response output
  auto& trajectory = res.trajectory_;
  auto& planning_time = res.planning_time_;
  auto& result_code = res.error_code_.val;
  result_code = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;  // Default to happy path

  // Extract start and goal states
  const auto& req = getMotionPlanRequest();
  const moveit::core::RobotState start_state(*getPlanningScene()->getCurrentStateUpdated(req.start_state));
  moveit::core::RobotState goal_state(start_state);
  constraint_samplers::ConstraintSamplerManager sampler_manager;
  auto goal_sampler = sampler_manager.selectSampler(getPlanningScene(), getGroupName(), req.goal_constraints.at(0));
  if (!goal_sampler || !goal_sampler->sample(goal_state))
  {
    result_code = moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
    return false;  // Can't plan without valid goal state
  }

  // STOMP config, task, planner instance
  const auto group = getPlanningScene()->getRobotModel()->getJointModelGroup(getGroupName());
  auto config = getStompConfig(params_, group->getActiveJointModels().size() /* num_dimensions */);
  robot_trajectory::RobotTrajectoryPtr input_trajectory; // input_trajectoryという軌道を格納する変数を定義
  
  // カスタム軌道使用フラグ（falseに設定するとカスタム軌道を使用しない = sとgの線形補間）
  bool use_custom_trajectory = true;
  RCLCPP_INFO(rclcpp::get_logger("stomp_moveit"), "カスタム軌道の使用設定: %s", 
              use_custom_trajectory ? "有効" : "無効");
  
  if (use_custom_trajectory && setCustomTrajectory(trajectory_data, input_trajectory))
  {
    RCLCPP_INFO(rclcpp::get_logger("stomp_moveit"), "Custom trajectoryが設定されました!!!!!!!!!!!!!");
    config.num_timesteps = input_trajectory->size();
  }
  else if (extractSeedTrajectory(request_, getPlanningScene()->getRobotModel(), input_trajectory))
    config.num_timesteps = input_trajectory->size();
  const auto task = createStompTask(config, *this);
  stomp_ = std::make_shared<stomp::Stomp>(config, task);

  std::condition_variable cv;
  std::mutex cv_mutex;
  bool finished = false;
  auto timeout_future = std::async(std::launch::async, [&, stomp = stomp_]() {
    std::unique_lock<std::mutex> lock(cv_mutex);
    cv.wait_for(lock, std::chrono::duration<double>(req.allowed_planning_time), [&finished] { return finished; });
    if (!finished)
    {
      stomp->cancel();
    }
  });

  // Solve
  if (!solveWithStomp(stomp_, start_state, goal_state, group, input_trajectory, trajectory))
  {
    // We timed out if the timeout task has completed so that the timeout future is valid and ready
    bool timed_out =
        timeout_future.valid() && timeout_future.wait_for(std::chrono::nanoseconds(1)) == std::future_status::ready;
    result_code =
        timed_out ? moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT : moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
  }
  stomp_.reset();
  {
    std::unique_lock<std::mutex> lock(cv_mutex);
    finished = true;
    cv.notify_all();
  }

  // Stop time
  std::chrono::duration<double> elapsed_seconds = std::chrono::steady_clock::now() - time_start;
  planning_time = elapsed_seconds.count();

  // プランニング結果のtrajectoryをコンソールに出力
  if (trajectory && !trajectory->empty()) {
    const auto& joint_names = group->getActiveJointModelNames();
    for (std::size_t i = 0; i < trajectory->getWayPointCount(); ++i) {
      std::cout << "Step " << i << ": ";
      for (std::size_t j = 0; j < joint_names.size(); ++j) {
        double value = trajectory->getWayPoint(i).getVariablePosition(joint_names[j]);
        std::cout << value;
        if (j + 1 < joint_names.size()) std::cout << " ";
      }
      std::cout << std::endl;
    }
  }

  return result_code == moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
}

bool StompPlanningContext::solve(planning_interface::MotionPlanDetailedResponse& /*res*/)
{
  return false;
}

bool StompPlanningContext::terminate()
{
  // Copy shared pointer to avoid race conditions
  auto stomp = stomp_;
  if (stomp)
  {
    return stomp->cancel();
  }

  return true;
}

void StompPlanningContext::clear()
{
}

void StompPlanningContext::setPathPublisher(
    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> path_publisher)
{
  path_publisher_ = path_publisher;
}

std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> StompPlanningContext::getPathPublisher()
{
  return path_publisher_;
}

bool StompPlanningContext::setCustomTrajectory(const Eigen::MatrixXd& trajectory_data, robot_trajectory::RobotTrajectoryPtr& input_trajectory)
{
  // ロボットモデルと関節グループを取得
  const auto robot_model = getPlanningScene()->getRobotModel();
  const auto group = robot_model->getJointModelGroup(getGroupName());
  const auto& joints = group->getActiveJointModels();

  // 軌道データのサイズチェック
  if (static_cast<size_t>(trajectory_data.rows()) != joints.size())
  {
    RCLCPP_ERROR(rclcpp::get_logger("stomp_moveit"), 
                "軌道データの次元数が一致しません。期待: %lu, 実際: %ld", 
                joints.size(), trajectory_data.rows());
    return false;
  }

  // 初期状態を取得
  moveit::core::RobotState robot_state(robot_model);
  moveit::core::robotStateMsgToRobotState(getMotionPlanRequest().start_state, robot_state);

  // 新しい軌道を作成
  input_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model, group);

  // 各タイムステップの関節角度を設定
  for (int timestep = 0; timestep < trajectory_data.cols(); ++timestep)
  {
    const auto waypoint = std::make_shared<moveit::core::RobotState>(robot_state);
    for (size_t joint_index = 0; joint_index < joints.size(); ++joint_index)
    {
      waypoint->setJointPositions(joints[joint_index], &trajectory_data(joint_index, timestep));
    }
    input_trajectory->addSuffixWayPoint(waypoint, 0.1 /* placeholder dt */);
  }

  return !input_trajectory->empty();
}
}
