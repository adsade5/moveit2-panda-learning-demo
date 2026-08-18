#include <algorithm>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  const auto node = std::make_shared<rclcpp::Node>(
      "panda_joint_goal_demo",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  const rclcpp::Logger logger = node->get_logger();

  static const std::string planning_group = "panda_arm";

  // MoveGroupInterface 是我们与 move_group 通信的高级 C++ 接口
  moveit::planning_interface::MoveGroupInterface move_group(node, planning_group);

  RCLCPP_INFO(logger, "Planning group: %s", move_group.getName().c_str());
  RCLCPP_INFO(logger, "Planning frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(logger, "End effector link: %s", move_group.getEndEffectorLink().c_str());

  move_group.setStartStateToCurrentState();

  const std::vector<std::string> joint_names = move_group.getJointNames();
  const std::vector<double> current_joint_values = move_group.getCurrentJointValues();

  if (joint_names.empty() || current_joint_values.empty() ||
      joint_names.size() != current_joint_values.size())
  {
    RCLCPP_ERROR(logger,
                 "Failed to read a valid current joint state. joint_names=%zu, joint_values=%zu",
                 joint_names.size(), current_joint_values.size());
    executor.cancel();
    if (spinner.joinable())
    {
      spinner.join();
    }
    rclcpp::shutdown();
    return 1;
  }

  if (joint_names.size() != 7)
  {
    RCLCPP_ERROR(logger, "Expected 7 Panda arm joints, but got %zu.", joint_names.size());
    executor.cancel();
    if (spinner.joinable())
    {
      spinner.join();
    }
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(logger, "Current joint values:");
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    RCLCPP_INFO(logger, "  %s = %.6f", joint_names[i].c_str(), current_joint_values[i]);
  }

  const auto robot_model = move_group.getRobotModel();
  const auto* joint_model_group = robot_model ? robot_model->getJointModelGroup(planning_group) : nullptr;
  if (joint_model_group == nullptr)
  {
    RCLCPP_ERROR(logger, "Failed to get JointModelGroup: %s", planning_group.c_str());
    executor.cancel();
    if (spinner.joinable())
    {
      spinner.join();
    }
    rclcpp::shutdown();
    return 1;
  }

  const std::string first_joint_name = joint_names.front();
  const std::vector<std::string>& variable_names = joint_model_group->getVariableNames();
  const auto variable_it = std::find(variable_names.begin(), variable_names.end(), first_joint_name);
  if (variable_it == variable_names.end())
  {
    RCLCPP_ERROR(logger, "Joint %s is not part of planning group %s.",
                 first_joint_name.c_str(), planning_group.c_str());
    executor.cancel();
    if (spinner.joinable())
    {
      spinner.join();
    }
    rclcpp::shutdown();
    return 1;
  }

  const auto& bounds = robot_model->getVariableBounds(first_joint_name);
  const double current_joint1 = current_joint_values.front();

  auto make_target_if_valid =
      [&](double offset, std::vector<double>& target_joint_values) -> bool {
    const double candidate = current_joint1 + offset;
    if (bounds.position_bounded_ &&
        (candidate < bounds.min_position_ || candidate > bounds.max_position_))
    {
      return false;
    }

    target_joint_values = current_joint_values;
    target_joint_values.front() = candidate;

    // Joint Goal 直接指定目标关节角，因此这里不需要先通过末端 Pose 求 IK
    return move_group.setJointValueTarget(target_joint_values);
  };

  std::vector<double> target_joint_values;
  if (!make_target_if_valid(0.3, target_joint_values))
  {
    RCLCPP_WARN(logger, "%s + 0.3 rad is outside joint limits. Trying -0.3 rad.",
                first_joint_name.c_str());

    if (!make_target_if_valid(-0.3, target_joint_values))
    {
      RCLCPP_ERROR(logger, "Could not create a valid joint target for %s.", first_joint_name.c_str());
      executor.cancel();
      if (spinner.joinable())
      {
        spinner.join();
      }
      rclcpp::shutdown();
      return 1;
    }
  }

  RCLCPP_INFO(logger, "Target joint values:");
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    RCLCPP_INFO(logger, "  %s = %.6f", joint_names[i].c_str(), target_joint_values[i]);
  }

  move_group.setMaxVelocityScalingFactor(0.15);
  move_group.setMaxAccelerationScalingFactor(0.15);

  moveit::planning_interface::MoveGroupInterface::Plan plan;

  RCLCPP_INFO(logger, "Planning Joint Goal...");
  // plan() 只负责生成轨迹，并不会让机器人真正运动
  const bool planning_success =
      static_cast<bool>(move_group.plan(plan));

  if (!planning_success)
  {
    RCLCPP_ERROR(logger, "Planning FAILED");
    executor.cancel();
    if (spinner.joinable())
    {
      spinner.join();
    }
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(logger, "Planning SUCCESS");
  RCLCPP_INFO(logger, "Executing trajectory...");

  // execute() 将规划好的 trajectory 发送给 controller 执行
  const bool execution_success =
      static_cast<bool>(move_group.execute(plan));

  if (execution_success)
  {
    RCLCPP_INFO(logger, "Execution SUCCESS");
  }
  else
  {
    RCLCPP_ERROR(logger, "Execution FAILED");
  }

  executor.cancel();
  if (spinner.joinable())
  {
    spinner.join();
  }
  rclcpp::shutdown();

  return execution_success ? 0 : 1;
}
