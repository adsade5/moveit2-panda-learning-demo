#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>

void printPose(const rclcpp::Logger& logger, const std::string& title,
               const geometry_msgs::msg::Pose& pose)
{
  RCLCPP_INFO(logger, "%s", title.c_str());
  RCLCPP_INFO(logger, "  position x = %.6f", pose.position.x);
  RCLCPP_INFO(logger, "  position y = %.6f", pose.position.y);
  RCLCPP_INFO(logger, "  position z = %.6f", pose.position.z);
  RCLCPP_INFO(logger, "  orientation x = %.6f", pose.orientation.x);
  RCLCPP_INFO(logger, "  orientation y = %.6f", pose.orientation.y);
  RCLCPP_INFO(logger, "  orientation z = %.6f", pose.orientation.z);
  RCLCPP_INFO(logger, "  orientation w = %.6f", pose.orientation.w);
}

bool isFinitePose(const geometry_msgs::msg::Pose& pose)
{
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  const auto node = std::make_shared<rclcpp::Node>(
      "panda_pose_goal_demo",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  const auto shutdown = [&]() {
    executor.cancel();
    if (spinner.joinable())
    {
      spinner.join();
    }
    rclcpp::shutdown();
  };

  const rclcpp::Logger logger = node->get_logger();
  static const std::string planning_group = "panda_arm";

  // MoveGroupInterface 是我们与 move_group 通信的高级 C++ 接口
  moveit::planning_interface::MoveGroupInterface move_group(node, planning_group);

  RCLCPP_INFO(logger, "Planning group: %s", move_group.getName().c_str());
  RCLCPP_INFO(logger, "Planning frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(logger, "End effector link: %s", move_group.getEndEffectorLink().c_str());

  move_group.setStartStateToCurrentState();

  const geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose().pose;
  if (!isFinitePose(current_pose))
  {
    RCLCPP_ERROR(logger, "Failed to read a valid current end-effector pose.");
    shutdown();
    return 1;
  }

  printPose(logger, "Current Pose:", current_pose);

  geometry_msgs::msg::Pose target_pose = current_pose;
  target_pose.position.z += 0.05;

  printPose(logger, "Target Pose:", target_pose);

  move_group.setMaxVelocityScalingFactor(0.15);
  move_group.setMaxAccelerationScalingFactor(0.15);

  // Pose Goal 指定的是末端位姿，而不是直接指定 7 个关节角
  if (!move_group.setPoseTarget(target_pose))
  {
    RCLCPP_ERROR(logger, "Failed to set pose target.");
    move_group.clearPoseTargets();
    shutdown();
    return 1;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;

  RCLCPP_INFO(logger, "Planning Pose Goal...");
  // MoveIt 需要通过运动学/IK 找到满足该末端 Pose 的合法关节状态
  const bool planning_success =
      static_cast<bool>(move_group.plan(plan));

  if (!planning_success)
  {
    RCLCPP_ERROR(logger, "Planning FAILED");
    move_group.clearPoseTargets();
    shutdown();
    return 1;
  }

  RCLCPP_INFO(logger, "Planning SUCCESS");
  RCLCPP_INFO(logger, "Executing trajectory...");

  // execute() 将规划好的 trajectory 发送给 controller 执行
  const bool execution_success =
      static_cast<bool>(move_group.execute(plan));

  move_group.clearPoseTargets();

  if (execution_success)
  {
    RCLCPP_INFO(logger, "Execution SUCCESS");
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    const geometry_msgs::msg::Pose final_pose = move_group.getCurrentPose().pose;
    if (isFinitePose(final_pose))
    {
      printPose(logger, "Final Pose:", final_pose);
      RCLCPP_INFO(logger, "Delta z = %.6f m", final_pose.position.z - current_pose.position.z);
    }
  }
  else
  {
    RCLCPP_ERROR(logger, "Execution FAILED");
  }

  shutdown();
  return execution_success ? 0 : 1;
}
