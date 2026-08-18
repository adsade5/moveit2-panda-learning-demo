#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/color_rgba.hpp>

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

bool objectExists(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                  const std::string& object_id)
{
  const std::vector<std::string> object_names = planning_scene_interface.getKnownObjectNames();
  return std::find(object_names.begin(), object_names.end(), object_id) != object_names.end();
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  const auto node = std::make_shared<rclcpp::Node>(
      "panda_planning_scene_obstacle_demo",
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
  static const std::string box_id = "demo_box";

  // MoveGroupInterface 用于机器人目标、规划和执行
  moveit::planning_interface::MoveGroupInterface move_group(node, planning_group);

  // PlanningSceneInterface 用于向 MoveIt 的虚拟环境中添加或删除碰撞物体
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const std::string planning_frame = move_group.getPlanningFrame();
  const std::string end_effector_link = move_group.getEndEffectorLink();

  RCLCPP_INFO(logger, "Planning group: %s", move_group.getName().c_str());
  RCLCPP_INFO(logger, "Planning frame: %s", planning_frame.c_str());
  RCLCPP_INFO(logger, "End effector link: %s", end_effector_link.c_str());

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
  const double target_y_offset = current_pose.position.y >= 0.10 ? -0.22 : 0.22;
  target_pose.position.y += target_y_offset;

  printPose(logger, "Target Pose:", target_pose);

  moveit_msgs::msg::CollisionObject remove_object;
  remove_object.id = box_id;
  remove_object.header.frame_id = planning_frame;
  remove_object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  planning_scene_interface.applyCollisionObject(remove_object);

  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.id = box_id;
  collision_object.header.frame_id = planning_frame;

  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = { 0.14, 0.04, 0.08 };

  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.5 * (current_pose.position.x + target_pose.position.x);
  box_pose.position.y = 0.5 * (current_pose.position.y + target_pose.position.y);
  box_pose.position.z = 0.5 * (current_pose.position.z + target_pose.position.z) - 0.16;

  collision_object.primitives.push_back(box);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

  std_msgs::msg::ColorRGBA box_color;
  box_color.r = 0.9;
  box_color.g = 0.1;
  box_color.b = 0.1;
  box_color.a = 0.8;

  RCLCPP_INFO(logger, "Collision object id: %s", collision_object.id.c_str());
  RCLCPP_INFO(logger, "Collision object frame: %s", collision_object.header.frame_id.c_str());
  RCLCPP_INFO(logger, "Box dimensions x/y/z = %.3f / %.3f / %.3f m",
              box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X],
              box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y],
              box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z]);
  RCLCPP_INFO(logger, "Box pose position x/y/z = %.6f / %.6f / %.6f",
              box_pose.position.x, box_pose.position.y, box_pose.position.z);

  if (!planning_scene_interface.applyCollisionObject(collision_object, box_color))
  {
    RCLCPP_ERROR(logger, "Failed to add collision object '%s' to PlanningScene.",
                 box_id.c_str());
    shutdown();
    return 1;
  }

  RCLCPP_INFO(logger, "Collision object '%s' added to PlanningScene.", box_id.c_str());

  if (!objectExists(planning_scene_interface, box_id))
  {
    RCLCPP_ERROR(logger, "Collision object '%s' was not found in PlanningScene query.",
                 box_id.c_str());
    planning_scene_interface.applyCollisionObject(remove_object);
    shutdown();
    return 1;
  }

  RCLCPP_INFO(logger, "Collision object '%s' confirmed in PlanningScene.", box_id.c_str());

  move_group.setMaxVelocityScalingFactor(0.15);
  move_group.setMaxAccelerationScalingFactor(0.15);
  move_group.setPlanningTime(10.0);
  move_group.setNumPlanningAttempts(10);

  // Pose Goal 指定末端目标位姿，MoveIt 会结合 IK 和碰撞检测寻找可行轨迹
  if (!move_group.setPoseTarget(target_pose))
  {
    RCLCPP_ERROR(logger, "Failed to set pose target.");
    move_group.clearPoseTargets();
    planning_scene_interface.applyCollisionObject(remove_object);
    RCLCPP_INFO(logger, "Collision object removed.");
    shutdown();
    return 1;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;

  RCLCPP_INFO(logger, "Planning Pose Goal with obstacle...");
  const bool planning_success =
      static_cast<bool>(move_group.plan(plan));

  if (!planning_success)
  {
    RCLCPP_ERROR(logger, "Planning FAILED");
    move_group.clearPoseTargets();
    planning_scene_interface.applyCollisionObject(remove_object);
    RCLCPP_INFO(logger, "Collision object removed.");
    shutdown();
    return 1;
  }

  RCLCPP_INFO(logger, "Planning SUCCESS - collision-free trajectory found");
  RCLCPP_INFO(logger, "Executing collision-free trajectory...");

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
      RCLCPP_INFO(logger, "Delta y = %.6f m", final_pose.position.y - current_pose.position.y);
    }
  }
  else
  {
    RCLCPP_ERROR(logger, "Execution FAILED");
  }

  if (planning_scene_interface.applyCollisionObject(remove_object))
  {
    RCLCPP_INFO(logger, "Collision object removed.");
  }
  else
  {
    RCLCPP_ERROR(logger, "Failed to remove collision object.");
  }

  shutdown();
  return execution_success ? 0 : 1;
}
