#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

source /opt/ros/jazzy/setup.bash
source "${REPO_ROOT}/install/setup.bash"
cd "${REPO_ROOT}"
ros2 run panda_moveit_learning pose_goal_demo
