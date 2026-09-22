#!/usr/bin/env bash

set -u

pass_count=0
fail_count=0

pass_check() {
  echo "[PASS] $1"
  pass_count=$((pass_count + 1))
}

fail_check() {
  echo "[FAIL] $1"
  fail_count=$((fail_count + 1))
}

echo "=== ROS DISTRO ==="
echo "ROS_DISTRO=${ROS_DISTRO:-<unset>}"
if [[ "${ROS_DISTRO:-}" == "humble" ]]; then
  pass_check "ROS 2 Humble detected"
else
  fail_check "ROS_DISTRO is not humble"
fi

echo "=== CONTROLLERS ==="
controllers="$(timeout 8 ros2 control list_controllers 2>&1 | sed $'s/\033\\[[0-9;]*m//g')"
controller_status=$?
echo "${controllers}"
if [[ ${controller_status} -ne 0 ]]; then
  fail_check "ros2 control list_controllers returned ${controller_status}"
fi
if grep -Eq 'joint_state_broadcaster[[:space:]].*active' <<<"${controllers}"; then
  pass_check "joint_state_broadcaster active"
else
  fail_check "joint_state_broadcaster active"
fi
if grep -Eq 'joint_trajectory_controller[[:space:]].*active' <<<"${controllers}"; then
  pass_check "joint_trajectory_controller active"
else
  fail_check "joint_trajectory_controller active"
fi

echo "=== MOVEIT NODE ==="
nodes="$(ros2 node list 2>&1)"
echo "${nodes}"
if grep -Eq '^/move_group([[:space:]]|$)' <<<"${nodes}"; then
  pass_check "move_group found"
else
  fail_check "move_group found"
fi

echo "=== JOINT STATES ==="
topics="$(ros2 topic list 2>&1)"
echo "${topics}"
if grep -qx '/joint_states' <<<"${topics}"; then
  pass_check "/joint_states available"
else
  fail_check "/joint_states available"
fi
echo "--- /joint_states sample ---"
joint_sample="$(timeout 5 ros2 topic echo /joint_states --once 2>&1)"
joint_status=$?
echo "${joint_sample}"
if [[ ${joint_status} -eq 0 ]]; then
  pass_check "/joint_states publishes"
else
  fail_check "/joint_states publishes"
fi

echo "=== ACTIONS ==="
actions="$(ros2 action list 2>&1)"
echo "${actions}"
if grep -q '/joint_trajectory_controller/follow_joint_trajectory' <<<"${actions}"; then
  pass_check "joint trajectory action available"
else
  fail_check "joint trajectory action available"
fi

echo "=== DRAWING TOPIC ==="
if grep -qx '/drawing_path' <<<"${topics}"; then
  pass_check "/drawing_path available"
else
  fail_check "/drawing_path available"
fi
echo "--- /drawing_path info ---"
ros2 topic info /drawing_path 2>&1 || true

echo
echo "SUMMARY: ${pass_count} PASS, ${fail_count} FAIL"
if [[ ${fail_count} -eq 0 ]]; then
  exit 0
fi
exit 1
