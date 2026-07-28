#!/usr/bin/env bash
# =====================================================================
# Sende ein Anfahr-Kommando per ROS 2 Topic /arm/gesture
# Verwendung:
#   bash commands/move_to_tag.sh 3     (Fährt zu AprilTag 3 aus WorldSpaceNode)
#   bash commands/move_to_tag.sh       (Fährt zum AprilTag der Arm-Kamera)
# =====================================================================

TAG_ID="${1:-}"

source /opt/ros/humble/setup.bash
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    source "$HOME/ros2_ws/install/setup.bash"
fi

if [ -n "$TAG_ID" ]; then
    CMD="goto_worldspace_tag_${TAG_ID}"
    echo "[Command] Sende WorldSpace IK-Anfahrbefehl für AprilTag $TAG_ID auf /arm/gesture..."
else
    CMD="goto_arm_camera_tag"
    echo "[Command] Sende Arm-Kamera IK-Anfahrbefehl auf /arm/gesture..."
fi

ros2 topic pub --once /arm/gesture std_msgs/msg/String "{data: '$CMD'}"
