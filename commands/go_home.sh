#!/usr/bin/env bash
# =====================================================================
# Sende Befehl: Kinova Gen3 Arm fährt in HOME_POSITION
#
# VORAUSSETZUNG:
#   Der Kinova Controller (arm_mover) muss gestartet sein:
#   ros2 run kinova_controller arm_mover
# =====================================================================

if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    source "$HOME/ros2_ws/install/setup.bash"
fi

echo "=== Sende Befehl: Kinova Gen3 Arm fährt in HOME_POSITION... ==="
echo "[HINWEIS] Stelle sicher, dass der Controller läuft: ros2 run kinova_controller arm_mover"
ros2 topic pub --once /arm/gesture std_msgs/msg/String "{data: 'home'}"
