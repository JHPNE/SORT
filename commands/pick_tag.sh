#!/usr/bin/env bash
# =====================================================================
# Sende ein Greif-Kommando (Pick) für ein AprilTag-Objekt
# Verwendung:
#   bash commands/pick_tag.sh 3     (Führt die Greif-Sequenz für Tag 3 aus)
# =====================================================================

TAG_ID="${1:-3}"

if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    source "$HOME/ros2_ws/install/setup.bash"
fi

CMD="pick_tag_${TAG_ID}"
echo "=== [Pick & Place] Sende Greif-Befehl für AprilTag $TAG_ID auf /arm/gesture... ==="
ros2 topic pub --once /arm/gesture std_msgs/msg/String "{data: '$CMD'}"
