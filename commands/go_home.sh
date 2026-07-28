#!/bin/bash
# Command script to move Kinova Gen3 arm to HOME_POSITION

echo "=== Sende Befehl: Kinova Gen3 Arm fährt in HOME_POSITION... ==="
ros2 topic pub /arm/gesture std_msgs/msg/String "{data: 'home'}" --once
