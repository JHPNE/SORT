#!/usr/bin/env bash
#
# start_sort.sh - build the workspace and bring up the four SORT nodes.
#
#   ./start_sort.sh              build, then run everything
#   ./start_sort.sh -n           skip colcon build (fast restart)
#   ./start_sort.sh -w ~/ros2_ws
#
# Ctrl+C once shuts all four nodes down. If any node dies on its own, the
# rest are torn down too - a feedback node talking to a dead detector is
# worse than no feedback at all.

set -eo pipefail          # not -u: ROS setup.bash trips over unset vars

WS="${SORT_WS:-$HOME/ros2_ws}"
DO_BUILD=1

while [[ $# -gt 0 ]]; do
    case "$1" in
        -n|--no-build) DO_BUILD=0; shift ;;
        -w|--workspace) WS="$2"; shift 2 ;;
        -h|--help) sed -n '2,12p' "$0"; exit 0 ;;
        *) echo "unknown option: $1" >&2; exit 2 ;;
    esac
done

# package        executable      order matters: producers before consumers
NODES=(
    "vision_module      tag_detector"
    "vision_module      world_space"
    "control_module     gesture_node"
    "feedback_controller feedback_node"
)

PIDS=()
LOG_DIR=""

cleanup() {
    trap - INT TERM EXIT
    echo
    echo "--- shutting down ---"
    for pid in "${PIDS[@]}"; do
        # negative pid = the whole process group, so the python process
        # underneath 'ros2 run' goes too
        kill -TERM -"$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
    done
    sleep 2
    for pid in "${PIDS[@]}"; do
        kill -KILL -"$pid" 2>/dev/null || true
    done
    [[ -n "$LOG_DIR" ]] && echo "logs: $LOG_DIR"
}
trap cleanup INT TERM EXIT

cd "$WS" || { echo "no workspace at $WS (use -w)" >&2; exit 1; }

# ROS itself, if the shell has not sourced it already
set +u
if [[ -z "${ROS_DISTRO:-}" ]]; then
    for d in /opt/ros/*/setup.bash; do
        # shellcheck disable=SC1090
        [[ -f "$d" ]] && source "$d" && break
    done
fi
set -u
: "${ROS_DISTRO:?no ROS 2 found under /opt/ros}"
echo "ROS $ROS_DISTRO, workspace $WS"

if [[ $DO_BUILD -eq 1 ]]; then
    echo "--- colcon build ---"
    # --symlink-install: python sources are linked, not copied, so editing a
    # node only needs a restart, not a rebuild
    colcon build --symlink-install
fi

[[ -f install/setup.bash ]] || {
    echo "install/setup.bash missing - build first (drop -n)" >&2; exit 1; }

set +u
# shellcheck disable=SC1091
source install/setup.bash
set -u

LOG_DIR="$WS/log/sort_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"

echo "--- starting nodes ---"
for entry in "${NODES[@]}"; do
    read -r pkg exe <<< "$entry"
    # setsid: own process group, so cleanup can take the whole tree down
    # stdbuf -oL: line buffered, otherwise python output arrives in 4k lumps
    setsid stdbuf -oL ros2 run "$pkg" "$exe" \
        > >(stdbuf -oL sed "s/^/[$exe] /" | tee "$LOG_DIR/$exe.log") 2>&1 &
    PIDS+=($!)
    echo "  $pkg $exe (pid ${PIDS[-1]})"
    sleep 1        # let discovery settle before the next one comes up
done

ros2 topic pub --once /arm/gesture std_msgs/msg/String "data: 'home'"

echo "--- running, Ctrl+C to stop ---"
wait -n            # returns as soon as any one node exits
echo "a node exited - stopping the rest" >&2
