#!/usr/bin/env bash
# =====================================================================
# Script to build ROS 2 packages and source the workspace setup.bash
# Includes --symlink-install by default for Python packages.
#
# Verwendung:
#   bash commands/build_and_source.sh                        (Baut ALLE Pakete per Symlink)
#   bash commands/build_and_source.sh vision_module          (Baut nur vision_module per Symlink)
#   bash commands/build_and_source.sh kinova_controller vision_module
#   bash commands/build_and_source.sh -select vision_module
#
# TODO: [WORKSPACE PFAD]
# Setze hier den Pfad zu deinem ROS 2 Workspace auf der VM (Standard: $HOME/ros2_ws)
# =====================================================================

WORKSPACE_DIR="${WORKSPACE_DIR:-$HOME/ros2_ws}"

# 1. Source ROS 2 system installation (Jazzy or Humble)
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
    echo "[Build] ROS 2 Jazzy erkannt & gesourced."
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "[Build] ROS 2 Humble erkannt & gesourced."
else
    echo "[WARNUNG] Keine ROS 2 System-Installation unter /opt/ros/ gefunden!"
fi

# 2. Build-Argumente parsen (--symlink-install + --packages-select)
COLCON_ARGS=("--symlink-install")
if [ $# -gt 0 ]; then
    first_arg="$1"
    if [[ "$first_arg" == "--packages-select" || "$first_arg" == "-packages-select" || "$first_arg" == "-select" || "$first_arg" == "-s" ]]; then
        shift
        COLCON_ARGS+=("--packages-select" "$@")
    else
        COLCON_ARGS+=("--packages-select" "$@")
    fi
fi

# 3. Build Workspace
if [ -d "$WORKSPACE_DIR" ]; then
    echo "=== Starte colcon build ${COLCON_ARGS[*]} im Workspace: $WORKSPACE_DIR ==="
    cd "$WORKSPACE_DIR" || exit 1
    colcon build "${COLCON_ARGS[@]}"
    BUILD_EXIT_CODE=$?

    if [ $BUILD_EXIT_CODE -eq 0 ]; then
        echo "=== Colcon Build ERFOLGREICH! Source install/setup.bash... ==="
        if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
            source "$WORKSPACE_DIR/install/setup.bash"
            echo "[OK] Workspace erfolgreich gesourced ($WORKSPACE_DIR/install/setup.bash)."
        else
            echo "[FEHLER] setup.bash unter $WORKSPACE_DIR/install/setup.bash nicht gefunden!"
        fi
    else
        echo "[FEHLER] Colcon Build fehlgeschlagen mit Exit Code $BUILD_EXIT_CODE!"
        exit $BUILD_EXIT_CODE
    fi
else
    echo "[FEHLER] Workspace-Verzeichnis '$WORKSPACE_DIR' existiert nicht!"
    echo "[TIPP] Passe den Pfad 'WORKSPACE_DIR' im Skript commands/build_and_source.sh an."
    exit 1
fi
