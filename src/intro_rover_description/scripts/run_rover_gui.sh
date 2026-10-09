#!/usr/bin/env bash
set -eo pipefail

PROJECT_ROOT="/home/ahmet/projects/Software_Intro_Task_2026-2027"

set +u
source /opt/ros/jazzy/setup.bash
source "$PROJECT_ROOT/install/setup.bash"
set -u

export QT_QPA_PLATFORM=xcb
export GZ_SIM_RESOURCE_PATH="$PROJECT_ROOT/install/intro_rover_description/share:${GZ_SIM_RESOURCE_PATH:-}"

echo "Starting Gazebo GUI with the stable OpenGL backend..."
gz sim /usr/share/gz/gz-sim/worlds/empty.sdf \
  --render-engine-gui ogre2 \
  --render-engine-gui-api-backend opengl \
  -v 4 &
GAZ_PID=$!

for _ in $(seq 1 20); do
  if gz topic -l 2>/dev/null | grep -q '/world/empty'; then
    break
  fi
  sleep 1
done

echo "Spawning rover model..."
ros2 launch intro_rover_description spawn_gazebo.launch.py

wait "$GAZ_PID"
