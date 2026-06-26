#!/usr/bin/env bash
# ════════════════════════════════════════════════════════════════════
#  run_px4_sim.sh
#  PX4 + Gazebo Harmonic Drone Simulation — Quick Launch Script
#
#  Usage:
#    bash run_px4_sim.sh          # Gazebo only  (no PX4 SITL)
#    bash run_px4_sim.sh --px4    # Gazebo + PX4 SITL (requires build)
#    bash run_px4_sim.sh --viewer # Auto-launch camera FOV viewer too
#
#  Full stack:
#    bash run_px4_sim.sh --px4 --viewer
# ════════════════════════════════════════════════════════════════════

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLD_FILE="$SCRIPT_DIR/px4_baylands_world.sdf"
VIEWER_FILE="$SCRIPT_DIR/px4_camera_fov_viewer.py"
PX4_DIR="$SCRIPT_DIR/PX4-Autopilot"

LAUNCH_PX4=false
LAUNCH_VIEWER=false

for arg in "$@"; do
  case $arg in
    --px4)    LAUNCH_PX4=true ;;
    --viewer) LAUNCH_VIEWER=true ;;
    -h|--help)
      echo "Usage: $0 [--px4] [--viewer]"
      echo "  --px4      Also start PX4 SITL (requires compiled PX4)"
      echo "  --viewer   Also start the camera FOV viewer window"
      exit 0
      ;;
  esac
done

# ── Banner ─────────────────────────────────────────────────────────
echo ""
echo "  ██████╗ ██╗  ██╗██╗  ██╗    ██████╗ ██████╗  ███████╗"
echo "  ██╔══██╗╚██╗██╔╝██║  ██║    ██╔══██╗██╔══██╗ ██╔════╝"
echo "  ██████╔╝ ╚███╔╝  ███████║    ██║  ██║██████╔╝ ███████╗"
echo "  ██╔═══╝  ██╔██╗  ╚════██║    ██║  ██║██╔══██╗ ╚════██║"
echo "  ██║     ██╔╝ ██╗       ██║    ██████╔╝██║  ██║ ███████║"
echo "  ╚═╝     ╚═╝  ╚═╝       ╚═╝    ╚═════╝ ╚═╝  ╚═╝ ╚══════╝"
echo ""
echo "  PX4 + Gazebo Harmonic Drone Simulation"
echo "  World  : $WORLD_FILE"
echo "  PX4    : $LAUNCH_PX4"
echo "  Viewer : $LAUNCH_VIEWER"
echo ""

# ── Check Gazebo ──────────────────────────────────────────────────
if ! command -v gz &>/dev/null; then
  echo "[ERROR] 'gz' not found. Install Gazebo Harmonic:"
  echo "        sudo apt install gz-harmonic"
  exit 1
fi

# ── Check world file ──────────────────────────────────────────────
if [[ ! -f "$WORLD_FILE" ]]; then
  echo "[ERROR] World file not found: $WORLD_FILE"
  exit 1
fi

# ── Set Gazebo resource paths ──────────────────────────────────────
export GZ_SIM_RESOURCE_PATH="$SCRIPT_DIR:$PX4_DIR/Tools/simulation/gz/models:${GZ_SIM_RESOURCE_PATH:-}"
echo "[INFO]  GZ_SIM_RESOURCE_PATH = $GZ_SIM_RESOURCE_PATH"

# ── Source ROS2 if available ───────────────────────────────────────
ROS_SETUP="/opt/ros/humble/setup.bash"
if [[ -f "$ROS_SETUP" ]]; then
  # shellcheck source=/dev/null
  source "$ROS_SETUP"
  echo "[INFO]  ROS2 Jazzy sourced."
else
  echo "[WARN]  ROS2 Jazzy not found at $ROS_SETUP — skipping."
fi

# ── Track child PIDs for cleanup ──────────────────────────────────
PIDS=()

cleanup() {
  echo ""
  echo "[Shutdown] Stopping all processes..."
  for pid in "${PIDS[@]}"; do
    kill "$pid" 2>/dev/null || true
  done
  wait 2>/dev/null
  echo "[Shutdown] Done."
}
trap cleanup EXIT INT TERM

# ── 1. Launch Gazebo ──────────────────────────────────────────────
echo ""
echo "════════════════════════════════════════════════════════════"
echo "  [1/3] Starting Gazebo Harmonic..."
echo "        $ gz sim $WORLD_FILE"
echo "════════════════════════════════════════════════════════════"

gz sim "$WORLD_FILE" &
GZ_PID=$!
PIDS+=("$GZ_PID")
echo "[INFO]  Gazebo PID = $GZ_PID"

# Give Gazebo time to initialise
echo "[INFO]  Waiting 4s for Gazebo to initialise..."
sleep 4

# Verify Gazebo is still running
if ! kill -0 "$GZ_PID" 2>/dev/null; then
  echo "[ERROR] Gazebo exited unexpectedly. Check SDF file."
  exit 1
fi
echo "[INFO]  Gazebo running ✓"

# ── Useful topic reminder ──────────────────────────────────────────
echo ""
echo "  Camera topic : /drone/camera/image"
echo "  IMU topic    : /imu"
echo "  Cmd vel      : /drone/cmd_vel"
echo ""
echo "  Quick checks:"
echo "    gz topic -l                            (list all topics)"
echo "    gz topic -e --json-output -t /drone/camera/image | head -5"
echo ""

# ── 2. Optional: PX4 SITL ─────────────────────────────────────────
if [[ "$LAUNCH_PX4" == "true" ]]; then
  echo "════════════════════════════════════════════════════════════"
  echo "  [2/3] Starting PX4 SITL..."
  echo "════════════════════════════════════════════════════════════"

  if [[ ! -d "$PX4_DIR" ]]; then
    echo "[WARN]  PX4-Autopilot not found at $PX4_DIR"
    echo "        Skipping PX4 SITL."
  else
    # PX4 SITL expects to be run from PX4-Autopilot dir
    echo "[INFO]  Building/launching PX4 SITL (gz_x500 airframe)..."
    echo "[INFO]  First build can take 15–30 min."
    (
      cd "$PX4_DIR"
      # Use GZ_SIM_RENDER_ENGINE=none to not spawn a second Gazebo window
      PX4_GZ_STANDALONE=1 make px4_sitl gz_x500 2>&1
    ) &
    PX4_PID=$!
    PIDS+=("$PX4_PID")
    echo "[INFO]  PX4 PID = $PX4_PID"
    sleep 2
  fi
fi

# ── 3. Optional: Camera FOV viewer ────────────────────────────────
if [[ "$LAUNCH_VIEWER" == "true" ]]; then
  echo "════════════════════════════════════════════════════════════"
  echo "  [3/3] Starting Camera FOV Viewer..."
  echo "════════════════════════════════════════════════════════════"

  if ! command -v python3 &>/dev/null; then
    echo "[WARN]  python3 not found, skipping viewer."
  elif [[ ! -f "$VIEWER_FILE" ]]; then
    echo "[WARN]  Viewer script not found: $VIEWER_FILE"
  else
    python3 "$VIEWER_FILE" &
    VW_PID=$!
    PIDS+=("$VW_PID")
    echo "[INFO]  Viewer PID = $VW_PID"
  fi
fi

echo ""
echo "════════════════════════════════════════════════════════════"
echo "  Simulation running. Press Ctrl+C to stop everything."
echo "════════════════════════════════════════════════════════════"
echo ""

# ── Hover — wait until user interrupts ───────────────────────────
wait "$GZ_PID"
