#!/usr/bin/env bash
set -Ee -o pipefail
set +u 2>/dev/null || true

WS="/home/jjj/code/Apex_Deploy"
HTTP_PORT="${HTTP_PORT:-8081}"
PLAYBACK_PKG="bag_playback_nodes_py"
PLAYBACK_SERVICE_ENTRY="$WS/install/bag_playback_nodes_py/lib/bag_playback_nodes_py/playback_service"
PLAYBACK_NODE_ENTRY="$WS/install/bag_playback_nodes_py/lib/bag_playback_nodes_py/rd_playback_node"

usage() {
  cat <<'EOF'
Usage:
  ./play_data_once.sh --data /path/to/bag_or_bag_parent

Examples:
  ./play_data_once.sh --data /media/jjj/DATA-S2/recorded_bags/my_bag-26-05-29-16-16-34
  ./play_data_once.sh --data /media/jjj/DATA-S2/recorded_bags/my_bag-26-05-29-16-16-34/data

Environment:
  HTTP_PORT=8081   Playback HTTP port, default 8081.
EOF
}

DATA_PATH=""
while [ "$#" -gt 0 ]; do
  case "$1" in
    --data)
      DATA_PATH="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage
      exit 2
      ;;
  esac
done

if [ -z "$DATA_PATH" ]; then
  echo "Missing required argument: --data" >&2
  usage
  exit 2
fi

if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi

cd "$WS"
if [ -f install/setup.bash ]; then
  source install/setup.bash
fi

DATA_PATH="${DATA_PATH%/}"
if [ -f "$DATA_PATH/metadata.yaml" ]; then
  BAG_PATH="$DATA_PATH"
elif [ -f "$DATA_PATH/data/metadata.yaml" ]; then
  BAG_PATH="$DATA_PATH/data"
else
  echo "Cannot find ROS2 bag metadata.yaml in:" >&2
  echo "  $DATA_PATH" >&2
  echo "  $DATA_PATH/data" >&2
  exit 1
fi

kill_old_playback_backend() {
  local port_pids playback_pids

  port_pids=""
  if command -v lsof >/dev/null 2>&1; then
    port_pids="$(lsof -ti ":$HTTP_PORT" 2>/dev/null || true)"
  fi

  playback_pids="$(pgrep -f "$PLAYBACK_SERVICE_ENTRY|$PLAYBACK_NODE_ENTRY" 2>/dev/null || true)"

  if [ -z "$port_pids" ] && [ -z "$playback_pids" ]; then
    return
  fi

  if [ -n "$port_pids" ]; then
    local pid cmd
    for pid in $port_pids; do
      cmd="$(ps -p "$pid" -o args= 2>/dev/null || true)"
      case "$cmd" in
        *"$PLAYBACK_SERVICE_ENTRY"*|*"$PLAYBACK_NODE_ENTRY"*)
          ;;
        *)
          echo "Port $HTTP_PORT is already in use by a non-playback process:" >&2
          echo "  PID $pid: $cmd" >&2
          echo "Not killing it automatically." >&2
          exit 1
          ;;
      esac
    done
  fi

  echo "Stopping old playback backend..."
  curl -fsS -X POST "http://127.0.0.1:$HTTP_PORT/api/playback_control" \
    -H "Content-Type: application/json" \
    -d '{"type":"stop"}' >/dev/null 2>&1 || true

  if [ -n "$playback_pids" ]; then
    kill $playback_pids >/dev/null 2>&1 || true
    sleep 1
    playback_pids="$(pgrep -f "$PLAYBACK_SERVICE_ENTRY|$PLAYBACK_NODE_ENTRY" 2>/dev/null || true)"
    if [ -n "$playback_pids" ]; then
      kill -9 $playback_pids >/dev/null 2>&1 || true
    fi
  fi

  if [ -n "$port_pids" ]; then
    sleep 0.5
    if command -v lsof >/dev/null 2>&1 && lsof -i ":$HTTP_PORT" >/dev/null 2>&1; then
      echo "Port $HTTP_PORT is still in use after stopping old playback backend." >&2
      lsof -i ":$HTTP_PORT" >&2 || true
      exit 1
    fi
  fi
}

kill_old_playback_backend

PLAYBACK_SERVICE_PID=""
PLAYBACK_NODE_PID=""
CLEANED_UP=0

cleanup() {
  if [ "$CLEANED_UP" = "1" ]; then
    return
  fi
  CLEANED_UP=1
  set +e
  echo
  echo "Stopping playback..."
  curl -fsS -X POST "http://127.0.0.1:$HTTP_PORT/api/playback_control" \
    -H "Content-Type: application/json" \
    -d '{"type":"stop"}' >/dev/null 2>&1

  if [ -n "$PLAYBACK_SERVICE_PID" ]; then
    kill "$PLAYBACK_SERVICE_PID" >/dev/null 2>&1
  fi
  if [ -n "$PLAYBACK_NODE_PID" ]; then
    kill "$PLAYBACK_NODE_PID" >/dev/null 2>&1
  fi
  wait "$PLAYBACK_SERVICE_PID" "$PLAYBACK_NODE_PID" >/dev/null 2>&1
}

handle_signal() {
  trap - EXIT INT TERM
  cleanup
  exit 130
}

trap cleanup EXIT
trap handle_signal INT TERM

echo "Starting playback backend..."
ros2 run "$PLAYBACK_PKG" playback_service --ros-args \
  -p "bag_directory:=$(dirname "$BAG_PATH")" \
  -p "http_port:=$HTTP_PORT" &
PLAYBACK_SERVICE_PID=$!

ros2 run "$PLAYBACK_PKG" rd_playback_node &
PLAYBACK_NODE_PID=$!

echo "Waiting for playback service on port $HTTP_PORT..."
for _ in $(seq 1 80); do
  if curl -fsS "http://127.0.0.1:$HTTP_PORT/api/playback_status" >/dev/null 2>&1; then
    break
  fi
  sleep 0.25
done

if ! curl -fsS "http://127.0.0.1:$HTTP_PORT/api/playback_status" >/dev/null 2>&1; then
  echo "Playback service did not become ready on port $HTTP_PORT." >&2
  exit 1
fi

echo "Waiting for playback node subscription..."
for _ in $(seq 1 80); do
  TOPIC_INFO="$(ros2 topic info /control/playback_control 2>/dev/null || true)"
  if echo "$TOPIC_INFO" | grep -q 'Subscription count: [1-9]'; then
    break
  fi
  sleep 0.25
done

TOPIC_INFO="$(ros2 topic info /control/playback_control 2>/dev/null || true)"
if ! echo "$TOPIC_INFO" | grep -q 'Subscription count: [1-9]'; then
  echo "Playback node did not subscribe to /control/playback_control." >&2
  echo "$TOPIC_INFO" >&2
  exit 1
fi

echo "Loading bag: $BAG_PATH"
curl -fsS -X POST "http://127.0.0.1:$HTTP_PORT/api/playback_control" \
  -H "Content-Type: application/json" \
  -d "{\"type\":\"load\",\"bag_path\":\"$BAG_PATH\"}" >/dev/null

sleep 0.5

echo "Playing once..."
curl -fsS -X POST "http://127.0.0.1:$HTTP_PORT/api/playback_control" \
  -H "Content-Type: application/json" \
  -d '{"type":"play"}' >/dev/null

STARTED=0
while true; do
  STATUS="$(curl -fsS "http://127.0.0.1:$HTTP_PORT/api/playback_status" 2>/dev/null || true)"
  if echo "$STATUS" | grep -q '"is_playing": true'; then
    STARTED=1
  fi
  if [ "$STARTED" = "1" ] && echo "$STATUS" | grep -q '"is_playing": false'; then
    echo "Playback completed."
    exit 0
  fi
  sleep 0.5
done
