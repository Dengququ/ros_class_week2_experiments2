#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STATE_DIR="$SCRIPT_DIR/.voice_qa"
mkdir -p "$STATE_DIR"

source /opt/ros/noetic/setup.bash
source /home/bcsh/upros_class_code/devel/setup.bash
source /home/bcsh/djf_ros_class_ws/devel/setup.bash
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://bcsh:11311}"

start_bg() {
  local name="$1"
  shift
  local log_file="$STATE_DIR/${name}.log"
  local pid_file="$STATE_DIR/${name}.pid"

  if [[ -f "$pid_file" ]]; then
    local old_pid
    old_pid="$(cat "$pid_file")"
    if kill -0 "$old_pid" 2>/dev/null; then
      echo "$name is already running with pid $old_pid"
      return
    fi
  fi

  nohup "$@" >"$log_file" 2>&1 &
  echo $! >"$pid_file"
  echo "Started $name (pid $(cat "$pid_file"))"
  echo "Log: $log_file"
}

if ! rostopic list >/dev/null 2>&1; then
  start_bg roscore roscore
  sleep 3
else
  echo "roscore is already running on $ROS_MASTER_URI"
fi

start_bg speech_to_word roslaunch upros_chat speech_to_word.launch
sleep 2
start_bg word_to_speech roslaunch upros_chat word_to_speech.launch
sleep 2
start_bg llm_chat rosrun my_class_pkg llm_chat.py
sleep 2

echo "Voice Q&A stack started."
echo "Check logs under $STATE_DIR"
echo "Stop with: $SCRIPT_DIR/stop_voice_qa.sh"
