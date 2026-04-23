#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STATE_DIR="$SCRIPT_DIR/.voice_qa"

stop_one() {
  local name="$1"
  local pid_file="$STATE_DIR/${name}.pid"
  if [[ ! -f "$pid_file" ]]; then
    echo "$name is not tracked"
    return
  fi

  local pid
  pid="$(cat "$pid_file")"
  if kill -0 "$pid" 2>/dev/null; then
    kill "$pid" || true
    sleep 1
    if kill -0 "$pid" 2>/dev/null; then
      kill -9 "$pid" || true
    fi
    echo "Stopped $name ($pid)"
  else
    echo "$name already stopped"
  fi
  rm -f "$pid_file"
}

stop_one llm_chat
stop_one word_to_speech
stop_one speech_to_word
stop_one roscore
