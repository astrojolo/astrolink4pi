#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROFILE_CONF="$SCRIPT_DIR/profile.conf"
PID_FILE="$SCRIPT_DIR/indiserver.pid"
LOG_FILE="$SCRIPT_DIR/indiserver.log"

ensure_profile_conf() {
    if [[ ! -f "$PROFILE_CONF" ]]; then
        cat <<EOF > "$PROFILE_CONF"
# List INDI drivers (one per line)
# Lines starting with # are ignored

indi_astrolink4pi
EOF
        echo "Created default profile.conf"
    fi
}

load_drivers() {
    ensure_profile_conf

    mapfile -t DRIVERS < <(grep -vE '^\s*#|^\s*$' "$PROFILE_CONF")

    if [[ ${#DRIVERS[@]} -eq 0 ]]; then
        echo "No drivers defined in $PROFILE_CONF"
        echo "Edit profile.conf to configure drivers."
        exit 1
    fi
}

is_running() {
    [[ -f "$PID_FILE" ]] && ps -p "$(cat "$PID_FILE")" > /dev/null 2>&1
}

start_server() {
    if is_running; then
        echo "INDI server already running (PID: $(cat "$PID_FILE"))"
        echo "Stop it using: $0 stop"
        exit 0
    fi

    load_drivers

    echo "Edit profile.conf to change driver configuration."
    echo "In Ekos, use Remote profile and connect to localhost:7624."
    echo "To stop the server: $0 stop"

    nohup indiserver -v "${DRIVERS[@]}" >"$LOG_FILE" 2>&1 &

    PID=$!
    echo "$PID" > "$PID_FILE"

    sleep 1

    if ps -p "$PID" > /dev/null 2>&1; then
        echo "INDI server started in background."
        echo "PID: $PID"
        echo "Log: $LOG_FILE"
    else
        echo "Failed to start INDI server. Check log: $LOG_FILE"
        rm -f "$PID_FILE"
        exit 1
    fi
}

start_log() {
    if is_running; then
        echo "INDI server already running (PID: $(cat "$PID_FILE"))"
        echo "Stop it using: $0 stop"
        exit 0
    fi

    load_drivers

    echo "Starting INDI server in debug mode (live logs)..."
    echo "Press Ctrl+C to stop."

    indiserver -v "${DRIVERS[@]}" 2>&1 | tee "$LOG_FILE"
}

stop_server() {
    if is_running; then
        PID="$(cat "$PID_FILE")"
        kill "$PID"
        rm -f "$PID_FILE"
        echo "INDI server stopped."
    else
        echo "INDI server is not running."
    fi
}

status_server() {
    if is_running; then
        echo "INDI server is running (PID: $(cat "$PID_FILE"))"
    else
        echo "INDI server is not running."
    fi
}

restart_server() {
    stop_server
    sleep 1
    start_server
}

case "${1:-}" in
    start)
        start_server
        ;;
    startlog)
        start_log
        ;;
    stop)
        stop_server
        ;;
    restart)
        restart_server
        ;;
    status)
        status_server
        ;;
    *)
        echo "Usage: $0 {start|startlog|stop|restart|status}"
        exit 1
        ;;
esac