#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
VENV_PATH="$SCRIPT_DIR/lab_5"
PYTHON_SCRIPT="$SCRIPT_DIR/flight_controller.py"
LOG_FILE="$SCRIPT_DIR/logs/startup.log"

mkdir -p "$SCRIPT_DIR/logs"

echo "========================================" >> "$LOG_FILE"
echo "Starting at $(date)" >> "$LOG_FILE"
echo "========================================" >> "$LOG_FILE"

if [ ! -d "$VENV_PATH" ]; then
    echo "Virtual environment not found at $VENV_PATH" >> "$LOG_FILE"
    exit 1
fi

if [ ! -f "$PYTHON_SCRIPT" ]; then
    echo "Python script not found at $PYTHON_SCRIPT" >> "$LOG_FILE"
    exit 1
fi

source "$VENV_PATH/bin/activate"

export PYTHONUNBUFFERED=1

"$VENV_PATH/bin/python" "$PYTHON_SCRIPT" >> "$LOG_FILE" 2>&1 &

PID=$!
echo "Started with PID: $PID" >> "$LOG_FILE"
echo $PID > "$SCRIPT_DIR/flight_controller.pid"

exit 0
