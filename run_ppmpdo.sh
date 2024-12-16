#!/bin/bash

# Capture the target positions and wait time from command line arguments.
# If no target positions are passed, we'll use a default value.
# Similarly, if no wait time is passed, we'll use a default of 30 seconds.

# Default values
DEFAULT_WAIT_TIME=30
HOST="131.215.193.41"
PORT=7469

# Parse arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        --send-multiple-ppmpdo)
            TARGET_POSITIONS=("$2")  # Capture the list of target positions
            shift 2
            ;;
        --wait-time)
            WAIT_TIME="$2"  # Capture the wait time
            shift 2
            ;;
    esac
done

# If no wait time was passed, use the default
if [ -z "$WAIT_TIME" ]; then
    WAIT_TIME=$DEFAULT_WAIT_TIME
fi

# If no target positions are passed, ask the user for input
if [ -z "$TARGET_POSITIONS" ]; then
    read -p "Enter the target positions (separate by spaces): " -a TARGET_POSITIONS
fi

# Display the parameters for debugging purposes
echo "Target Positions: ${TARGET_POSITIONS[@]}"
echo "Wait Time: $WAIT_TIME"
echo "Host: $HOST"
echo "Port: $PORT"

# Run the Python script with the parsed arguments
python3 commands.py "$HOST" "$PORT" --send-multiple-ppmpdo "${TARGET_POSITIONS[@]}" --wait-time "$WAIT_TIME"

