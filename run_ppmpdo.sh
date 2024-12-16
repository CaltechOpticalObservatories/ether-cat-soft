#!/bin/bash

# Default values
DEFAULT_WAIT_TIME=30
DEFAULT_HOST="131.215.193.41"
DEFAULT_PORT=7469

# Initialize variables
HOST="$DEFAULT_HOST"
PORT="$DEFAULT_PORT"
TARGET_POSITIONS=()
WAIT_TIME=""
CSV_FILE=""

# Parse arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        --send-multiple-ppmpdo)
            # Capture all target positions passed after --send-multiple-ppmpdo
            shift  # Move past the --send-multiple-ppmpdo flag
            TARGET_POSITIONS=()  # Reset the target positions array
            # Capture all the target positions until another flag is encountered
            while [[ $# -gt 0 && ! "$1" =~ ^-- ]]; do
                TARGET_POSITIONS+=("$1")
                shift
            done
            ;;
        --wait-time)
            WAIT_TIME="$2"  # Capture the wait time
            shift 2
            ;;
        --host)
            HOST="$2"  # Override the default host if provided
            shift 2
            ;;
        --port)
            PORT="$2"  # Override the default port if provided
            shift 2
            ;;
        --csv)
            CSV_FILE="$2"  # Capture CSV file path
            shift 2
            ;;
        *)
            echo "Unknown argument: $1"
            shift
            ;;
    esac
done

# If a CSV file is provided, we read it and set the parameters
if [ -n "$CSV_FILE" ]; then
    while IFS=, read -r pos1 pos2 pos3 wait_time; do
        # Ignore empty lines and comment lines
        if [ -z "$pos1" ] || [[ "$pos1" == \#* ]]; then
            continue
        fi
        
        # Set positions and wait time for each row in the CSV
        TARGET_POSITIONS=("$pos1" "$pos2" "$pos3")
        WAIT_TIME="${wait_time:-$DEFAULT_WAIT_TIME}"  # Use the provided wait time, default to 30 if not set
        
        # Display the parameters for debugging purposes
        echo "Target Positions: ${TARGET_POSITIONS[@]}"
        echo "Wait Time: $WAIT_TIME"
        echo "Host: $HOST"
        echo "Port: $PORT"

        # Run the Python script with the parsed arguments
        python3 commands.py "$HOST" "$PORT" --send-multiple-ppmpdo "${TARGET_POSITIONS[@]}" --slave-ids 0 1 2 --wait-time "$WAIT_TIME"
    done < "$CSV_FILE"
else
    # If no CSV file, ask the user for input
    if [ ${#TARGET_POSITIONS[@]} -eq 0 ]; then
        read -p "Enter the target positions (separate by spaces): " -a TARGET_POSITIONS
    fi

    # If no wait time was passed, use the default
    if [ -z "$WAIT_TIME" ]; then
        WAIT_TIME=$DEFAULT_WAIT_TIME
    fi

    # Display the parameters for debugging purposes
    echo "Target Positions: ${TARGET_POSITIONS[@]}"
    echo "Wait Time: $WAIT_TIME"
    echo "Host: $HOST"
    echo "Port: $PORT"

    # Run the Python script with the parsed arguments
    python3 commands.py "$HOST" "$PORT" --send-multiple-ppmpdo "${TARGET_POSITIONS[@]}" --slave-ids 0 1 2 --wait-time "$WAIT_TIME"
fi
