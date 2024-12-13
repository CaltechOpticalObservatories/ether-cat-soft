#!/bin/bash

HOST="131.215.193.25"
PORT=7469

# Send a single PPM PDO message with target position
python3 commands.py $HOST $PORT --send-ppmpdo 1000
