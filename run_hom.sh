#!/bin/bash

HOST="131.215.193.25"
PORT=7469

# Send Homing PDO message
python3 commands.py $HOST $PORT --send-homingpdo


