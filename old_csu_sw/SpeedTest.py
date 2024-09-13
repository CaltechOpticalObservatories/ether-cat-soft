import cv2
import numpy as np
import matplotlib.pyplot as plt
import scipy.signal as signal
import ctypes
import time
import pysoem
import importlib
from helpers.EPOS4 import *
from helpers.position_camera_helpers import *
import logging
from typing import NamedTuple

logger = logging.getLogger()
logger.setLevel(logging.DEBUG)
# create file handler which logs even debug messages
fh = logging.FileHandler('SpeedTest.log')
fh.setLevel(logging.DEBUG)
# create console handler with a higher log level
ch = logging.StreamHandler()
ch.setLevel(logging.INFO)
# create formatter and add it to the handlers
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
fh.setFormatter(formatter)
ch.setFormatter(formatter)
# add the handlers to logger
logger.addHandler(fh)
logger.addHandler(ch)

# wait for keypress to start
input("Press Enter to start...")
start_time = time.time()

# Set up pysoem
time.sleep(6)
if_offset = 4
ifname = pysoem.find_adapters()[if_offset].name
logging.info(pysoem.find_adapters()[if_offset])
try:
    while True:
        master = None
        master = pysoem.Master(ifname)
        master.close()
        # time.sleep(2)
        master.open(ifname)
        if master.config_init() > 0:
            logging.info("Found {} slaves".format(len(master.slaves)))
            if len(master.slaves) < 4:
                logging.error("Not enough slaves found!")
                continue
            for slave in master.slaves:
                slave.recover()
                logging.info("Found {} with Product ID {}".format(slave.name, slave.id))
        else:
            logging.error("No slaves found!")
            continue
        break
except OSError as e:
    logging.error("Error while finding slaves: %s" % e)

# Home each slave if the encoder has already been calibrated
for slave in master.slaves:
    logging.info("Homing %s" % slave.name)
    home_slave_using_current_ssi_position(slave)

# perform a move
MOVE_DISTANCE = -50000
VELOCITIES = [1700, 9500, 9500, 9500, 50000]
ACCELERATIONS = [500, 500, 500, 500, 5000]
MOVE_DISTANCES = [MOVE_DISTANCE//2, MOVE_DISTANCE, MOVE_DISTANCE, MOVE_DISTANCE, MOVE_DISTANCE*5]

initial_positions = []
for i in range(len(master.slaves)):
    slave = master.slaves[i]
    logging.info("Moving %s" % slave.name)
    clear_faults(slave)
    initial_positions.append(get_position_actual_value(slave))
    # set_position_using_ppm_mode(slave, initial_positions[i]+MOVE_DISTANCES[i], VELOCITIES[i], ACCELERATIONS[i])

logging.info("took %s seconds to move" % (time.time() - start_time))

for i in range(len(master.slaves)):
    slave = master.slaves[i]
    set_position_using_ppm_mode(slave, initial_positions[i], VELOCITIES[i], ACCELERATIONS[i])

while True:
    for i in range(len(master.slaves)):
        slave = master.slaves[i]
        logging.info("Moving %s" % slave.name)
        clear_faults(slave)
        time.sleep(1)
        set_position_using_ppm_mode(slave, initial_positions[i]+MOVE_DISTANCES[i], VELOCITIES[i], ACCELERATIONS[i])
    time.sleep(10)
    for i in range(len(master.slaves)):
        slave = master.slaves[i]
        set_position_using_ppm_mode(slave, initial_positions[i], VELOCITIES[i], ACCELERATIONS[i])
    time.sleep(10)
