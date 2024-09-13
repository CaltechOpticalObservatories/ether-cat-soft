# import cv2
import numpy as np
# import matplotlib.pyplot as plt
# import scipy.signal as signal
import ctypes
import time
import pysoem
# import importlib
from helpers.EPOS4 import *
import logging
from typing import NamedTuple

# Set up logging
logger = logging.getLogger()
logger.setLevel(logging.DEBUG)
# create file handler which logs even debug messages
fh = logging.FileHandler('TMT_Slits_Demo.log')
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


# Set up pysoem
if_offset = 4
ifname = pysoem.find_adapters()[if_offset].name
master = None
try:
    master = pysoem.Master(ifname)
    master.close()
    master.open(ifname)
    if master.config_init() > 0:
        logging.info("Found {} slaves".format(len(master.slaves)))
        for slave in master.slaves:
            slave.recover()
            logging.info("Found {} with Product ID {}".format(slave.name, slave.id))
    else:
        logging.error("No slaves found!")
        
except OSError as e:
    logging.error("Error while finding slaves: %s" % e)


# Home each slave if the encoder has already been calibrated
for slave in master.slaves:
    logging.info("Homing %s" % slave.name)
    home_slave_using_current_ssi_position(slave)

# get slave starting positions
current_positions = []
for slave in master.slaves:
    current_positions.append(get_position_actual_value(slave))

# Move each slave
move_value = -20000

while (True):
    for i in range(len(master.slaves)):
        slave = master.slaves[i]
        logging.info("Moving {} from {} to {}".format(slave.name, current_positions[i], current_positions[i]+move_value))
        set_position_using_csp_mode(slave, current_positions[i]+move_value)
        time.sleep(2)
    for i in range(len(master.slaves)):
        slave = master.slaves[i]
        set_position_using_csp_mode(slave, current_positions[i])
        time.sleep(2)
