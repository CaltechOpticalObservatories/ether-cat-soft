%load_ext autoreload
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
%aimport helpers.EPOS4
%aimport helpers.position_camera_helpers
%autoreload 1
import logging
from typing import NamedTuple

# Set up logging
logger = logging.getLogger()
logger.setLevel(logging.DEBUG)
# create file handler which logs even debug messages
fh = logging.FileHandler('EPOS4_repeatability_and_longevity_tests.log')
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

# convert pixels to um
UM_PER_PIXEL = 23
PEAK_WIDTH = 25

def p_to_um(p):
    return p*UM_PER_PIXEL


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

# Set up the camera
WEBCAM_INDEX = 1
ROT_ANG = 90
camera = open_webcam_ueye(WEBCAM_INDEX)

# get original tracker image for each slit
NUMBER_OF_SLITS = 5

class frame_bounds(NamedTuple):
    x1: int = 80
    y1: int = 900
    x2: int = 2300
    y2: int = 1800

bounds = frame_bounds()

GAP_HEIGHT = 100
test_frame = get_frame(camera, bounds, ROT_ANG)
plt.imshow(test_frame)
plt.show()
cv2.imwrite("test_frame.png", test_frame)

regions = get_n_regions(test_frame, NUMBER_OF_SLITS, True)

time.sleep(2)
OFFSET_LIMIT = 0.8

slaves = master.slaves
# slaves = slaves[::-1]

for i in range(len(master.slaves)):
    slave = slaves[i]
    
    # get difference between top and bottom regions
    frame = get_frame(camera, bounds, ROT_ANG)
    regions = get_n_regions(frame, NUMBER_OF_SLITS, False)
    offset = get_pixel_shift_n_regions(regions[0], regions, peak_width=PEAK_WIDTH, display=False)[i + 1]
    logger.info("Offset (X, Y): {}".format(offset))
    
    while (abs(offset[0]) > OFFSET_LIMIT):
        # Move each slave to try and align
        move_value = int(1.5*offset[0])
        clear_faults(slave)
        current_position = get_position_actual_value(slave)
        logging.info("Moving from {} to {}".format(current_position, current_position+move_value))
        set_position_using_csp_mode(slave, current_position+move_value)

        time.sleep(2)

        # get difference between top and bottom regions
        frame = get_frame(camera, bounds, ROT_ANG)
        regions = get_n_regions(frame, NUMBER_OF_SLITS, False)
        offset = get_pixel_shift_n_regions(regions[0], regions, peak_width=PEAK_WIDTH, display=False)[i + 1]
        logger.info("Offset (X, Y): {}".format(offset))

        regions = get_n_regions(frame, NUMBER_OF_SLITS, False)

# get original tracker image for each slit
UPSCALE_FACTOR = 1
MOVE_DISTANCE = -1000

# frame = get_frame(camera, bounds, ROT_ANG)
# initial_regions = get_n_regions(frame, NUMBER_OF_SLITS, False)

initial_positions = []
for slave in master.slaves:
    clear_faults(slave)
    initial_positions.append(get_position_actual_value(slave))
    set_position_using_csp_mode(slave, initial_positions[-1])

slit_offsets = []

time.sleep(1)

def get_highest_frame_number():
    import glob
    import os
    files = glob.glob("frame_*.png")
    return max([int(os.path.splitext(f)[0].split("_")[1]) for f in files])

i = 0
while True:
    frame = get_frame(camera)
    #save the frame
    cv2.imwrite("frame_{}.png".format(get_highest_frame_number()+1), frame)
    regions = get_n_regions(frame, NUMBER_OF_SLITS, False)

    for i in range(len(master.slaves)):
        slave = master.slaves[i]
        clear_faults(slave)
        set_position_using_csp_mode(slave, initial_positions[i]+MOVE_DISTANCE)
    time.sleep(0.5)
    for i in range(len(master.slaves)):
        slave = master.slaves[i]
        clear_faults(slave)
        set_position_using_csp_mode(slave, initial_positions[i])
    time.sleep(2)

    slit_offsets.append(get_pixel_shift_n_regions(regions[0], regions, peak_width=PEAK_WIDTH, display=False))
    logger.info("Test #{} resulted in offsets: {}{}{}{}{}{}".format(i, slit_offsets[-1][1], slit_offsets[-1][2], slit_offsets[-1][3], slit_offsets[-1][4], slit_offsets[-1][5], slit_offsets[-1][6]))