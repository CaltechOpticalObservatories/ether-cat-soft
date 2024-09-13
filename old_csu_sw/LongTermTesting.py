# %%
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

  # %%
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

# %% [markdown]
# # Setup and Home Each Slit

# %%
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


# %%
# Home each slave if the encoder has already been calibrated
for slave in master.slaves:
    logging.info("Homing %s" % slave.name)
    home_slave_using_current_ssi_position(slave)

# %% [markdown]
# # Set up camera and take reference image

set_position_using_csp_mode()

# %%
# Set up the camera
WEBCAM_INDEX = 1
ROT_ANG = 0
camera = open_webcam_ueye(WEBCAM_INDEX)

# %% [markdown]
# ### Camera view of trackers

# %%
# get original tracker image for each slit
NUMBER_OF_SLITS = 6

class frame_bounds(NamedTuple):
    x1: int = 80
    y1: int = 600
    x2: int = 2800
    y2: int = 2200

bounds = frame_bounds()
bounds = None

GAP_HEIGHT = 100
test_frame = get_frame(camera, bounds, ROT_ANG)
plt.imshow(test_frame)
plt.show()
cv2.imwrite("test_frame.png", test_frame)

regions = get_n_regions(test_frame, NUMBER_OF_SLITS, True)

# %% [markdown]
# ### Pixels to um conversion
# 
# By taking a picture of a ruler in the same focal plane as the registration targets, we can compute the number of microns per pixel. The image shown below allows for pixels to be counted and results in a calibration value of approximately 23 microns per pixel.
# 
# <div>
# <img src="calibration.png" width="500"/>
# </div>

# %%
# convert pixels to um
UM_PER_PIXEL = 17
PEAK_WIDTH = 39

def p_to_um(p):
    return p*UM_PER_PIXEL

# %%
# get current offset of each slit
GAP_HEIGHT = 100
frame = get_frame(camera, bounds, ROT_ANG)
plt.imshow(frame)
plt.show()
cv2.imwrite("test_frame.png", test_frame)

regions = get_n_regions(frame, NUMBER_OF_SLITS, True)
print("Region shapes: {}".format([r.shape for r in regions]))
get_pixel_shift_n_regions(regions[0], regions, peak_width=PEAK_WIDTH, display=True)

# %% [markdown]
# ### Use camera to align the slits

# %%
VELOCITY = 1500
ACCELERATION = 500

# # %%
# OFFSET_LIMIT = 0.3
# slaves = master.slaves
# # slaves = slaves[::-1]

# for i in range(len(master.slaves)):
#     slave = slaves[i]
    
#     # get difference between top and bottom regions
#     frame = get_frame(camera, bounds, ROT_ANG)
#     regions = get_n_regions(frame, NUMBER_OF_SLITS, False)
#     offset = get_pixel_shift_n_regions(regions[0], regions, peak_width=PEAK_WIDTH, display=False)[i + 1]
#     logger.info("Offset (X, Y): {}".format(offset))
    
#     while (abs(offset[0]) > OFFSET_LIMIT):
#         # Move each slave to try and align
#         move_value = int(10*offset[0])
#         if (i == 4):
#             move_value = int(100*offset[0])
#         clear_faults(slave)
#         current_position = get_position_actual_value(slave)
#         logging.info("{}: Moving from {} to {}".format(i, current_position, current_position+move_value))
#         set_position_using_ppm_mode(slave, current_position+move_value, VELOCITY, ACCELERATION, True)

#         time.sleep(2)

#         # get difference between top and bottom regions
#         frame = get_frame(camera, bounds, ROT_ANG)
#         regions = get_n_regions(frame, NUMBER_OF_SLITS, False)
#         offset = get_pixel_shift_n_regions(regions[0], regions, peak_width=PEAK_WIDTH, display=False)[i + 1]
#         logger.info("{}: Offset (X, Y): {}".format(i, offset))

#         regions = get_n_regions(frame, NUMBER_OF_SLITS, False)

# %% [markdown]
# # Full test but with faster implementation of subpixel analysis

# %%
# get original tracker image for each slit
UPSCALE_FACTOR = 1
MOVE_DISTANCE = -100000

VELOCITIES = [1700, 9500, 9500, 9500, 50000]
ACCELERATIONS = [500, 500, 500, 500, 5000]
MOVE_DISTANCES = [MOVE_DISTANCE//2, MOVE_DISTANCE, MOVE_DISTANCE, MOVE_DISTANCE, MOVE_DISTANCE*5]

# frame = get_frame(camera, bounds, ROT_ANG)
# initial_regions = get_n_regions(frame, NUMBER_OF_SLITS, False)
#
# initial_positions = []
# for i in range(len(master.slaves)):
#     slave = master.slaves[i]
#     clear_faults(slave)
#     initial_positions.append(get_position_actual_value(slave))
#     set_position_using_ppm_mode(slave, initial_positions[i], VELOCITIES[i], ACCELERATIONS[i])
#
# slit_offsets = []
#
# time.sleep(1)
#
#
# for i in range(200):
#     frame = get_frame(camera, bounds, ROT_ANG)
#     regions = get_n_regions(frame, NUMBER_OF_SLITS, False)
#
#     for i in range(len(master.slaves)):
#         slave = master.slaves[i]
#         # clear_faults(slave)
#         set_position_using_ppm_mode_fast(slave, initial_positions[i]+MOVE_DISTANCES[i], VELOCITIES[i], ACCELERATIONS[i])
#
#     time.sleep(9)
#     for i in range(len(master.slaves)):
#         slave = master.slaves[i]
#         # clear_faults(slave)
#         set_position_using_ppm_mode_fast(slave, initial_positions[i], VELOCITIES[i], ACCELERATIONS[i])
#
#     time.sleep(10)
#     # logger.info("First bar actual position: {}".format(get_position_actual_value(master.slaves[0])))
#     # slit_offsets.append(get_pixel_shift_n_regions(regions[0], regions, peak_width=PEAK_WIDTH, display=False))
#     # logger.info("Test #{} resulted in offsets: {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}".format(i, slit_offsets[-1][0][0], slit_offsets[-1][1][0], slit_offsets[-1][2][0], slit_offsets[-1][3][0], slit_offsets[-1][4][0], slit_offsets[-1][5][0]))

# %%
# get original tracker image for each slit
UPSCALE_FACTOR = 1
MOVE_DISTANCE = -100000

VELOCITIES = [1700, 9500, 9500, 9500, 50000]
ACCELERATIONS = [500, 500, 500, 500, 5000]
MOVE_DISTANCES = [MOVE_DISTANCE // 2, MOVE_DISTANCE, MOVE_DISTANCE, MOVE_DISTANCE, MOVE_DISTANCE * 5.5]

# frame = get_frame(camera, bounds, ROT_ANG)
# initial_regions = get_n_regions(frame, NUMBER_OF_SLITS, False)


initial_positions = []
for i in range(len(master.slaves)):
    slave = master.slaves[i]
    clear_faults(slave)
    initial_positions.append(get_position_actual_value(slave))
    set_position_using_ppm_mode(slave, initial_positions[i], VELOCITIES[i], ACCELERATIONS[i])

slit_offsets = []

time.sleep(1)

for i in range(200):
    frame = get_frame(camera, bounds, ROT_ANG)
    regions = get_n_regions(frame, NUMBER_OF_SLITS, False)

    t0 = time.time()
    run_on_multiple_slaves(set_controlword, master.slaves, CONTROLWORD_COMMAND_SHUTDOWN)
    # time.sleep(i)
    t1 = time.time()
    # run_on_multiple_slaves(get_statusword, master.slaves)
    run_on_multiple_slaves(wait_for_statusword_bits, master.slaves, STATUSWORD_COMMAND_SHUTDOWN_MASK, STATUSWORD_COMMAND_SHUTDOWN_VALUE)
    # time.sleep(i)
    t2 = time.time()
    run_on_multiple_slaves(set_controlword, master.slaves, CONTROLWORD_COMMAND_SWITCH_ON_AND_ENABLE)
    # time.sleep(i)
    t3 = time.time()
    # run_on_multiple_slaves(get_statusword, master.slaves)
    run_on_multiple_slaves(wait_for_statusword_bits, master.slaves, STATUSWORD_COMMAND_SWITCH_ON_AND_ENABLE_MASK, STATUSWORD_COMMAND_SWITCH_ON_AND_ENABLE_VALUE)
    # time.sleep(i)
    t4 = time.time()
    for j in range(len(master.slaves)):
        slave = master.slaves[j]
        # clear_faults(slave)
        set_target_position(slave, int(initial_positions[j] + MOVE_DISTANCES[j]))
    run_on_multiple_slaves(set_controlword, master.slaves, CONTROLWORD_COMMAND_ABSOLUTE_START_IMMEDIATELY)
    # logger.info("delay = {}, t0 = {}, t1 = {}, t2 = {}, t3 = {}, t4 = {}".format(i, t1-t0, t2-t1, t3-t2, t4-t3, time.time()-t4))
    logger.info("total time = {}".format(time.time()-t0))
    # time.sleep(10)
    run_on_multiple_slaves(set_controlword, master.slaves, CONTROLWORD_COMMAND_SHUTDOWN)
    run_on_multiple_slaves(wait_for_statusword_bits, master.slaves, 0b111111, 0b100001)
    run_on_multiple_slaves(set_controlword, master.slaves, CONTROLWORD_COMMAND_SWITCH_ON_AND_ENABLE)
    run_on_multiple_slaves(wait_for_statusword_bits, master.slaves, 0b111111, 0b110111)
    for j in range(len(master.slaves)):
        slave = master.slaves[j]
        # clear_faults(slave)
        set_target_position(slave, int(initial_positions[j]))
    run_on_multiple_slaves(set_controlword, master.slaves, CONTROLWORD_COMMAND_ABSOLUTE_START_IMMEDIATELY)

    # time.sleep(10)
    # logger.info("First bar actual position: {}".format(get_position_actual_value(master.slaves[0])))
    # slit_offsets.append(get_pixel_shift_n_regions(regions[0], regions, peak_width=PEAK_WIDTH, display=False))
    # logger.info("Test #{} resulted in offsets: {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}".format(i, slit_offsets[-1][0][0], slit_offsets[-1][1][0], slit_offsets[-1][2][0], slit_offsets[-1][3][0], slit_offsets[-1][4][0], slit_offsets[-1][5][0]))

#%%
# run_on_multiple_slaves(test_run_on_multiple_slaves, master.slaves, 19, 20, 10)
