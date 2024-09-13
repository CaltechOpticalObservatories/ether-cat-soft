"""! @package EPOS4_support
@brief This module contains functions to support the EPOS4 motor controller
@file EPOS4_support.py
@author Jake Zimmer
@date 2023.07.31
@version 0.1.0
@details This module uses the pysoem library to communicate with the EPOS4 motor controller. While
    Maxon provides a python library for communicating with the EPOS4, it is not compatible with
    EtherCAT. This module therefore leverages the pysoem library to communicate with the EPOS4.
"""

import time
import pysoem
import ctypes
import struct
import logging
from helpers.CSU_config_defaults import *

"""! Miscellaneous constants"""
CONTROLWORD_DELAY_TIME = 0.01
STATUSWORD_DELAY_TIME = 0.01

"""! Homing method enumerations"""
HOMING_METHOD_ACTUAL_POSITION = 37
HOMING_METHOD_INDEX_POSITIVE_SPEED = 34
HOMING_METHOD_INDEX_NEGATIVE_SPEED = 33
HOMING_METHOD_HOME_SWITCH_POSITIVE_SPEED = 23
HOMING_METHOD_HOME_SWITCH_NEGATIVE_SPEED = 27
HOMING_METHOD_LIMIT_SWITCH_POSITIVE = 18
HOMING_METHOD_LIMIT_SWITCH_NEGATIVE = 17

"""! Mode of operation enumerations"""
MODE_OF_OPERATION_PROFILE_POSITION = 1
MODE_OF_OPERATION_PROFILE_VELOCITY = 3
MODE_OF_OPERATION_HOMING = 6
MODE_OF_OPERATION_CYCLIC_SYNCHRONOUS_POSITION = 8
MODE_OF_OPERATION_CYCLIC_SYNCHRONOUS_VELOCITY = 9
MODE_OF_OPERATION_CYCLIC_SYNCHRONOUS_TORQUE = 10

"""! Controlword bit offsets"""
CONTROLWORD_BIT_OFFSET_HALT = 8
CONTROLWORD_BIT_OFFSET_START_HOMING = 4
CONTROLWORD_BIT_OFFSET_QUICK_STOP = 2
CONTROLWORD_BIT_OFFSET_ENABLE_VOLTAGE = 1
CONTROLWORD_BIT_OFFSET_ENABLE_OPERATION = 3
CONTROLWORD_BIT_OFFSET_NEW_SETPOINT = 4
CONTROLWORD_BIT_OFFSET_CHANGE_SET_IMMEDIATELY = 5
CONTROLWORD_BIT_OFFSET_ABSOLUTE_RELATIVE = 6
CONTROLWORD_BIT_OFFSET_FAULT_RESET = 7

"""! Controlword Common Commands"""
CONTROLWORD_COMMAND_SHUTDOWN = 0x0006
CONTROLWORD_COMMAND_SWITCH_ON_AND_ENABLE = 0x000F
CONTROLWORD_COMMAND_START_HOMING = 0x001F
CONTROLWORD_COMMAND_QUICK_STOP = 0x000B
CONTROLWORD_COMMAND_HALT_HOMING = 0x011F
CONTROLWORD_COMMAND_ABSOLUTE_START_IMMEDIATELY = 0x003F


"""! Statusword bit offsets"""
STATUSWORD_BIT_OFFSET_READY_TO_SWITCH_ON = 0
STATUSWORD_BIT_OFFSET_SWITCHED_ON = 1
STATUSWORD_BIT_OFFSET_OPERATION_ENABLED = 2
STATUSWORD_BIT_OFFSET_FAULT = 3
STATUSWORD_BIT_OFFSET_VOLTAGE_ENABLED = 4
STATUSWORD_BIT_OFFSET_QUICK_STOP = 5
STATUSWORD_BIT_OFFSET_SWITCH_ON_DISABLED = 6
STATUSWORD_BIT_OFFSET_WARNING = 7
STATUSWORD_BIT_OFFSET_REMOTE = 9
STATUSWORD_BIT_OFFSET_TARGET_REACHED = 10
STATUSWORD_BIT_OFFSET_INTERNAL_LIMIT_ACTIVE = 11
STATUSWORD_BIT_OFFSET_HOMING_ATTAINED = 12
STATUSWORD_BIT_OFFSET_HOMING_ERROR = 13
STATUSWORD_BIT_OFFSET_POSITION_REFERENCE_TO_HOME = 15

"""! Statusword Common Commands"""
STATUSWORD_COMMAND_SHUTDOWN_VALUE = 0b0100001
STATUSWORD_COMMAND_SHUTDOWN_MASK = 0b0111111
STATUSWORD_COMMAND_SWITCH_ON_AND_ENABLE_VALUE = 0b0110111
STATUSWORD_COMMAND_SWITCH_ON_AND_ENABLE_MASK = 0b0111111

"""! Motor type"""
MOTOR_TYPE_DC = 1
MOTOR_TYPE_BLDC_SINUSOIDAL = 10
MOTOR_TYPE_BLDC_TRAPEZOIDAL = 11

"""PDO Classes"""
class InputPDO_PPM(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ('statusword', ctypes.c_uint16),
        ('position_actual_value', ctypes.c_int32),
        ('velocity_actual_value', ctypes.c_int32),
        ('following_error_actual_value', ctypes.c_int32),
        ('modes_of_operation_display', ctypes.c_uint8),
        ('digital_inputs', ctypes.c_uint8),
    ]

class OutputPDO_PPM(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ('controlword', ctypes.c_uint16),
        ('target_position', ctypes.c_int32),
        ('profile_acceleration', ctypes.c_uint32),
        ('profile_deceleration', ctypes.c_uint32),
        ('profile_velocity', ctypes.c_uint32),
        ('modes_of_operation', ctypes.c_uint8),
        ('digital_outputs', ctypes.c_uint8),
    ]

############################################
# Low-level helper functions
############################################
def set_bit(value, is_set, offset):
    """! Set a bit in a value
    @param value: the value to set the bit in
    @param is_set: the value to set the bit to
    @param offset: the offset of the bit to set
    @return: the value with the bit set
    """
    if (is_set != 0) and (is_set != 1):
        print('is_set must be 0 or 1')
        return value
    
    if (is_set):
        return value | (is_set << offset)
    else:
        return value & ~(is_set << offset)
    
def get_bit(value, offset):
    """! Get a bit in a value
    @param value: the value to get the bit from
    @param offset: the offset of the bit to get
    @return: the value of the bit
    """
    return (value >> offset) & 1

def bit_offset_to_mask(offset):
    """! Convert a bit offset to a mask
    @param offset: the offset of the bit to convert
    @return: the mask of the bit
    """
    return 1 << offset

############################################
# Low-level EPOS4 functions
############################################
def get_device_type(slave):
    """! Get the device type of the slave
    @param slave: the slave to get the device type from
    @return: the device type of the slave
    """
    
    if slave == None:
        logging.error('get_device_type: no slave available')
        return None
    
    device_type = ctypes.c_uint32.from_buffer_copy(slave.sdo_read(0x1000, 0)).value
    return device_type

def get_error_register(slave):
    """! Get the error register of the slave
    @param slave: the slave to get the error register from
    @return: the error register of the slave
    """
    
    if slave == None:
        logging.error('get_error_register: no slave available')
        return None
    
    error_register = ctypes.c_uint8.from_buffer_copy(slave.sdo_read(0x1001, 0)).value
    return error_register

def get_error_code(slave):
    """! Get the error code of the slave
    @param slave: the slave to get the error code from
    @return: the error code of the slave
    """
    
    if slave == None:
        logging.error('get_error_code: no slave available')
        return None
    
    error_code = ctypes.c_uint16.from_buffer_copy(slave.sdo_read(0x603F, 0)).value
    return error_code

def set_controlword(slave, controlword):
    """! Set the controlword of the slave
    @param slave: the slave to set the controlword of
    @param controlword: the controlword to set
    """
    
    if slave == None:
        logging.error('set_controlword: no slave available')
        return
    
    slave.sdo_write(0x6040, 0, bytes(ctypes.c_uint16(controlword)))
    time.sleep(CONTROLWORD_DELAY_TIME) # Needed before continuing or checking statusword

def get_controlword(slave):
    """! Get the controlword of the slave
    @param slave: the slave to get the controlword from
    @return: the controlword of the slave
    """
    
    if slave == None:
        logging.error('get_controlword: no slave available')
        return None
    
    controlword = ctypes.c_uint16.from_buffer_copy(slave.sdo_read(0x6040, 0)).value
    return controlword

def set_controlword_from_bitmasks(slave, set_bitmask, clear_bitmask, wait_for_completion = True):
    """! Set the controlword of the slave from bitmasks
    @param slave: the slave to set the controlword of
    @param set_bitmask: the bitmask of bits to set
    @param clear_bitmask: the bitmask of bits to clear
    @param wait_for_completion: whether to wait for the command to complete
    """

    if slave == None:
        logging.error('set_controlword_from_bitmasks: no slave available')
        return
    
    controlword = get_controlword(slave)
    controlword = controlword | set_bitmask
    controlword = controlword & ~clear_bitmask
    set_controlword(slave, controlword)
    #
    # bitmask = set_bitmask | ~clear_bitmask
    # bitvalues = set_bitmask
    #
    # if wait_for_completion:
    #     wait_for_statusword_bits(slave, bitmask, bitvalues)

def get_statusword(slave):
    """! Get the statusword of the slave
    @param slave: the slave to get the statusword from
    @return: the statusword of the slave
    """
    
    if slave == None:
        logging.error('get_statusword: no slave available')
        return None
    
    statusword = ctypes.c_uint16.from_buffer_copy(slave.sdo_read(0x6041, 0)).value
    return statusword

def wait_for_statusword_bits(slave, bitmask, bitvalues = -1, timeout = 1):
    """! Wait for the statusword bits of the slave to be set
    @param slave: the slave to wait for the statusword bits of
    @param bitmask: the bitmask of the statusword bits to wait for
    @param bitvalues: the values of the statusword bits to wait for if different from the bitmask
    @param timeout: the timeout in seconds
    @return: True if the statusword bits were set, False otherwise
    """
    
    if slave == None:
        logging.error('wait_for_statusword_bits: no slave available')
        return False
    
    if bitvalues == -1:
        bitvalues = bitmask

    start_time = time.time()
    while True:
        statusword = get_statusword(slave)
        if (statusword & bitmask) == bitvalues:
            # logging.info("took {} seconds to wait for statusword bits".format(time.time() - start_time))
            return True
        if time.time() - start_time > timeout:
            logging.info("timeout waiting for statusword ({}) with bitmask ({}) and bitvalues ({})".format(bin(statusword), bin(bitmask), bin(bitvalues)))
            return False
        time.sleep(STATUSWORD_DELAY_TIME)

def get_mode_of_operation(slave):
    """! Get the mode of operation of the slave
    @param slave: the slave to get the mode of operation from
    @return: the mode of operation of the slave
    """
    
    if slave == None:
        logging.error('get_mode_of_operation: no slave available')
        return None
    
    mode_of_operation = ctypes.c_uint8.from_buffer_copy(slave.sdo_read(0x6061, 0)).value
    return mode_of_operation

def set_mode_of_operation(slave, mode_of_operation, timeout=1):
    """! Set the mode of operation of the slave
    @param slave: the slave to set the mode of operation of
    @param mode_of_operation: the mode of operation to set
    """
    
    if slave == None:
        logging.error('set_mode_of_operation: no slave available')
        return
    
    slave.sdo_write(0x6060, 0, bytes(ctypes.c_uint8(mode_of_operation)))

def get_position_demand_value(slave):
    """! Get the position demand value of the slave
    @param slave: the slave to get the position demand value from
    @return: the position demand value of the slave
    """
    
    if slave == None:
        logging.error('get_position_demand_value: no slave available')
        return None
    
    position_demand_value = ctypes.c_int32.from_buffer_copy(slave.sdo_read(0x6062, 0)).value
    return position_demand_value

def get_position_actual_value(slave):
    """! Get the position actual value of the slave
    @param slave: the slave to get the position actual value from
    @return: the position actual value of the slave
    """
    
    if slave == None:
        logging.error('get_postion_actual_value: no slave available')
        return None
    
    position_actual_value = ctypes.c_int32.from_buffer_copy(slave.sdo_read(0x6064, 0)).value
    return position_actual_value

def get_velocity_demand_value(slave):
    """! Get the velocity demand value of the slave
    @param slave: the slave to get the velocity demand value from
    @return: the velocity demand value of the slave
    """
    
    if slave == None:
        logging.error('get_velocity_demand_value: no slave available')
        return None
    
    velocity_demand_value = ctypes.c_int32.from_buffer_copy(slave.sdo_read(0x606B, 0)).value
    return velocity_demand_value

def get_velocity_actual_value(slave):
    """! Get the velocity actual value of the slave
    @param slave: the slave to get the velocity actual value from
    @return: the velocity actual value of the slave
    """
    
    if slave == None:
        logging.error('get_velocity_actual_value: no slave available')
        return None
    
    velocity_actual_value = ctypes.c_int32.from_buffer_copy(slave.sdo_read(0x606C, 0)).value
    return velocity_actual_value

def set_target_torque(slave, target_torque):
    """! Set the target torque of the slave
    @param slave: the slave to set the target torque of
    @param target_torque: the target torque to set
    """
    
    if slave == None:
        logging.error('set_target_torque: no slave available')
        return
    
    slave.sdo_write(0x6071, 0, bytes(ctypes.c_int16(target_torque)))

def get_target_torque(slave):
    """! Get the target torque of the slave
    @param slave: the slave to get the target torque from
    @return: the target torque of the slave
    """
    
    if slave == None:
        logging.error('get_target_torque: no slave available')
        return None
    
    target_torque = ctypes.c_int16.from_buffer_copy(slave.sdo_read(0x6071, 0)).value
    return target_torque

def get_torque_actual_value(slave):
    """! Get the torque actual value of the slave
    @param slave: the slave to get the torque actual value from
    @return: the torque actual value of the slave
    """
    
    if slave == None:
        logging.error('get_torque_actual_value: no slave available')
        return None
    
    torque_actual_value = ctypes.c_int16.from_buffer_copy(slave.sdo_read(0x6077, 0)).value
    return torque_actual_value

def set_target_position(slave, target_position):
    """! Set the target position of the slave
    @param slave: the slave to set the target position of
    @param target_position: the target position to set
    """
    
    if slave == None:
        logging.error('set_target_position: no slave available')
        return
    
    slave.sdo_write(0x607A, 0, bytes(ctypes.c_int32(target_position)))

def set_min_position_limit(slave, min_position_limit):
    """! Set the min position limit of the slave
    @param slave: the slave to set the min position limit of
    @param min_position_limit: the min position limit to set
    """
    
    if slave == None:
        logging.error('set_min_position_limit: no slave available')
        return
    
    slave.sdo_write(0x607D, 1, bytes(ctypes.c_int32(min_position_limit)))

def set_max_position_limit(slave, max_position_limit):
    """! Set the max position limit of the slave
    @param slave: the slave to set the max position limit of
    @param max_position_limit: the max position limit to set
    """
    
    if slave == None:
        logging.error('set_max_position_limit: no slave available')
        return
    
    slave.sdo_write(0x607D, 2, bytes(ctypes.c_int32(max_position_limit)))

def set_max_profile_velocity(slave, max_profile_velocity):
    """! Set the max profile velocity of the slave
    @param slave: the slave to set the max profile velocity of
    @param max_profile_velocity: the max profile velocity to set
    """
    
    if slave == None:
        logging.error('set_max_profile_velocity: no slave available')
        return
    
    if (max_profile_velocity > 0x7FFFFFFF or max_profile_velocity <= 0):
        logging.error('set_max_profile_velocity: max_profile_velocity out of range')
        return
    
    slave.sdo_write(0x607F, 0, bytes(ctypes.c_uint32(max_profile_velocity)))

def set_max_motor_speed(slave, max_motor_speed):
    """! Set the max motor speed of the slave
    @param slave: the slave to set the max motor speed of
    @param max_motor_speed: the max motor speed to set
    """
    
    if slave == None:
        logging.error('set_max_motor_speed: no slave available')
        return
    
    slave.sdo_write(0x6080, 0, bytes(ctypes.c_uint32(max_motor_speed)))

def set_profile_velocity(slave, profile_velocity):
    """! Set the profile velocity of the slave
    @param slave: the slave to set the profile velocity of
    @param profile_velocity: the profile velocity to set
    """
    
    if slave == None:
        logging.error('set_profile_velocity: no slave available')
        return
    
    if (profile_velocity > 0x7FFFFFFF or profile_velocity <= 0):
        logging.error('set_profile_velocity: profile_velocity out of range')
        return
    
    slave.sdo_write(0x6081, 0, bytes(ctypes.c_uint32(profile_velocity)))

def set_profile_acceleration(slave, profile_acceleration):
    """! Set the profile acceleration of the slave
    @param slave: the slave to set the profile acceleration of
    @param profile_acceleration: the profile acceleration to set
    """
    
    if slave == None:
        logging.error('set_profile_acceleration: no slave available')
        return
    
    slave.sdo_write(0x6083, 0, bytes(ctypes.c_uint32(profile_acceleration)))

def set_profile_deceleration(slave, profile_deceleration):
    """! Set the profile deceleration of the slave
    @param slave: the slave to set the profile deceleration of
    @param profile_deceleration: the profile deceleration to set
    """
    
    if slave == None:
        logging.error('set_profile_deceleration: no slave available')
        return
    
    slave.sdo_write(0x6084, 0, bytes(ctypes.c_uint32(profile_deceleration)))

def set_quick_stop_deceleration(slave, quick_stop_deceleration):
    """! Set the quick stop deceleration of the slave
    @param slave: the slave to set the quick stop deceleration of
    @param quick_stop_deceleration: the quick stop deceleration to set
    """
    
    if slave == None:
        logging.error('set_quick_stop_deceleration: no slave available')
        return
    
    slave.sdo_write(0x6085, 0, bytes(ctypes.c_uint32(quick_stop_deceleration)))

def set_homing_method(slave, homing_method):
    """! Set the homing method of the slave
    @param slave: the slave to set the homing method of
    @param homing_method: the homing method to set
    """
    
    if slave == None:
        logging.error('set_homing_method: no slave available')
        return
    
    slave.sdo_write(0x6098, 0, bytes(ctypes.c_int8(homing_method)))

def set_homing_speed(slave, homing_speed):
    """! Set the homing speeds of the slave
    @param slave: the slave to set the homing speeds of
    @param homing_speed: the homing speeds to set
    """
    
    if slave == None:
        logging.error('set_homing_speed: no slave available')
        return
    
    slave.sdo_write(0x6099, 1, bytes(ctypes.c_uint32(homing_speed)))
    slave.sdo_write(0x6099, 2, bytes(ctypes.c_uint32(homing_speed)))

def set_homing_acceleration(slave, homing_acceleration):
    """! Set the homing acceleration of the slave
    @param slave: the slave to set the homing acceleration of
    @param homing_acceleration: the homing acceleration to set
    """
    
    if slave == None:
        logging.error('set_homing_acceleration: no slave available')
        return
    
    slave.sdo_write(0x609A, 0, bytes(ctypes.c_uint32(homing_acceleration)))

def set_home_position(slave, home_position):
    """! Set the home position of the slave
    @param slave: the slave to set the home position of
    @param home_position: the home position to set
    """
    
    if slave == None:
        logging.error('set_home_position: no slave available')
        return
    
    slave.sdo_write(0x30B0, 0, bytes(ctypes.c_int32(home_position)))

def set_home_offset_move_distance(slave, home_offset_move_distance):
    """! Set the home offset move distance of the slave
    @param slave: the slave to set the home offset move distance of
    @param home_offset_move_distance: the home offset move distance to set
    """
    
    if slave == None:
        logging.error('set_home_offset_move_distance: no slave available')
        return
    
    slave.sdo_write(0x30B1, 0, bytes(ctypes.c_uint32(home_offset_move_distance)))

def set_position_offset(slave, position_offset):
    """! Set the position offset of the slave
    @param slave: the slave to set the position offset of
    @param position_offset: the position offset to set
    """
    
    if slave == None:
        logging.error('set_position_offset: no slave available')
        return
    
    slave.sdo_write(0x60B0, 0, bytes(ctypes.c_int32(position_offset)))

def set_velocity_offset(slave, velocity_offset):
    """! Set the velocity offset of the slave
    @param slave: the slave to set the velocity offset of
    @param velocity_offset: the velocity offset to set
    """
    
    if slave == None:
        logging.error('set_velocity_offset: no slave available')
        return
    
    slave.sdo_write(0x60B1, 0, bytes(ctypes.c_int32(velocity_offset)))

def set_torque_offset(slave, torque_offset):
    """! Set the torque offset of the slave
    @param slave: the slave to set the torque offset of
    @param torque_offset: the torque offset to set
    """
    
    if slave == None:
        logging.error('set_torque_offset: no slave available')
        return
    
    slave.sdo_write(0x60B2, 0, bytes(ctypes.c_int16(torque_offset)))

def get_following_error_actual_value(slave):
    """! Get the following error actual value of the slave
    @param slave: the slave to get the following error actual value of
    @return: the following error actual value of the slave
    """
    
    if slave == None:
        logging.error('get_following_error_actual_value: no slave available')
        return None
    
    following_error_actual_value = ctypes.c_int32.from_buffer_copy(slave.sdo_read(0x6077, 0)).value
    return following_error_actual_value

def get_additional_position_actual_values(slave):
    """! Get the additional position actual values of the slave
    @param slave: the slave to get the additional position actual values of
    @return: the additional position actual values of the slave (sensor 1, sensor 2, sensor 3)
    """
    
    if slave == None:
        logging.error('get_additional_position_actual_values: no slave available')
        return None
    
    value_sensor_1 = ctypes.c_int32.from_buffer_copy(slave.sdo_read(0x60E4, 1)).value
    value_sensor_2 = ctypes.c_int32.from_buffer_copy(slave.sdo_read(0x60E4, 2)).value
    value_sensor_3 = ctypes.c_int32.from_buffer_copy(slave.sdo_read(0x60E4, 3)).value
    return (value_sensor_1, value_sensor_2, value_sensor_3)

def get_ssi_position_raw_value_complete(slave):
    """! Get the complete SSI position raw value of the slave
    @param slave: the slave to get the complete SSI position raw value of
    @return: the complete SSI position raw value of the slave
    """
    
    if slave == None:
        logging.error('get_ssi_position_raw_value_complete: no slave available')
        return None
    
    ssi_position_raw_value_complete = ctypes.c_uint64.from_buffer_copy(slave.sdo_read(0x3012, 0x0D)).value
    return ssi_position_raw_value_complete

def set_node_id(slave, node_id=1):
    """! Set the node ID of the slave
    @param slave: the slave to set the node ID of
    @param node_id: the node ID to set
    """
    
    if slave == None:
        logging.error('set_node_id: no slave available')
        return
    
    slave.sdo_write(0x2000, 0, bytes(ctypes.c_uint8(node_id)))

def set_motor_type(slave, motor_type):
    """! Set the motor type of the slave
    @param slave: the slave to set the motor type of
    @param motor_type: the motor type to set
    """
    
    if slave == None:
        logging.error('set_motor_type: no slave available')
        return
    
    slave.sdo_write(0x6402, 0, bytes(ctypes.c_uint16(motor_type)))

def set_motor_data(slave, nominal_current_ma, output_current_limit_ma, number_of_pole_pairs,
                   thermal_time_constant_winding_ms, torque_constant_uNm_A):
    """! Set the motor data of the slave
    @param slave: the slave to set the motor data of
    @param nominal_current_ma: the nominal current in mA
    @param output_current_limit_ma: the output current limit in mA
    @param number_of_pole_pairs: the number of pole pairs
    @param thermal_time_constant_winding_ms: the thermal time constant of the winding in ms
    @param torque_constant_uNm_A: the torque constant in uNm/A
    """

    if slave == None:
        logging.error('set_motor_data: no slave available')
        return
    
    slave.sdo_write(0x3001, 1, bytes(ctypes.c_uint32(nominal_current_ma)))
    slave.sdo_write(0x3001, 2, bytes(ctypes.c_uint32(output_current_limit_ma)))
    slave.sdo_write(0x3001, 3, bytes(ctypes.c_uint8(number_of_pole_pairs)))
    slave.sdo_write(0x3001, 4, bytes(ctypes.c_uint16(thermal_time_constant_winding_ms//100)))
    slave.sdo_write(0x3001, 5, bytes(ctypes.c_uint16(torque_constant_uNm_A)))

def set_gear_data(slave, gear_reduction_numberator, gear_reduction_denominator, 
                  max_input_speed_rpm, orientation):
    """! Set the gear data of the slave
    @param slave: the slave to set the gear data of
    @param gear_reduction_numberator: the gear reduction numberator
    @param gear_reduction_denominator: the gear reduction denominator
    @param max_input_speed_rpm: the max input speed in rpm
    @param orientation: the orientation of the motor (0: normal, 1: inverted)
    """
    
    if slave == None:
        logging.error('set_gear_data: no slave available')
        return
    
    slave.sdo_write(0x3003, 1, bytes(ctypes.c_uint32(gear_reduction_numberator)))
    slave.sdo_write(0x3003, 2, bytes(ctypes.c_uint32(gear_reduction_denominator)))
    slave.sdo_write(0x3003, 3, bytes(ctypes.c_uint32(max_input_speed_rpm)))
    slave.sdo_write(0x3003, 4, bytes(ctypes.c_uint32(orientation)))

def set_digital_incremental_encoder_data(slave, number_of_pulses_per_turn, encoder_type, direction, method):
    """! Set the digital incremental encoder data of the slave
    @param slave: the slave to set the digital incremental encoder data of
    @param number_of_pulses_per_turn: the number of pulses per turn
    @param encoder_type: the encoder type (0: encoder without index (2-channel), 1: encoder with index (3-channel), 3: encoder with index (3-channel without index supervision))
    @param direction: the direction (0: normal, 1: inverted)
    @param method: the method (0: speed measured as time between consecutive edges, 1: speed measured as number of edges per time unit)
    """
    
    if slave == None:
        logging.error('set_digital_incremental_encoder_data: no slave available')
        return
    
    direction_offset = 4
    method_offset = 9
    configuration_data = encoder_type | (direction << direction_offset) | (method << method_offset)
    
    slave.sdo_write(0x3010, 1, bytes(ctypes.c_uint32(number_of_pulses_per_turn)))
    slave.sdo_write(0x3010, 2, bytes(ctypes.c_uint16(configuration_data)))

def set_ssi_encoder_data(slave, data_rate, number_of_bits, encoding_type, direction, check_frame, timeout_time, number_of_multi_turn_bits, number_of_single_turn_bits):
    """! Set the SSI encoder data of the slave
    @param slave: the slave to set the SSI encoder data of
    @param data_rate: the data rate in bits/s
    @param number_of_bits: the number of bits
    @param encoding_type: the encoding type (0: gray code, 1: binary code)
    @param direction: the direction (0: normal, 1: inverted)
    @param check_frame: the check frame (0: no check frame, 1: check frame)
    @param timeout_time: the timeout time in ms
    @param number_of_multi_turn_bits: the number of multi-turn bits
    @param number_of_single_turn_bits: the number of single-turn bits
    """
    
    if slave == None:
        logging.error('set_ssi_encoder_data: no slave available')
        return
    
    direction_offset = 4
    check_frame_offset = 8
    configuration_data = encoding_type | (direction << direction_offset) | (check_frame << check_frame_offset)

    multiturn_offset = 8
    position_bits = number_of_multi_turn_bits << multiturn_offset | number_of_single_turn_bits
    
    slave.sdo_write(0x3011, 1, bytes(ctypes.c_uint32(data_rate)))
    slave.sdo_write(0x3011, 2, bytes(ctypes.c_uint8(number_of_bits)))
    slave.sdo_write(0x3011, 3, bytes(ctypes.c_uint8(configuration_data)))
    slave.sdo_write(0x3011, 5, bytes(ctypes.c_uint16(timeout_time//10)))
    slave.sdo_write(0x3011, 5, bytes(ctypes.c_uint8(position_bits)))



############################################
# High-level EPOS4 functions
############################################
def clear_faults(slave):
    """! Clear the faults of the slave
    @param slave: the slave to clear the faults of
    """
    
    if slave == None:
        logging.error('clear_faults: no slave available')
        return
    
    set_controlword_from_bitmasks(slave, 1<<CONTROLWORD_BIT_OFFSET_FAULT_RESET, 0)
    wait_for_statusword_bits(slave, 1<<STATUSWORD_BIT_OFFSET_FAULT, 0)

def reset_node(slave):
    """! Reset the EtherCAT node of the slave
    @param slave: the slave to reset the EtherCAT node of
    """
    
    print('reset_node')
    if slave == None:
        logging.error('reset_node: no slave available')
        return
    
    slave.sdo_write(0x1f51, 1, bytes(ctypes.c_uint8(0x02)))
    print(ctypes.c_int8.from_buffer_copy(slave.sdo_read(index=0x1f51, subindex=0)).value)

def configure_slave(slave, motor_configuration = CSU_motor_config_defaults(), 
                    gear_configuration = CSU_gear_config_defaults(), 
                    digital_incremental_encoder_configuration = CSU_digital_incremental_encoder_config_defaults(), 
                    ssi_encoder_configuration = CSU_ssi_encoder_config_defaults()):
    """! Configure the slave
    @param slave: the slave to configure
    @param motor_configuration: the motor configuration
    @param gear_configuration: the gear configuration
    @param digital_incremental_encoder_configuration: the digital incremental encoder configuration
    @param ssi_encoder_configuration: the SSI encoder configuration
    """

    if slave == None:
        logging.error('configure_slave: no slave available')
        return
    
    set_node_id(slave, 1)
    set_motor_data(slave, motor_configuration.nominal_current_ma, motor_configuration.output_current_limit_ma, 
                   motor_configuration.thermal_time_constant_winding_ms, motor_configuration.torque_constant_uNm_A)
    set_gear_data(slave, gear_configuration.gear_reduction_numberator, 
                  gear_configuration.gear_reduction_denominator, gear_configuration.gear_max_input_speed_rpm, 
                  gear_configuration.orientation)
    set_digital_incremental_encoder_data(slave, digital_incremental_encoder_configuration.number_of_pulses_per_turn, 
                                         digital_incremental_encoder_configuration.encoder_type, 
                                         digital_incremental_encoder_configuration.direction, 
                                         digital_incremental_encoder_configuration.method)
    set_ssi_encoder_data(slave, ssi_encoder_configuration.data_rate, ssi_encoder_configuration.number_of_bits, 
                         ssi_encoder_configuration.encoding_type, ssi_encoder_configuration.direction, 
                         ssi_encoder_configuration.check_frame, ssi_encoder_configuration.timeout_time_ms, 
                         ssi_encoder_configuration.number_of_multi_turn_bits, 
                         ssi_encoder_configuration.number_of_single_turn_bits)
    slave.dc_sync(act=True,sync0_cycle_time=1000000)
    
def home_slave(slave):
    """! Home the slave
    @param slave: the slave to home
    """

    if slave == None:
        logging.error('home_slave: no slave available')
        return
    
    set_mode_of_operation(slave, MODE_OF_OPERATION_HOMING)
    while(get_mode_of_operation(slave) != MODE_OF_OPERATION_HOMING):
        time.sleep(0.1)
    set_quick_stop_deceleration(slave, 100000)
    set_homing_speed(slave, 100)
    set_homing_acceleration(slave, 10000)
    set_home_offset_move_distance(slave, 0)
    
    # Homing phase 1: search for negative limit switch
    set_homing_method(slave, HOMING_METHOD_LIMIT_SWITCH_NEGATIVE)
    set_controlword(slave, CONTROLWORD_COMMAND_SHUTDOWN)
    wait_for_statusword_bits(slave, STATUSWORD_COMMAND_SHUTDOWN_MASK, STATUSWORD_COMMAND_SHUTDOWN_VALUE)
    set_controlword(slave, CONTROLWORD_COMMAND_SWITCH_ON_AND_ENABLE)
    wait_for_statusword_bits(slave, STATUSWORD_COMMAND_SWITCH_ON_AND_ENABLE_MASK, STATUSWORD_COMMAND_SWITCH_ON_AND_ENABLE_VALUE)
    set_controlword(slave, CONTROLWORD_COMMAND_START_HOMING)

    # Temporary method of homing used until future method is implemnted
    ssi_maximum_position_jump = 2000
    last_position = get_ssi_position_raw_value_complete(slave)
    current_position = last_position
    while True:
        delta = abs(current_position - last_position)
        if delta > ssi_maximum_position_jump:
            set_controlword(slave, CONTROLWORD_COMMAND_QUICK_STOP)
            wait_for_statusword_bits(slave, 1<<STATUSWORD_BIT_OFFSET_QUICK_STOP)
            logging.info('home_slave: homing passed phase 1 due to SSI encoder position jump of {}'.format(delta))
            break
        last_position = current_position
        current_position = get_ssi_position_raw_value_complete(slave)
        logging.info('home_slave: homing phase 1 current position {}'.format(current_position))
        time.sleep(0.01)

    # Correct method of homing to be implemented physically in the future listed below
    # while get_statusword(slave) & bit_offset_to_mask(STATUSWORD_BIT_OFFSET_HOMING_ATTAINED) == 0:
    #     time.sleep(0.1)

    # Homing phase 2: get absolute position from SSI encoder
    time.sleep(2)
    set_home_position(slave, get_ssi_position_raw_value_complete(slave))
    set_homing_method(slave, HOMING_METHOD_ACTUAL_POSITION)
    set_controlword(slave, CONTROLWORD_COMMAND_SHUTDOWN)
    wait_for_statusword_bits(slave, STATUSWORD_COMMAND_SHUTDOWN_MASK, STATUSWORD_COMMAND_SHUTDOWN_VALUE)
    set_controlword(slave, CONTROLWORD_COMMAND_SWITCH_ON_AND_ENABLE)
    wait_for_statusword_bits(slave, STATUSWORD_COMMAND_SWITCH_ON_AND_ENABLE_MASK, STATUSWORD_COMMAND_SWITCH_ON_AND_ENABLE_VALUE)
    set_controlword(slave, CONTROLWORD_COMMAND_START_HOMING)
    if get_statusword(slave) & bit_offset_to_mask(STATUSWORD_BIT_OFFSET_HOMING_ERROR) == 1:
        logging.error('home_slave: homing failed')
        return
    
    logging.info('home_slave: homing successful')
    logging.info('home_slave: absolute position: {}'.format(get_ssi_position_raw_value_complete(slave)))

def home_slave_using_current_ssi_position(slave):
    """! Home the slave using the current SSI position
    @param slave: the slave to home
    """

    if slave == None:
        logging.error('home_slave_using_current_ssi_position: no slave available')
        return
    
    set_mode_of_operation(slave, MODE_OF_OPERATION_HOMING)
    while(get_mode_of_operation(slave) != MODE_OF_OPERATION_HOMING):
        time.sleep(0.1)

    set_home_position(slave, get_ssi_position_raw_value_complete(slave))
    set_homing_method(slave, HOMING_METHOD_ACTUAL_POSITION)
    set_controlword(slave, CONTROLWORD_COMMAND_SHUTDOWN)
    wait_for_statusword_bits(slave, STATUSWORD_COMMAND_SHUTDOWN_MASK, STATUSWORD_COMMAND_SHUTDOWN_VALUE)
    set_controlword(slave, CONTROLWORD_COMMAND_SWITCH_ON_AND_ENABLE)
    wait_for_statusword_bits(slave, STATUSWORD_COMMAND_SWITCH_ON_AND_ENABLE_MASK, STATUSWORD_COMMAND_SWITCH_ON_AND_ENABLE_VALUE)
    set_controlword(slave, CONTROLWORD_COMMAND_START_HOMING)
    while(get_statusword(slave) & bit_offset_to_mask(STATUSWORD_BIT_OFFSET_HOMING_ATTAINED) == 0):
        time.sleep(0.1)
        if get_statusword(slave) & bit_offset_to_mask(STATUSWORD_BIT_OFFSET_HOMING_ERROR) == 1:
            logging.error('home_slave: homing failed')
            return
    
    logging.info('home_slave: homing successful with absolute position: {}'.format(get_ssi_position_raw_value_complete(slave)))

def set_position_using_csp_mode(slave, position, ignore_errors=False):
    """! Set the position of the slave using CSP mode
    @param slave: the slave to set the position of
    @param position: the position to set the slave to
    """

    if slave == None:
        logging.error('set_position_using_csp_mode: no slave available')
        return
    
    if get_error_register(slave) != 0:
        logging.error('set_position_using_csp_mode: slave reports error code {} and error register'.format(get_error_code(slave), get_error_register(slave)))
        if ignore_errors == False:
            return
        slave.recover()
        time.sleep(2)
        clear_faults(slave)
        time.sleep(2)
        
    set_mode_of_operation(slave, MODE_OF_OPERATION_CYCLIC_SYNCHRONOUS_POSITION)
    set_controlword(slave, CONTROLWORD_COMMAND_SHUTDOWN)
    wait_for_statusword_bits(slave, STATUSWORD_COMMAND_SHUTDOWN_MASK, STATUSWORD_COMMAND_SHUTDOWN_VALUE)
    set_controlword(slave, CONTROLWORD_COMMAND_SWITCH_ON_AND_ENABLE)
    wait_for_statusword_bits(slave, STATUSWORD_COMMAND_SWITCH_ON_AND_ENABLE_MASK, STATUSWORD_COMMAND_SWITCH_ON_AND_ENABLE_VALUE)
    set_target_position(slave, position)

def fast_position_move_already_in_csp_mode(slave, position, ignore_errors=False):
    """! Move the slave to a position using CSP mode without sending an enter CSP mode command and using the significantly faster PDO system
    @param slave: the slave to move
    @param position: the position to move the slave to
    """

    """PDO Mappings for RxPDO:
    1st: Controlword
    2nd: Target Position
    3rd: Position Offset
    4th: Torque Offset
    5th: Modes of Operation
    6th: Digital Outputs
    """
    tx_controlword = ctypes.c_uint16(CONTROLWORD_COMMAND_SWITCH_ON_AND_ENABLE)
    tx_target_position = ctypes.c_int32(position)
    tx_position_offset = ctypes.c_int32(0)
    tx_torque_offset = ctypes.c_int16(0)
    tx_modes_of_operation = ctypes.c_int8(MODE_OF_OPERATION_CYCLIC_SYNCHRONOUS_POSITION)
    tx_digital_outputs = ctypes.c_uint8(0)

    output_buffer = struct.pack('HiihbB', tx_controlword.value, tx_target_position.value, tx_position_offset.value, tx_torque_offset.value, tx_modes_of_operation.value, tx_digital_outputs.value)

    if slave == None:
        logging.error('fast_position_move_already_in_csp_mode: no slave available')
        return
    
    if get_error_register(slave) != 0:
        logging.error('fast_position_move_already_in_csp_mode: slave reports error code {}'.format(get_error_register(slave)))
        if ignore_errors == False:
            return
        set_position_using_csp_mode(slave, position, ignore_errors)
        time.sleep(1)
        return
    
    # set_controlword(slave, CONTROLWORD_COMMAND_SHUTDOWN)
    # wait_for_statusword_bits(slave, CONTROLWORD_COMMAND_SHUTDOWN)
    # set_controlword(slave, CONTROLWORD_COMMAND_SWITCH_ON_AND_ENABLE)
    # wait_for_statusword_bits(slave, CONTROLWORD_COMMAND_SWITCH_ON_AND_ENABLE)
    # set_target_position(slave, position)

    slave.output = output_buffer
    # time.sleep(0.05)

    # """PDO Mappings for TxPDO:
    # 1st: Statusword
    # 2nd: Position Actual Value
    # 3rd: Velocity Actual Value
    # 4th: Torque Actual Value
    # 5th: Modes of Operation Display
    # 6th: Digital Inputs
    # """
    # result = struct.unpack('HiihbB', slave.input)
    # return result

def set_position_using_ppm_mode(slave, position, velocity, acceleration, ignore_error=False):
    """! Set the position of the slave using PPM mode
    @param slave: the slave to set the position of
    @param position: the position to set the slave to
    @param velocity: the velocity to set the slave to
    @param acceleration: the acceleration to set the slave to
    """

    if slave == None:
        logging.error('set_position_using_ppm_mode: no slave available')
        return
    
    if get_error_register(slave) != 0:
        logging.error('set_position_using_ppm_mode: slave reports error code {} and error register'.format(get_error_code(slave), get_error_register(slave)))
        if ignore_error == False:
            return
        slave.recover()
        time.sleep(2)
        clear_faults(slave)
        time.sleep(2)
     
    set_mode_of_operation(slave, MODE_OF_OPERATION_PROFILE_POSITION)
    while(get_mode_of_operation(slave) != MODE_OF_OPERATION_PROFILE_POSITION):
        time.sleep(0.1)
    set_max_profile_velocity(slave, velocity*2)
    set_profile_velocity(slave, velocity)
    set_profile_acceleration(slave, acceleration)
    set_profile_deceleration(slave, acceleration)
    set_controlword(slave, CONTROLWORD_COMMAND_SHUTDOWN)
    wait_for_statusword_bits(slave, STATUSWORD_COMMAND_SHUTDOWN_MASK, STATUSWORD_COMMAND_SHUTDOWN_VALUE)
    set_controlword(slave, CONTROLWORD_COMMAND_SWITCH_ON_AND_ENABLE)
    wait_for_statusword_bits(slave, STATUSWORD_COMMAND_SWITCH_ON_AND_ENABLE_MASK, STATUSWORD_COMMAND_SWITCH_ON_AND_ENABLE_VALUE)
    set_target_position(slave, position)
    set_controlword(slave, CONTROLWORD_COMMAND_ABSOLUTE_START_IMMEDIATELY)

def set_position_using_ppm_mode_fast(slave, position, velocity = None, acceleration = None, ignore_error=False):
    """! Set the position of the slave using PPM mode
    @param slave: the slave to set the position of
    @param position: the position to set the slave to
    @param velocity: (NOT USED, COMPATIBILITY ONLY) the velocity to set the slave to
    @param acceleration: (NOT USED, COMPATIBILITY ONLY) the acceleration to set the slave to
    """

    if slave == None:
        logging.error('set_position_using_ppm_mode: no slave available')
        return
    
    if get_error_register(slave) != 0:
        logging.error('set_position_using_ppm_mode: slave reports error code {} and error register'.format(get_error_code(slave), get_error_register(slave)))
        if ignore_error == False:
            return
        slave.recover()
        time.sleep(2)
        clear_faults(slave)
        time.sleep(2)

    set_controlword(slave, CONTROLWORD_COMMAND_SHUTDOWN)
    wait_for_statusword_bits(slave, STATUSWORD_COMMAND_SHUTDOWN_MASK, STATUSWORD_COMMAND_SHUTDOWN_VALUE)
    set_controlword(slave, CONTROLWORD_COMMAND_SWITCH_ON_AND_ENABLE)
    wait_for_statusword_bits(slave, STATUSWORD_COMMAND_SWITCH_ON_AND_ENABLE_MASK, STATUSWORD_COMMAND_SWITCH_ON_AND_ENABLE_VALUE)
    set_target_position(slave, position)
    set_controlword(slave, CONTROLWORD_COMMAND_ABSOLUTE_START_IMMEDIATELY)

def run_on_multiple_slaves(function, slaves, *args):
    for slave in slaves:
        function(slave, *args)

def test_run_on_multiple_slaves(slave, position, velocity, acceleration):
    print(position)
    print(velocity)
    print(acceleration)

"""PDO Based Commands"""

def configure_master_for_pdo_operation(master: pysoem.Master):
    """! Configure the master for PDO operation
    @param master: the master to configure for PDO operation
    """

    if master == None:
        logging.error('configure_master_for_pdo_operation: no master available')
        return

    master.read_state()
    io_map_size = master.config_map()
    logging.info('configure_master_for_pdo_operation: io_map_size: {}'.format(io_map_size))

    if master.state_check(pysoem.SAFEOP_STATE, 500000) != pysoem.SAFEOP_STATE:
        master.read_state()
        for slave in master.slaves:
            if not slave.state == pysoem.SAFEOP_STATE:
                print('{} did not reach SAFEOP state'.format(slave.name))
                print('al status code {} ({})'.format(hex(slave.al_status),
                                                      pysoem.al_status_code_to_string(slave.al_status)))
            raise Exception('not all slaves reached SAFEOP state')

        master.state = pysoem.OP_STATE
        master.write_state()

        master.state_check(pysoem.OP_STATE, 500000)
        if master.state != pysoem.OP_STATE:
            master.read_state()
            for slave in master.slaves:
                if not slave.state == pysoem.OP_STATE:
                    print('{} did not reach OP state'.format(slave.name))
                    print('al status code {} ({})'.format(hex(slave.al_status),
                                                          pysoem.al_status_code_to_string(slave.al_status)))
                raise Exception('not all slaves reached OP state')

def configure_position_using_pdo(master, slaves, positions, velocities, accelerations, ignore_error=False):
    """! configure the position of the slave using PDO. Note: does not send the command yet
    @param slaves: the slaves to set the position of
    @param positions: the position to set each slave to
    @param velocities: the velocity to set each slave to
    @param acceleration: the acceleration to set each slave to
    """

    if slaves == None:
        logging.error('set_position_using_pdo: no slaves available')
        return

    output_data = OutputPDO_PPM()

    for i in range(len(slaves)):
        output_data.controlword = CONTROLWORD_COMMAND_SHUTDOWN
        output_data.target_position = positions[i]
        output_data.profile_velocity = velocities[i]
        output_data.profile_acceleration = accelerations[i]
        output_data.profile_deceleration = accelerations[i]
        output_data.modes_of_operation = MODE_OF_OPERATION_PROFILE_POSITION
        output_data.digital_outputs = 0
        slaves[i].output = output_data.pack()
    master.send_processdata()
    master.receive_processdata(1000)

    for i in range(len(slaves)):
        output_data.controlword = CONTROLWORD_COMMAND_SWITCH_ON_AND_ENABLE
        output_data.target_position = positions[i]
        output_data.profile_velocity = velocities[i]
        output_data.profile_acceleration = accelerations[i]
        output_data.profile_deceleration = accelerations[i]
        output_data.modes_of_operation = MODE_OF_OPERATION_PROFILE_POSITION
        output_data.digital_outputs = 0
        slaves[i].output = output_data.pack()

    master.send_processdata()    
    master.receive_processdata(1000)

    for i in range(len(slaves)):
        output_data.controlword = CONTROLWORD_COMMAND_ABSOLUTE_START_IMMEDIATELY
        output_data.target_position = positions[i]
        output_data.profile_velocity = velocities[i]
        output_data.profile_acceleration = accelerations[i]
        output_data.profile_deceleration = accelerations[i]
        output_data.modes_of_operation = MODE_OF_OPERATION_PROFILE_POSITION
        output_data.digital_outputs = 0
        slaves[i].output = output_data.pack()

    master.send_processdata()
    master.receive_processdata(1000)



