import threading
import time
import struct
from typing import Callable
import logging
from logging import getLogger
import functools
import threading

from early_lab_scripts.helpers.EPOS4 import CONTROLWORD_BIT_OFFSET_ABSOLUTE_RELATIVE
from .helpers import *
from .bus import EthercatBus

#TODO Big picture: why is there both slave.set_device_state() and set_device_states_pdo (which uses PDO messages)


class EPOS4Bus:

    @staticmethod
    def pdo_mode_only(func):
        def wrapper(self, *args, **kwargs):
            if not self.pdo_mode_is_active:
                raise RuntimeError("PDO mode must be enabled before calling this method.")
            return func(self, *args, **kwargs)
        return wrapper

    def __init__(self, interface: str) -> None:
        """
        Inputs:
            configFuncs: list of functions or dictionary of functions
                The functions will be run in the 'Pre-Operational' NMT state and the 'Switch on disabled'
                device state.
        """
        self._bus = EthercatBus(interface)
        self.slaves: list["EPOS4Motor"] = []
        self.PDOCycleTime = 0.005  # 10ms, official sync manager 2 cycle time is 2ms but I've run into issues
        self._pdo_shutdown = threading.Event()
        self._pdo_thread = None
        self._pdo_lock = threading.Lock()

    def open(self):
        self._bus.open()

    def close(self):
        self._bus.close()

    def initialize_slaves(self, id_type_map=dict[int, "EPOS4Motor"]):
        """Creates slave objects in the HAL and in this instance. Sends some basic information to the 
        actual hardware to do this.

        pass a dictionary of bus id types if passed no default will be used. if None EPOS4Motor will be used
        """
        self._bus.initialize_slaves()
        self.slaves = []
        for i, instance in enumerate(self._bus.pysoem_master.slaves):
            #TODO if the type can wrap or extend the pysoem cdefslave then we might be able to
            # patch into the underlying library and forgo having two things that are so tightly coupled
            device = id_type_map[i](self, i, instance.name)
            self.slaves.append(device)
            try:
                instance.config_func = device.config_func  # config func takes a node number
                # instance.setup_func = device.setup_func  # just gets the device
            except AttributeError:
                getLogger(__name__).warning(f'Device type {type(device)} does not provide a setup/config function.')

    def configure_slaves(self):
        """Configure slaves with specific constants, mode and PDO mappings."""
        if not self.assert_network_state(NetworkManagementStates.PRE_OP):
            self.set_network_state(NetworkManagementStates.PRE_OP)

        if not self.assert_device_states(StatuswordStates.SWITCH_ON_DISABLED):
            self.set_device_states(StatuswordStates.SWITCH_ON_DISABLED)

        self._bus.configureSlaves()

        if not self.assert_network_state(NetworkManagementStates.SAFE_OP):
            raise RuntimeError("Failed to transition to Safe-OP state after configuring slaves.")
        
        for slave in self.slaves:
            slave._initialize_pdo_vars()
            slave._create_pdo_message([0] * len(slave.currentRxPDOMap))
    
    def get_slaves_info(self, as_string=False)-> str | list[dict]:
        """Return a list of dictionaries containing information about each slave."""
        slave_info = []
        
        for slave in self.slaves:
            slave_data = slave.get_info()
            slave_info.append(slave_data)

        record = ("  Node: {node}\n" +
                  "  NetState: {networkState}\n" +
                  "  DevState: {state}\n" +
                  "  Object Dictionary: {objectDictionary}\n" +
                  "  Current Rx PDO Map: {currentRxPDOMap}\n" +
                  "  Current Tx PDO Map: {currentTxPDOMap}\n" +
                  ("-" * 40))

        if as_string:
            return "Slave Information:\n" + '\n'.join([record.format_map(slave_data) for slave_data in slave_info])
        else:
            return slave_info

    def slave_alarms(self):
        self._bus.pysoem_master.read_state()
        return [s.al_status for s in  self._bus.pysoem_master.slaves]

    ### State Methods  SDO ###
    def assert_network_state(self, state: Enum) -> bool:
        return self._bus.assertNetworkWideState(state.value)
    
    def get_network_state(self) -> int:
        return self._bus.getNetworkWideState()
    
    def set_network_state(self, state: Enum):
        self._bus.setNetworkWideState(state.value)
    
    def assert_device_states(self, state: StatuswordStates, id=None) -> bool:
        slaves = [self.slaves[id]] if id is not None else self.slaves
        for slave in slaves:
            if not slave.assert_device_state(state):
                return False
        return True

    def get_device_states(self):
        return [slave.get_device_state() for slave in self.slaves]

    def set_device_states(self, state: Enum, id=None):
        slaves = [self.slaves[id]] if id is not None else self.slaves
        for slave in slaves:
            slave.set_device_state(state)

    ### PDO Methods ###
    def enable_pdo(self):
        if self._pdo_thread is not None:
            raise RuntimeError('PDO thread must be terminated and joined')

        getLogger(__name__).debug("Enabling PDO")

        def pdo_sender():
            self._pdo_shutdown.clear()
            while not self._pdo_shutdown.is_set():
                #start = time.perf_counter_ns()
                self._send_pdo(sleep=False)
                # time.sleep(max(self.PDOCycleTime - (time.perf_counter_ns() - start) * 1e-9, 0))
                #start = time.perf_counter_ns()
                self._receive_pdo(sleep=False)
                # time.sleep(max(self.PDOCycleTime - (time.perf_counter_ns() - start) * 1e-9, 0))
                time.sleep(self.PDOCycleTime)

        self._pdo_thread = threading.Thread(name='Ethercat PDO thread', target=pdo_sender, daemon=True)

        self.set_network_state(NetworkManagementStates.OPERATIONAL)
        self._pdo_thread.start()
        time.sleep(self.PDOCycleTime*8)

        if self.assert_network_state(NetworkManagementStates.OPERATIONAL):
            getLogger(__name__).debug("PDO Enabled")
            return True
        else:
            getLogger(__name__).error("Failed to enable PDO")
            self._pdo_shutdown.set()
            getLogger(__name__).debug("Joining on PDO thread")
            self._pdo_thread.join()
            self._pdo_thread = None
            return False

    def disable_pdo(self):
        if self._pdo_thread and self._pdo_thread.is_alive():
            self._pdo_shutdown.set()
            self._pdo_thread.join()
            self._pdo_thread = None
        self.set_network_state(NetworkManagementStates.SAFE_OP)

    @property
    def pdo_mode_is_active(self):
        #TODO this is a check on the thread, not the actual mode robustness would be enhanced if we add code to ensure
        # the thread dies if we leave pdo mode
        return self._pdo_thread and self._pdo_thread.is_alive()

    def _send_pdo(self, sleep=True):
        with self._pdo_lock:
            self._bus.sendProcessData()
            for s in self.slaves:
                s.pdo_message_pending.clear()
        if sleep:
            time.sleep(self.PDOCycleTime)

    def _receive_pdo(self, sleep=True, timeout=2000):
        with self._pdo_lock:
            self._bus.pysoem_master.receive_processdata(timeout=timeout)
            start = time.perf_counter_ns()
            for slave in self.slaves:
                # Put the low level HAL slave byte buffer into slave.PDOInput
                # slave._fetch_pdo_data() # Put the low level HAL slave byte buffer into slave.PDOInput
                # slave.PDOInput = struct.unpack('<' + slave.currentTxPDOPackFormat, slave.PDOInput)
                x = self._bus.pysoem_master.slaves[slave.node].input
                slave.pdo_input = struct.unpack('<' + slave.currentTxPDOPackFormat, x)
            finish = time.perf_counter_ns()

        # Enforce the minimum PDO cycle time after performing all the above operations
        if sleep:
            time.sleep(max(self.PDOCycleTime - (finish - start) * 1e-9,0))

    def _send_receive_pdo(self, sleep=True, timeout=2000):
        with self._pdo_lock:
            self._send_pdo(sleep=sleep)
            self._receive_pdo(sleep=sleep, timeout=timeout)

    @pdo_mode_only
    def wait_for_device_states_pdo(self, state: StatuswordStates, timeout=10):
        """Wait until all slaves are in the desired state. This function will block until the desired state is reached."""
        start = time.time()

        while True:
            if self.assert_device_states_pdo(state):
                break

            if time.time() - start > timeout:
                raise TimeoutError(f"Timeout waiting for slaves to reach state {state}")
            time.sleep(0.1)

    @pdo_mode_only
    def assert_device_states_pdo(self, state: StatuswordStates):
        # Check the states of all slaves
        for slave in self.slaves:
            if not assertStatuswordState(slave._statusword, state):
                getLogger(__name__).debug(f"Slave {slave.node} not in state {state} "
                                          f"(in {slave._statusword & STATUSWORD_STATE_BITMASK})")
                return False
            return True

    @pdo_mode_only
    def set_device_states_pdo(self, state: StatuswordStates, wait=True):
        """Change the device state of all slaves to the desired state. Will only work if all slaves are in the same start state."""
        #TODO check that this overhauled function actually works now
        if self.assert_device_states_pdo(state):
            return

        self.wait_for_device_states_pdo(state)

        # TODO get rid of this 4-line nonsense, should consolidate with proper state classes and object methods
        startingState = getStatuswordState(self.slaves[0]._statusword)
        endState = getStatuswordState(state)
        state_control_sequence = getStateTransitions(startingState, endState)
        getLogger(__name__).debug(f'State transition control word sequence: {state_control_sequence}')

        for slave in self.slaves[1:]:
            assert slave._statusword == self.slaves[0]._statusword, 'Foundational assumption of functional correctness failed.'

        for controlword in state_control_sequence:
            self.wait_for_pdo_transmit()
            for slave in self.slaves:
                slave.rx_data[slave._controlwordPDOIndex] = controlword
                slave._create_pdo_message(slave.rx_data)

        if wait:
            self.wait_for_device_states_pdo(state)

    @pdo_mode_only
    def _block_until_target(self, verbose=False, timeout=10):
        """Block the master until all slaves have reached their target positions."""
        if verbose:
            getLogger(__name__).info("Actual positions:")
            string = '  |  '.join([f'Slave {slave.node}' for slave in self.slaves])
            getLogger(__name__).info(string)

        time.sleep(self.PDOCycleTime * 6)

        start_time = time.time()

        while True:
            moving = False
            strings = []
            for slave in self.slaves:
                # Print the actual position if asked
                strings.append(f'{slave.position()}')
                moving |= slave.moving

            if verbose:
                getLogger(__name__).info('  |  '.join(strings))

            # If no slave is moving anymore, we are done
            if not moving:
                break

            time.sleep(self.PDOCycleTime * 6)
            # Exit loop if timeout is reached
            if time.time() - start_time > timeout:
                raise TimeoutError("Timeout while waiting for slaves to reach target positions.")

    @pdo_mode_only
    def wait_for_pdo_transmit(self, timeout=10):
        """Wait until all slaves don't have PDO message pending."""
        start = time.time()
        while any([s.pdo_message_pending.is_set() for s in self.slaves]):
            time.sleep(0.01)
            if not self.pdo_mode_is_active:
                raise RuntimeError('Left PDO mode while waiting for messages.')
            if time.time() - start > timeout:
                raise TimeoutError("Timeout while waiting for PDO messages to be received.")

    @pdo_mode_only
    def home_motors(self, verbose=True):
        """Start the homing process of all slaves."""

        if not self.assert_device_states_pdo(StatuswordStates.OPERATION_ENABLED):
            self.set_device_states_pdo(StatuswordStates.OPERATION_ENABLED)

        for slave in self.slaves:
            slave.change_operating_mode(OperatingModes.HOMING_MODE)

        self.wait_for_pdo_transmit()

        for slave in self.slaves:
            data = slave.rx_data
            data[slave._controlwordPDOIndex] = ControlWord.START_HOMING.value
            slave._create_pdo_message(data)
        
        self.wait_for_pdo_transmit()

        for slave in self.slaves:
            data = slave.rx_data
            data[slave._controlwordPDOIndex] = ControlWord.START.value
            slave._create_pdo_message(data)

        self.wait_for_pdo_transmit()

        self._block_until_target(verbose=verbose)

    @pdo_mode_only
    def move_to(self, positions: int | dict[int], acceleration=10000, deceleration=10000, speed=1000, blocking=True,
                verbose=False):
        """Send slaves to the given positions based on their node IDs."""
        
        # Ensure the network is in operational mode
        if not self.assert_device_states_pdo(StatuswordStates.OPERATION_ENABLED):
            self.set_device_states_pdo(StatuswordStates.OPERATION_ENABLED)

        for slave in self.slaves:
            slave.change_operating_mode(OperatingModes.PROFILE_POSITION_MODE)

        self.wait_for_pdo_transmit()

        # If no slave_ids are provided, assume all slaves get the same position
        if isinstance(positions, int):
            positions = {i:positions for i in range(len(self.slaves))}

        for id, position in positions.items():
            slave = self.slaves[id]

            # Print information about the slave and its target position
            getLogger(__name__).info(f"Moving Slave {slave.node} to position {position}")

            # Prepare the data for the slave
            data = slave.rx_data
            data[slave._controlwordPDOIndex] = ControlWord.START.value
            data[1] = position  # Target position
            data[2] = acceleration  # Profile acceleration
            data[3] = deceleration  # Profile deceleration
            data[4] = speed  # Profile velocity
            slave._create_pdo_message(data)  # Create PDO message for this slave

        self.wait_for_pdo_transmit()

        # Remove the start PPM motion control word from all slave buffers (necessary to avoid faults)
        for id in positions.keys():
            slave = self.slaves[id]

            # Print information about the slave and its target position
            getLogger(__name__).debug(f"Remove the start PPM motion control word from {slave.node} too avoid faults")

            data = slave.rx_data
            data[slave._controlwordPDOIndex] = ControlWord.STOP.value
            slave._create_pdo_message(data)

        self.wait_for_pdo_transmit()

        # If blocking, wait until all slaves have reached their target positions
        if blocking:
            self._block_until_target(verbose=verbose)

    def check_errors(self):
        for slave in self.slaves:
            slave.check_errors()

    def __del__(self):
        self.close()


class EPOS4Motor:

    def __init__(self, master: EPOS4Bus, node: int, objectDictionary: str):
        """Initializes a slave object with the given Finite State Automation and object dictionary.
        
        Args: 
            node (int): The node number of the slave
            objectDictionary (str): The object dictionary that the slave uses
        
        Notes:
            - This object shouldn't be used directly, it should be created by the master class.
            - The object dictionary should be a string that corresponds to the object dictionary of the slave."""

        self.pdo_input = None
        self.HAL : EthercatBus = master._bus
        self.node = node
        self.currentRxPDOMap = None
        self.currentTxPDOMap = None
        self.rx_data = None
        self.object_dict = None
        self.pdo_message_pending = threading.Event()

        match objectDictionary:
            case "EPOS4":
                self.object_dict = EPOS4ObjDict
        
        ### Default Operation Mode PDO Maps ###
        self.PPMRx = [self.object_dict.CONTROLWORD, self.object_dict.TARGET_POSITION,
                      self.object_dict.PROFILE_ACCELERATION, self.object_dict.PROFILE_DECELERATION,
                      self.object_dict.PROFILE_VELOCITY, self.object_dict.MODES_OF_OPERATION,
                      self.object_dict.PHYSICAL_OUTPUTS]
        self.PPMTx = [self.object_dict.STATUSWORD, self.object_dict.POSITION_ACTUAL_VALUE, self.object_dict.VELOCITY_ACTUAL_VALUE,
                      self.object_dict.FOLLOWING_ERROR_ACTUAL_VALUE, self.object_dict.MODES_OF_OPERATION_DISPLAY, self.object_dict.DIGITAL_INPUTS]

    def __repr__(self):
        """String representation of the slave."""
        return f"EPOS4Motor(masternode={self.node}, net_state={self._get_network_state()}, dev_state={self.get_device_state()}, objectDictionary={self.object_dict})"

    @property
    def _statusword(self):
        return self.pdo_input[self._statuswordPDOIndex]

    # def _set_controlword(self, value: int):
    #     getLogger(__name__).debug(f"Setting controlword (index {self._controlwordPDOIndex}) to {value} in rxdata (from {self.RxData}).")
    #     self.RxData[self._controlwordPDOIndex] = value
    #
    # def _set_target_position(self, value: int):
    #     self.RxData[self._controlwordPDOIndex] = ControlWord.START.value
    #     self.RxData[1] = value
    #
    # def _set_profile_acceleration(self, value: int):
    #     self.RxData[self._controlwordPDOIndex] = ControlWord.START.value
    #     self.RxData[2] = value

    def identify(self, enable=True):
        """Enable or disable identification"""
        pass

    def check_errors(self):
        print(f"Node {self.node} diagnostics:")
        resp = self._sdo_read(self.object_dict.DIAGNOSIS_HISTORY_DIAGNOSIS_MESSAGE_1.value)
        print(" Diagnosis message 1: ", resp)
        resp = self._sdo_read(self.object_dict.DIAGNOSIS_HISTORY_DIAGNOSIS_MESSAGE_2.value)
        print(" Diagnosis message 2: ", resp)
        resp = self._sdo_read(self.object_dict.DIAGNOSIS_HISTORY_DIAGNOSIS_MESSAGE_3.value)
        print(" Diagnosis message 3: ", resp)
        resp = self._sdo_read(self.object_dict.DIAGNOSIS_HISTORY_DIAGNOSIS_MESSAGE_4.value)
        print(" Diagnosis message 4: ", resp)
        resp = self._sdo_read(self.object_dict.DIAGNOSIS_HISTORY_DIAGNOSIS_MESSAGE_5.value)
        print(" Diagnosis message 5: ", resp)

    # Example method to get slave-specific info
    def get_info(self):
        """Returns key information about this slave."""
        return {
            "node": self.node,
            "networkState": self._get_network_state(),
            "state": self.get_device_state(),
            "objectDictionary": self.object_dict,
            "currentRxPDOMap": self.currentRxPDOMap,
            "currentTxPDOMap": self.currentTxPDOMap,
        }

    @property
    def position(self):
        return self.pdo_input[1]

    @property
    def velocity(self):
        return self.pdo_input[2]

    @property
    def moving(self):
        #TODO this may actually merely be target position attained
        # old code was: (statusword & (1 << 10) != 1 << 10)  # Checking if the target position is reached
        return bool((self.pdo_input[self._statuswordPDOIndex] >> 10) & 0b1)

    @property
    def following_error(self):
        return self.pdo_input[3]

    @property
    def device_mode_string(self):
        try:
            return StatuswordStates(self.pdo_input[self._statuswordPDOIndex] & STATUSWORD_STATE_BITMASK)
        except ValueError:
            return f"Unknown mode: ({self.pdo_input[self._statuswordPDOIndex] & STATUSWORD_STATE_BITMASK})"

    @property
    def operation_mode_string(self):
        return OperatingModes(self.pdo_input[self._modesOfOperationDisplayPDOIndex])

    ### State methods ###
    def _assert_network_state(self, state: Enum) -> bool:
        return self.HAL.assertNetworkState(self, state)
    
    def _get_network_state(self):
        return self.HAL.getNetworkState(self)

    def _set_network_state(self, state: Enum):
        if isinstance(state, Enum) and not isinstance(state, StatuswordStates):
            state = state.value
        return self.HAL.setNetworkState(self, state)

    def assert_device_state(self, state: Enum) -> bool:
        return self.HAL.assertDeviceState(self, state)

    def get_device_state(self):
        return self.HAL.getDeviceState(self)

    def set_device_state(self, state: Enum, mode ="automated"):
        """Set the device state of an individual slave. If the mode is default,
        try to set the state regardless of current state. If the mode is automated,
        automatically find the correct set of transitions and set the state."""

        if mode.lower() == 'default': 
            self.HAL.setDeviceState(self, state)
        elif mode.lower() == 'automated':
            statusword = self.get_device_state()
            device_state = getStatuswordState(statusword)
            desired_state = state

            controlwords = getStateTransitions(device_state, getStatuswordState(desired_state))
            getLogger(__name__).debug(f'State transition control word: {controlwords}')
            for controlword in controlwords:
                self.HAL.setDeviceState(self, controlword)

        #TODO this is "Failing" as it is getting SWITCHED_ON, likely because the get state poll is too fast and it hasn't yet attained
        # OPERATION_ENABLED, it should optionally wait or at least not speciously warn
        time.sleep(0.1)
        statusword = self.get_device_state()
        if not assertStatuswordState(statusword, state):
            getLogger(__name__).warning(f"Failed to set device state, wanted {state}, got {statusword & STATUSWORD_STATE_BITMASK}")

    def _sdo_read(self, address: Enum):
        return self.HAL.SDORead(self, address.value)

    def _sdo_write(self, address: Enum | tuple, value: int, completeAccess=False):
        self.HAL.SDOWrite(self, address, value, completeAccess)

    def _choose_pdo_map(self, syncManager, PDOAddress):
        raise NotImplementedError

    def _create_pdo_message(self, data: list[int]):
        """Create a PDO rx (outgoing) message to this slave with the given data. This will overwrite all data currently
        in RxData"""
        if self.pdo_message_pending.is_set():
            raise RuntimeError("PDO message is already pending.")
        packFormat = ''.join([address[2] for address in self.currentRxPDOMap])
        self.HAL.addPDOMessage(self, packFormat, data)  #TODO make the HAL the EPOS4 BUS, this encapsulation is a mess
        self.pdo_message_pending.set()
        self.rx_data = data

    def _initialize_pdo_vars(self):
        """Finds the most important addresses of the Rx and Tx PDO and assigns them to the variables, as well as making the pack formats for the rx and tx pdos."""
        self._statuswordPDOIndex = None
        self._controlwordPDOIndex = None
        self._setOperationModePDOIndex = None
        self._modesOfOperationDisplayPDOIndex = None

        ### Find the most important addresses in the RxPDO ###
        for i, address in enumerate(self.currentRxPDOMap):
            
            match address:

                case self.object_dict.CONTROLWORD.value.value:
                    self._controlwordPDOIndex = i
                case self.object_dict.MODES_OF_OPERATION.value.value:
                    self._setOperationModePDOIndex = i

        ### Find the most important addresses in the TxPDO ###
        for i, address in enumerate(self.currentTxPDOMap):
                match address:
                    case self.object_dict.STATUSWORD.value.value:
                        self._statuswordPDOIndex = i
                    case self.object_dict.MODES_OF_OPERATION_DISPLAY.value.value:
                        self._modesOfOperationDisplayPDOIndex = i

    @property
    def tx_pdo_pack_format(self):
        return ''.join([x[2] for x in self.currentTxPDOMap])

    @property
    def rx_pdo_pack_format(self):
        return ''.join([x[2] for x in self.currentRxPDOMap])

    def change_operating_mode(self, mode: OperatingModes):
        """Change the RxPDO output to switch the operating mode of the slave on the next master.SendPDO() call."""

        if self.currentRxPDOMap is None:
            raise ValueError("No PDO map was assigned to the slave (at least at a software level).")

        if self.rx_data is None:  # Catch edge case that happens if user wants to change the operating mode first without creating a PDO message first
            raise RuntimeError("Create a PDO message before changing operating mode.")
            # self.RxData = [0] * len(self.currentRxPDOMap)   # This could be bad, I'm trusting that maxon has it setup such that PDOs with all zeros or the lack of data results in no changes on the slave

        rx_ndx = None
        operationModeIndex, operationModeSubIndex, *_ = self.object_dict.MODES_OF_OPERATION.value.value
        for i, address in enumerate(self.currentRxPDOMap):
            if address[0] == operationModeIndex and address[1] == operationModeSubIndex:
                rx_ndx = i
        
        if rx_ndx is None:
            raise RuntimeError("Can't change operating mode with PDO because the current RxPDO map doesn't contain the MODES_OF_OPERATION address.")

        #TODO why are we passing around an attribute?
        self.rx_data[rx_ndx] = mode.value
        self._create_pdo_message(self.rx_data)
        
    #TODO as written these make assumptions about the collective bus (e.g. it is in pdo mode)
    # and don't allow clean waiting on pdo message transmission
    def home(self):
        """Requires PDO, operating mode homing, and device state operation enabled. Change the RxPDO output to tell the slave to begin the homing
        operation on the next master.SendPDO() call."""
        if not self.assert_device_state(StatuswordStates.OPERATION_ENABLED):
            raise RuntimeError("Device must be in OPERATION_ENABLED state to use profile position move.")

        startingState = getStatuswordState(self._statusword)
        endState = getStatuswordState(StatuswordStates.OPERATION_ENABLED)
        stateTransitionControlwords = getStateTransitions(startingState, endState)
        getLogger(__name__).debug(f'State transition control word: {stateTransitionControlwords}')

        for controlword in stateTransitionControlwords:
            self.rx_data[self._controlwordPDOIndex] = controlword
            self._create_pdo_message(self.rx_data)
            while self.pdo_message_pending.is_set():
                time.sleep(.1)  #todo don't have access to pdo cycle time in this object but that is the relevant interval
            time.sleep(1)  # wait for transition # TODO make this work nicely

        self.change_operating_mode(OperatingModes.PROFILE_POSITION_MODE)

        while self.pdo_message_pending.is_set():
            time.sleep(.1)  # todo don't have access to pdo cycle time in this object but that is the relevant interval

        data = self.rx_data
        data[self._controlwordPDOIndex] = ControlWord.START_HOMING.value
        self._create_pdo_message(data)

        while self.pdo_message_pending.is_set():
            time.sleep(.1)  # todo don't have access to pdo cycle time in this object but that is the relevant interval

        data = self.rx_data
        data[self._controlwordPDOIndex] = ControlWord.START.value
        self._create_pdo_message(data)

        while self.pdo_message_pending.is_set():
            time.sleep(.1)  # todo don't have access to pdo cycle time in this object but that is the relevant interval


    def profile_position_move(self, position:int, speed:int):
        if not self.assert_device_state(StatuswordStates.OPERATION_ENABLED):
            raise RuntimeError("Device must be in OPERATION_ENABLED state to use profile position move.")

        startingState = getStatuswordState(self._statusword)
        endState = getStatuswordState(StatuswordStates.OPERATION_ENABLED)
        stateTransitionControlwords = getStateTransitions(startingState, endState)
        getLogger(__name__).debug(f'State transition control word: {stateTransitionControlwords}')

        for controlword in stateTransitionControlwords:
            self.rx_data[self._controlwordPDOIndex] = controlword
            self._create_pdo_message(self.rx_data)
            while self.pdo_message_pending.is_set():
                time.sleep(.1)
            time.sleep(1)  # wait for transition # TODO make this work nicely

        self.change_operating_mode(OperatingModes.PROFILE_POSITION_MODE)
        time.sleep(1)

        data = self.rx_data
        data[self._controlwordPDOIndex] = ControlWord.START.value
        data[1] = position  # Target position
        data[2] = 10000  # Profile acceleration
        data[3] = 10000  # Profile deceleration
        data[4] = speed  # Profile velocity
        self._create_pdo_message(data)  # Create PDO message for this slave

        while self.pdo_message_pending.is_set():
            time.sleep(.1)

        #Change to QUICK_STOP_ACTIVE state
        startingState = getStatuswordState(self._statusword)
        endState = getStatuswordState(StatuswordStates.QUICK_STOP_ACTIVE)
        stateTransitionControlwords = getStateTransitions(startingState, endState)
        getLogger(__name__).debug(f'State transition control word: {stateTransitionControlwords}')

        for controlword in stateTransitionControlwords:
            self.rx_data[self._controlwordPDOIndex] = controlword
            self._create_pdo_message(self.rx_data)
            while self.pdo_message_pending.is_set():
                time.sleep(.1)
            time.sleep(1)  # wait for transition # TODO make this work nicely


    def watchdog(self, timeout_ms: float|None):
        """Set to None to disable the watchdog."""
        if timeout_ms is None:
            #TODO Disable the watchdog, is this even possible?
            raise NotImplementedError('Disabling watchdog not yet implemented')
        else:
            self.HAL.setWatchDog(self, timeout_ms)

# Note this is left as an example of a config function, users of the cooethercat library should write their own!
# def EPOS4MicroTRB_12CC_Config(slaveNum:int, slaves: list[EPOS4Motor]):
#     """ Configures an EPOS4 Micro TRB 12CC device """
#     #TODO see TODO note in EPOS4Bus.configureSlaves()
#     dev = slaves[slaveNum]
#     logging.debug(f"Configuring device {dev} (EPOS4 Micro 24/5)")
#
#
#     # Define the Process Data Objects for PPM (Rx and Tx)
#     PPMRx = [
#         dev.objectDictionary.CONTROLWORD,
#         dev.objectDictionary.TARGET_POSITION,
#         dev.objectDictionary.PROFILE_ACCELERATION,
#         dev.objectDictionary.PROFILE_DECELERATION,
#         dev.objectDictionary.PROFILE_VELOCITY,
#         dev.objectDictionary.MODES_OF_OPERATION,
#         dev.objectDictionary.PHYSICAL_OUTPUTS
#     ]
#     PPMTx = [
#         dev.objectDictionary.STATUSWORD,
#         dev.objectDictionary.POSITION_ACTUAL_VALUE,
#         dev.objectDictionary.VELOCITY_ACTUAL_VALUE,
#         dev.objectDictionary.FOLLOWING_ERROR_ACTUAL_VALUE,
#         dev.objectDictionary.MODES_OF_OPERATION_DISPLAY,
#         dev.objectDictionary.DIGITAL_INPUTS
#     ]
#
#     # Create rx and tx map integers
#     rxAddressInts = makePDOMapping(PPMRx)
#     txAddressInts = makePDOMapping(PPMTx)
#
#     # Assign rx map
#     dev._sdo_write(dev.objectDictionary.NUMBER_OF_MAPPED_OBJECTS_IN_RXPDO_1, 0)
#     for i, addressInt in enumerate(rxAddressInts):
#         dev._sdo_write((0x1600, i + 1, 'I'), addressInt)
#     dev._sdo_write(dev.objectDictionary.NUMBER_OF_MAPPED_OBJECTS_IN_RXPDO_1, len(PPMRx))
#
#     # Assign tx map
#     dev._sdo_write(dev.objectDictionary.NUMBER_OF_MAPPED_OBJECTS_IN_TXPDO_1, 0)
#     for i, addressInt in enumerate(txAddressInts):
#         dev._sdo_write((0x1A00, i + 1, 'I'), addressInt)
#     dev._sdo_write(dev.objectDictionary.NUMBER_OF_MAPPED_OBJECTS_IN_TXPDO_1, len(PPMTx))
#
#     dev.currentRxPDOMap = PPMRx
#     dev.currentTxPDOMap = PPMTx
#
#     # Configure Digital Inputs (example)
#     #TODO these write functions seem to expect a tuple not an Enum
#     dev._sdo_write(dev.objectDictionary.DIGITAL_INPUT_CONFIGURATION_DGIN_1, 255)
#     dev._sdo_write(dev.objectDictionary.DIGITAL_INPUT_CONFIGURATION_DGIN_2, 1)
#     dev._sdo_write(dev.objectDictionary.DIGITAL_INPUT_CONFIGURATION_DGIN_1, 0)
#
#     # Set the home offset move distance
#     dev._sdo_write(dev.objectDictionary.HOME_OFFSET_MOVE_DISTANCE, -622080)
#     logging.debug("Slave configuration complete.")