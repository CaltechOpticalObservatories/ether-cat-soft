import threading
import time
import struct
from typing import Callable
import logging
from logging import getLogger
import functools
import threading

from .helpers import *
from .bus import EthercatBus


class EPOS4Bus:
    def __init__(self, interface: str) -> None:
        """
        Inputs:
            configFuncs: list of functions or dictionary of functions
                The functions will be run in the 'Pre-Operational' NMT state and the 'Switch on disabled'
                device state.
        """
        self._bus = EthercatBus(interface)
        self.slaves: list["EPOS4Motor"] = []
        self.PDOCycleTime = 0.010  # 10ms, official sync manager 2 cycle time is 2ms but I've run into issues
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

        if not self.assertNetworkWideState(NetworkManagementStates.PRE_OP):
            self.setNetworkWideState(NetworkManagementStates.PRE_OP)

        if not self.assertCollectiveDeviceState(StatuswordStates.SWITCH_ON_DISABLED):
            self.setCollectiveDeviceState(StatuswordStates.SWITCH_ON_DISABLED)

        self._bus.configureSlaves()

        if not self.assertNetworkWideState(NetworkManagementStates.SAFE_OP):
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

    ### State Methods  SDO ###
    def assertNetworkWideState(self, state: Enum| int) -> bool:
        state = state.value if isinstance(state, Enum) else state
        return self._bus.assertNetworkWideState(state)
    
    def getNetworkWideState(self):
        return self._bus.getNetworkWideState()
    
    def setNetworkWideState(self, state: Enum| int):
        state = state.value if isinstance(state, Enum) else state
        self._bus.setNetworkWideState(state)
    
    def assertCollectiveDeviceState(self, state: StatuswordStates | int) -> bool:
        state = state.value if isinstance(state, Enum) else state
        for slave in self.slaves:
            if not slave.assert_device_state(state):
                return False
        return True
    
    def getCollectiveDeviceState(self):
        return [slave.get_device_state() for slave in self.slaves]

    def setCollectiveDeviceState(self, state: Enum | int | str):
        for slave in self.slaves:
            slave.set_device_state(state)

    ### PDO Methods ###
    """The PDO methods assume that all slaves are in the same state and that the PDOS all have the same base configuration, or at least that differences
    are handled at other abstraction levels"""
    def enable_pdo(self):
        if self._pdo_thread is not None:
            raise RuntimeError('PDO thread must be terminated and joined')

        getLogger(__name__).debug("Enabling PDO")
        self.setNetworkWideState(NetworkManagementStates.OPERATIONAL)
        def pdo_sender():
            self._pdo_shutdown.clear()
            while not self._pdo_shutdown.is_set():
                start = time.perf_counter_ns()
                self._send_pdo(sleep=False)
                time.sleep(max(self.PDOCycleTime - (time.perf_counter_ns() - start) * 1e-9, 0))
                start = time.perf_counter_ns()
                self._receive_pdo(sleep=False)
                time.sleep(max(self.PDOCycleTime - (time.perf_counter_ns() - start) * 1e-9, 0))

        self._pdo_thread = threading.Thread(name='Ethercat PDO thread', target=pdo_sender, daemon=True)
        self._pdo_thread.start()

        time.sleep(self.PDOCycleTime*8)

        if self.assertNetworkWideState(NetworkManagementStates.OPERATIONAL):
            getLogger(__name__).debug("PDO Enabled")
            return True
        else:
            getLogger(__name__).error("Failed to enable PDO")
            self._pdo_shutdown.set()
            getLogger(__name__).debug("Joining on PDO thread")
            self._pdo_thread.join(timeout=1)
            return False

    def disable_pdo(self):
        if self._pdo_thread and self._pdo_thread.is_alive():
            self._pdo_shutdown.set()
            self._pdo_thread.join()
        self.setNetworkWideState(NetworkManagementStates.SAFE_OP)
    
    def _send_pdo(self, sleep=True):
        with self._pdo_lock:
            self._bus.sendProcessData()
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
                slave.PDOInput = struct.unpack('<' + slave.currentTxPDOPackFormat, x)
            finish = time.perf_counter_ns()

        # Enforce the minimum PDO cycle time after performing all the above operations
        if sleep:
            time.sleep(max(self.PDOCycleTime - (finish - start) * 1e-9,0))

    def _send_receive_pdo(self, sleep=True, timeout=2000):
        with self._pdo_lock:
            self._send_pdo(sleep=sleep)
            self._receive_pdo(sleep=sleep, timeout=timeout)

    def waitForStatePDO(self, desiredState: StatuswordStates):
        """Wait until all slaves are in the desired state. This function will block until the desired state is reached."""
        #TODO This function appears to not work properly
        getLogger(__name__).debug(f"Waiting for state {desiredState}")

        for _ in range(3):
            self._send_receive_pdo()

        while True:
            pending = False
            for slave in self.slaves:
                if not assertStatuswordState(slave._statusword, desiredState):
                    pending = True
                    getLogger(__name__).debug(f"Slave {slave.node} not in state {desiredState} "
                                              f"(in {slave._statusword & STATUSWORD_STATE_BITMASK})")
                    break
            
            if not pending:
                break

            self._send_receive_pdo()

    def assertStatuswordStatePDO(self, state: StatuswordStates):
        #  Clear buffer
        for _ in range(4):
            self._send_receive_pdo()

        # Check the states of all slaves
        for slave in self.slaves:
            if not assertStatuswordState(slave._statusword, state):
                return False
        return True

    def changeDeviceStatesPDO(self, desiredState: StatuswordStates, wait=True):
        """Change the device state of all slaves to the desired state. Will only work if all slaves are in the same start state."""
        #TODO this function does not sucessfully change the device states

        #TODO This need to be done for all slaves (or at least some sort of interlink condition verified)
        if self.assertStatuswordStatePDO(desiredState):
            return

        self._send_receive_pdo()

        statusword = self.slaves[0]._statusword

        if not self.assertStatuswordStatePDO(StatuswordStates.NOT_READY_TO_SWITCH_ON):
            getLogger(__name__).info("Slaves needs time to auto switch states")
            while True:
                self._send_receive_pdo()
                statusword = self.slaves[0]._statusword
                getLogger(__name__).debug(f'{getStatuswordState(statusword)} ({bin(statusword)}), '
                                          f'{assertStatuswordState(statusword, StatuswordStates.NOT_READY_TO_SWITCH_ON)}')
                if not assertStatuswordState(statusword, StatuswordStates.NOT_READY_TO_SWITCH_ON):
                    getLogger(__name__).info("Ready to switch on")
                    break

        startingState = getStatuswordState(statusword)
        endState = getStatuswordState(desiredState)

        stateTransitionControlwords = getStateTransitions(startingState, endState)
        getLogger(__name__).debug(f'State transition control word: {stateTransitionControlwords}')

        for slave in self.slaves:
            assert slave._statusword == statusword, 'Foundational assumption of functional correctness failed.'

        for controlword in stateTransitionControlwords:
            for slave in self.slaves:
                slave._set_controlword(controlword)
                slave._create_pdo_message(slave.RxData)
            self._send_receive_pdo()

        if wait:
            self.waitForStatePDO(desiredState)

    def _block_during_move(self, verbose=False):
        """Block the master until all slaves have reached their target positions."""
        if verbose:
            getLogger(__name__).info("Actual positions:")
            string = '  |  '.join([f'Slave {slave.node}' for slave in self.slaves])
            getLogger(__name__).info(string)

        # Send and receive PDOs 3 times so that the three buffer system is cleared of previous values to avoid false 'Target reached' signals
        for _ in range(3):
            self._send_receive_pdo()

        while True:
            self._send_receive_pdo()

            moving = False
            strings = []
            for slave in self.slaves:
                # Print the actual position if asked
                strings.append(f'{slave.PDOInput[1]}')

                statusword = slave.PDOInput[slave._statuswordPDOIndex]
                moving = moving or (statusword & (1 << 10) != 1 << 10)  # Checking if the target position is reached

            if verbose:
                getLogger(__name__).info('  |  '.join(strings))

            # If no slave is moving anymore, we are done
            if not moving:
                break

    def home_motors(self, verbose=True):
        """Start the homing process of all slaves."""

        if not self.assertStatuswordStatePDO(StatuswordStates.OPERATION_ENABLED):
            self.changeDeviceStatesPDO(StatuswordStates.OPERATION_ENABLED)

        for slave in self.slaves:
            slave._change_operating_mode(OperatingModes.HOMING_MODE)

        self._send_receive_pdo()

        #TODO move these control words into Enums in cooethercat.helpers
        for slave in self.slaves:
            data = slave.RxData
            data[slave._controlwordPDOIndex] = 0b11111 # Control word to start homing
            slave._create_pdo_message(data)
        
        self._send_receive_pdo()

        for slave in self.slaves:
            data = slave.RxData
            data[slave._controlwordPDOIndex] = 0b01111  #TODO: What is this control word doing
            slave._create_pdo_message(data)

        self._block_during_move(verbose=verbose)

    def move_to(self, positions: list[int], acceleration=10000, deceleration=10000, speed=1000, blocking=True, verbose=False, slave_ids=None):
        """Send slaves to the given positions based on their node IDs."""
        
        # Ensure the network is in operational mode
        if not self.assertStatuswordStatePDO(StatuswordStates.OPERATION_ENABLED):
            self.changeDeviceStatesPDO(StatuswordStates.OPERATION_ENABLED)

        for slave in self.slaves:
            slave._change_operating_mode(OperatingModes.PROFILE_POSITION_MODE)

        self._send_receive_pdo()

        # If no slave_ids are provided, assume all slaves get the same position
        if slave_ids is None:
            slave_ids = [slave.node for slave in self.slaves]  # Use slave node IDs

        # Ensure we have enough positions for the number of slaves (or vice versa)
        if len(positions) != len(slave_ids):
            raise ValueError("The number of positions must match the number of slave IDs provided.")

        # Map slave node ID to target position
        for slave, position in zip(self.slaves, positions):

            # Print information about the slave and its target position
            getLogger(__name__).info(f"Moving Slave {slave.node} to position {position}")

            # Prepare the data for the slave
            data = slave.RxData
            data[slave._controlwordPDOIndex] = 0b01111  # Control word to start movement
            data[1] = position  # Target position
            data[2] = acceleration  # Profile acceleration
            data[3] = deceleration  # Profile deceleration
            data[4] = speed  # Profile velocity
            slave._create_pdo_message(data)  # Create PDO message for this slave

        self._send_receive_pdo()

        # Remove the start PPM motion control word from all slave buffers (necessary to avoid faults)
        for slave in self.slaves:
            data = slave.RxData
            data[slave._controlwordPDOIndex] = 0b11111  # Stop motion command
            slave._create_pdo_message(data)

        self._send_receive_pdo()

        # If blocking, wait until all slaves have reached their target positions
        if blocking:
            self._block_during_move(verbose=verbose)

    def __del__(self, checkErrorRegisters=True):

        #TODO this is almost certainly inappropriate for production,
        # assumes bus is still active at instance destruction and has no exception handling.
        if checkErrorRegisters:
            for slave in self.slaves:
                slave.check_errors()
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

        self.HAL : EthercatBus = master._bus
        self.node = node
        self.currentRxPDOMap = None
        self.currentRxPDOPackFormat = None
        self.currentTxPDOMap = None
        self.currentTxPDOPackFormat = None
        self.RxData = None
        self.objectDictionary = None

        match objectDictionary:
            case "EPOS4":
                self.objectDictionary = EPOS4ObjDict
        
        ### Default Operation Mode PDO Maps ###
        self.PPMRx = [self.objectDictionary.CONTROLWORD, self.objectDictionary.TARGET_POSITION,
                    self.objectDictionary.PROFILE_ACCELERATION, self.objectDictionary.PROFILE_DECELERATION,
                    self.objectDictionary.PROFILE_VELOCITY, self.objectDictionary.MODES_OF_OPERATION, 
                    self.objectDictionary.PHYSICAL_OUTPUTS]
        self.PPMTx = [self.objectDictionary.STATUSWORD, self.objectDictionary.POSITION_ACTUAL_VALUE, self.objectDictionary.VELOCITY_ACTUAL_VALUE, 
                      self.objectDictionary.FOLLOWING_ERROR_ACTUAL_VALUE, self.objectDictionary.MODES_OF_OPERATION_DISPLAY, self.objectDictionary.DIGITAL_INPUTS]

    def __repr__(self):
        """String representation of the slave."""
        return f"EPOS4Motor(masternode={self.node}, net_state={self._get_network_state()}, dev_state={self.get_device_state()}, objectDictionary={self.objectDictionary})"

    @property
    def _statusword(self):
        return self.PDOInput[self._statuswordPDOIndex]

    def _set_controlword(self, value: int):
        getLogger(__name__).debug(f"Setting controlword (index {self._controlwordPDOIndex}) to {value} in rxdata (from {self.RxData}).")
        self.RxData[self._controlwordPDOIndex] = value

    # def _set_target_position(self, value: int):
    #     self.RxData[self._controlwordPDOIndex] = 0b01111
    #     self.RxData[1] = value
    #
    # def _set_profile_acceleration(self, value: int):
    #     self.RxData[self._controlwordPDOIndex] = 0b01111
    #     self.RxData[2] = value

    def identify(self, enable=True):
        """Enable or disable identification"""
        pass

    def check_errors(self):
        print("Slave one diagnostics:")
        resp = self._sdo_read(self.objectDictionary.DIAGNOSIS_HISTORY_DIAGNOSIS_MESSAGE_1.value)
        print("Diagnosis message 1: ", resp)
        resp = self._sdo_read(self.objectDictionary.DIAGNOSIS_HISTORY_DIAGNOSIS_MESSAGE_2.value)
        print("Diagnosis message 2: ", resp)
        resp = self._sdo_read(self.objectDictionary.DIAGNOSIS_HISTORY_DIAGNOSIS_MESSAGE_3.value)
        print("Diagnosis message 3: ", resp)
        resp = self._sdo_read(self.objectDictionary.DIAGNOSIS_HISTORY_DIAGNOSIS_MESSAGE_4.value)
        print("Diagnosis message 4: ", resp)
        resp = self._sdo_read(self.objectDictionary.DIAGNOSIS_HISTORY_DIAGNOSIS_MESSAGE_5.value)
        print("Diagnosis message 5: ", resp)

    # Example method to get slave-specific info
    def get_info(self):
        """Returns key information about this slave."""
        return {
            "node": self.node,
            "networkState": self._get_network_state(),
            "state": self.get_device_state(),
            "objectDictionary": self.objectDictionary,
            "currentRxPDOMap": self.currentRxPDOMap,
            "currentTxPDOMap": self.currentTxPDOMap,
        }

    ### State methods ###
    def _assert_network_state(self, state: int) -> bool:
        return self.HAL.assertNetworkState(self, state)
    
    def _get_network_state(self):
        return self.HAL.getNetworkState(self)

    def _set_network_state(self, state: int):
        if isinstance(state, Enum) and not isinstance(state, StatuswordStates):
            state = state.value
        return self.HAL.setNetworkState(self, state)

    # b. Applies to device states
    def assert_device_state(self, state: int) -> bool:
        return self.HAL.assertDeviceState(self, state)

    def get_device_state(self):
        return self.HAL.getDeviceState(self)

    def set_device_state(self, state: int|Enum, mode ="automated"):
        """Set the device state of an individual slave. If the mode is default,
        try to set the state regardless of current state. If the mode is automated,
        automatically find the correct set of transitions and set the state."""

        if mode.lower() == 'default': 
            self.HAL.setDeviceState(self, state)
        elif mode.lower() == 'automated':
            statusword = self.get_device_state()
            deviceState = getStatuswordState(statusword)
            desiredState = state

            controlwords = getStateTransitions(deviceState, getStatuswordState(desiredState))
            getLogger(__name__).debug(f'State transition control word: {controlwords}')
            for controlword in controlwords:
                self.HAL.setDeviceState(self, controlword)

        #TODO this is "Failing" as it is getting SWITCHED_ON, likely because the get state poll is too fast and it hasn't yet attained
        # OPERATION_ENABLED, it should optionally wait or at least not speciously warn
        statusword = self.get_device_state()
        if not assertStatuswordState(statusword, state):
            getLogger(__name__).warning(f"Failed to set device state, wanted {state}, got {statusword & STATUSWORD_STATE_BITMASK}")

    ### Communication methods ###
    ## SDO Methods ##
    def _sdo_read(self, address: Enum):
        return self.HAL.SDORead(self, address.value)

    def _sdo_write(self, address: Enum | tuple, value: int, completeAccess=False):
        self.HAL.SDOWrite(self, address, value, completeAccess)

    ## PDO Methods ##
    def _choose_pdo_map(self, syncManager, PDOAddress):
        raise NotImplementedError

    def _create_pdo_message(self, data: list[int]):
        """Create a PDO rx (outgoing) message to this slave with the given data. This will overwrite all data currently
        in RxData"""
        packFormat = ''.join([address[2] for address in self.currentRxPDOMap])
        self.HAL.addPDOMessage(self, packFormat, data)
        self.RxData = data

    def _initialize_pdo_vars(self):
        """Finds the most important addresses of the Rx and Tx PDO and assigns them to the variables, as well as making the pack formats for the rx and tx pdos."""

        self._statuswordPDOIndex = None
        self._controlwordPDOIndex = None
        self._setOperationModePDOIndex = None
        self._modesOfOperationDisplayPDOIndex = None

        ### Find the most important addresses in the RxPDO ###
        for i, address in enumerate(self.currentRxPDOMap):
            
            match address:

                case self.objectDictionary.CONTROLWORD.value.value:
                    self._controlwordPDOIndex = i
                case self.objectDictionary.MODES_OF_OPERATION.value.value:
                    self._setOperationModePDOIndex = i

        ### Find the most important addresses in the TxPDO ###
        for i, address in enumerate(self.currentTxPDOMap):
                match address:
                    case self.objectDictionary.STATUSWORD.value.value:
                        self._statuswordPDOIndex = i
                    case self.objectDictionary.MODES_OF_OPERATION_DISPLAY.value.value:
                        self._modesOfOperationDisplayPDOIndex = i
        
        ### Create the pack formats for the Rx and Tx PDOs ###
        self.currentRxPDOPackFormat = ''
        self.currentTxPDOPackFormat = ''
        for rxAddress in self.currentRxPDOMap:
            self.currentRxPDOPackFormat += rxAddress[2]
        for txAddress in self.currentTxPDOMap:
            self.currentTxPDOPackFormat += txAddress[2]

    def _change_operating_mode(self, mode: OperatingModes):
        """Change the RxPDO output to switch the operating mode of the slave on the next master.SendPDO() call."""

        if self.currentRxPDOMap is None:
            raise ValueError("No PDO map was assigned to the slave (at least at a software level).")

        if self.RxData is None:  # Catch edge case that happens if user wants to change the operating mode first without creating a PDO message first
            raise RuntimeError("Create a PDO message before changing operating mode.")
            # self.RxData = [0] * len(self.currentRxPDOMap)   # This could be bad, I'm trusting that maxon has it setup such that PDOs with all zeros or the lack of data results in no changes on the slave

        rx_ndx = None
        operationModeIndex, operationModeSubIndex, *_ = self.objectDictionary.MODES_OF_OPERATION.value.value
        for i, address in enumerate(self.currentRxPDOMap):
            if address[0] == operationModeIndex and address[1] == operationModeSubIndex:
                rx_ndx = i
        
        if rx_ndx is None:
            raise RuntimeError("Can't change operating mode with PDO because the current RxPDO map doesn't contain the MODES_OF_OPERATION address.")

        #TODO why are we passing around an attribute?
        self.RxData[rx_ndx] = mode.value
        self._create_pdo_message(self.RxData)
        
    # # Homing Mode (HMM) #
    # def home(self):
    #     """Requires PDO, operating mode homing, and device state operation enabled. Change the RxPDO output to tell the slave to begin the homing
    #     operation on the next master.SendPDO() call."""
    #     pass
    #
    # # Profile Position Mode (PPM) #
    # def profile_position_move(self, position:int, configure_only=False):
    #     """Perform a profile position move using previously configured (or, more likely, default)
    #     motion profile parameters. If configure_only is true the move will not be started"""
    #     self.HAL.goToPositions(position, slave_ids=self._id)

    def watchdog(self, timeout_ms: float|None):
        """Set to None to disable the watchdog."""
        if timeout_ms is None:
            #TODO Disable the watchdog
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