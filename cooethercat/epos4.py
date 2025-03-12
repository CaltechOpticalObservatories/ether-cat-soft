import time
import threading

from . import EPOS4Bus
from .helpers import *
from .bus import EthercatBus

#TODO Big picture: why is there both slave.set_device_state() and set_device_states_pdo (which uses PDO messages)


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