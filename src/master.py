import os
import sys
import time
import struct
from typing import Callable

sys.path.append(os.path.dirname(__file__))
from helpers import *
from HAL.pysoem.pysoemMaster import pysoemHAL

class master:

    def __init__(self, networkInterfaceName: str, slaveConfigFuncs: list[Callable] = None, 
                 HAL=pysoemHAL) -> None:

        self.HAL = HAL(networkInterfaceName)
        self.numSlaves = 0
        self.slaveConfigFuncs = slaveConfigFuncs
        self.slaves: list[slave] = None
        self.PDOCycleTime = 0.010 # 10ms, official sync manager 2 cycle time is 2ms but I've run into issues

    ### Network interface methods ###
    def openNetworkInterface(self):
        self.HAL.openNetworkInterface()    

    def closeNetworkInterface(self):
        self.HAL.closeNetworkInterface()

    ### Slave configuration methods ###
    def initializeSlaves(self):
        """Creates slave objects in the HAL and in this instance. Sends some basic information to the 
        actual hardware to do this."""
        self.numSlaves = self.HAL.initializeSlaves()
        
        self.slaves = []
        for i, slaveInst in enumerate(self.HAL.slaves):
            self.slaves += [slave(self, i, slaveInst.name)]

    def configureSlaves(self):
        """Configure slaves with specific constants, mode and PDO mappings.
        Inputs:
            configFuncs: list of functions or dictionary of functions
                The functions will be run in the 'Pre-Operational' NMT state and the 'Switch on disabled'
                device state. 
            """
        if self.slaveConfigFuncs == None:
            print("WARN: No slave configuration functions provided. Pass them to master on initialization.")
            self.HAL.configureSlaves()
            return

        if not self.assertNetworkWideState(networkManagementStates.PRE_OP):
            self.setNetworkWideState(networkManagementStates.PRE_OP)

        if not self.assertCollectiveDeviceState(StatuswordStates.SWITCH_ON_DISABLED):
            self.setCollectiveDeviceState(StatuswordStates.SWITCH_ON_DISABLED)

        for i, slave in enumerate(self.slaves):
            self.HAL.addConfigurationFunc(slave, self.slaveConfigFuncs[i])

        self.HAL.configureSlaves()
        if not self.assertNetworkWideState(networkManagementStates.SAFE_OP):
            raise RuntimeError("Failed to transition to Safe-OP state after configuring slaves.")
        
        for slave in self.slaves:
            slave.initializePDOVars()
            slave.createPDOMessage([0] * len(slave.currentRxPDOMap))

    ### State Methods  SDO ###
    def assertNetworkWideState(self, state: int) -> bool:

        if isinstance(state, Enum):
            state = state.value
    
        return self.HAL.assertNetworkWideState(state)
    
    def getNetworkWideState(self):
        return self.HAL.getNetworkWideState()
    
    def setNetworkWideState(self, state: int):
        if isinstance(state, Enum):
            state = state.value
        self.HAL.setNetworkWideState(state)
    
    def assertCollectiveDeviceState(self, state: StatuswordStates | int) -> bool:
        
        if isinstance(state, Enum): # Handle the pythonic class and int type
           state = state.value
        
        for slave in self.slaves:
            if not slave.assertDeviceState(state):
                return False
        
        return True
    
    def getCollectiveDeviceState(self):
        states = []
        for slave in self.slaves:
            states += [slave.getDeviceState()]
        return states

    def setCollectiveDeviceState(self, state: int | str):
        for slave in self.slaves:
            slave.setDeviceState(state)

    ### PDO Methods ###
    """The PDO methods assume that all slaves are in the same state and that the PDOS all have the same base configuration, or at least that differences
    are handled at other abstraction levels"""
    def enablePDO(self):
        print("Enabling PDO")
        self.HAL.setNetworkWideState(networkManagementStates.OPERATIONAL)

        # Send PDO data to maintain the operational state
        for i in range(4): # Iterate 4 times to clear the three buffer sync managers
            self.sendPDO()
            self.receivePDO()

        print("PDO Enabled")
        return self.assertNetworkWideState(networkManagementStates.OPERATIONAL)
    
    def disablePDO(self):
        self.setNetworkWideState(networkManagementStates.SAFE_OP)
    
    def sendPDO(self):
        self.HAL.sendProcessData()
        time.sleep(self.PDOCycleTime)

    def receivePDO(self):
        self.HAL.receiveProcessData()
        start = time.perf_counter_ns()
        for slave in self.slaves:
            slave.fetchPDOData() # Put the low level HAL slave byte buffer into slave.PDOInput 

            slave.PDOInput = struct.unpack('<' + slave.currentTxPDOPackFormat, slave.PDOInput)
        
        # Enforce the minimum PDO cycle time after performing all the above operations
        if time.perf_counter_ns() - start > self.PDOCycleTime * 1e9:
            time.sleep(self.PDOCycleTime - (time.perf_counter_ns() - start) * 1e-9)

    def reconfigurePDO(self):
        """Set new config funcs by altering the function handles in master.slaveConfigFuncs then call this function.
        
        Ideally, we wouldn't need to essentially 'reboot' our whole network, but pysoem's reconfig() slave method doesn't
        seem to work and puts the slaves into fault. This function could use some updating in the future."""

        self.closeNetworkInterface()
        self.openNetworkInterface()
        self.initializeSlaves()
        self.configureSlaves()
 
    def waitForStatePDO(self, desiredState: StatuswordStates):
        """Wait until all slaves are in the desired state. This function will block until the desired state is reached."""

        print("Waiting for state")

        self.sendPDO()
        self.receivePDO()
        self.sendPDO()
        self.receivePDO()
        self.sendPDO()
        self.receivePDO()

        waiting = True
        while waiting:
            self.sendPDO()
            self.receivePDO()

            oneSlaveNotInState = False
            for slave in self.slaves:
                if not assertStatuswordState(slave.PDOInput[slave._statuswordPDOIndex], desiredState):
                    oneSlaveNotInState = True
                    print(slave.PDOInput[slave._statuswordPDOIndex])
                    print("One slave not in state")
                    print(slave._statuswordPDOIndex)
                    break
            
            if not oneSlaveNotInState:
                print("All slaves in state")
                waiting = False
                break

    def assertStatuswordStatePDO(self, state: StatuswordStates):

        # Clear buffer
        for i in range(4):
            self.sendPDO()
            self.receivePDO()

        # Check the states of all slaves
        for slave in self.slaves:
            statusword = slave.PDOInput[slave._statuswordPDOIndex]
            if not assertStatuswordState(statusword, state):
                return False
        return True

    def changeDeviceStatesPDO(self, desiredState: StatuswordStates):
        """Change the device state of all slaves to the desired state. Will only work if all slaves are in the same start state."""

        self.receivePDO()
        firstSlave = self.slaves[0]
        statusword = firstSlave.PDOInput[firstSlave._statuswordPDOIndex]

        if assertStatuswordState(statusword, StatuswordStates.NOT_READY_TO_SWITCH_ON):
            print("Slave needs time to auto switch states")
            while True:
                self.sendPDO()
                self.receivePDO()

                statusword = firstSlave.PDOInput[firstSlave._statuswordPDOIndex]
                if assertStatuswordState(statusword, StatuswordStates.SWITCH_ON_DISABLED):
                    print("Reached switch on disabled")
                    break
        
        startingState = getStatuswordState(statusword)
        endState = getStatuswordState(desiredState)

        stateTransitionControlwords = getStateTransitions(startingState, endState)
        print(stateTransitionControlwords)

        for controlword in stateTransitionControlwords:
            for slave in self.slaves:
                slave.RxData[slave._controlwordPDOIndex] = controlword
                print(slave.RxData)
                slave.createPDOMessage(slave.RxData)
                self.sendPDO()
                self.receivePDO()

        self.waitForStatePDO(desiredState)

    def changeOperatingMode(self, operatingMode: int | operatingModes):
        """Change the RxPDO output to switch the operating mode of the slave on the next master.SendPDO() call."""

        for slave in self.slaves:
            slave.changeOperatingMode(operatingMode)

        self.sendPDO()
        self.receivePDO()

    def performHoming(self):
        """Start the homing process of all slaves."""

        if not self.assertStatuswordStatePDO(StatuswordStates.OPERATION_ENABLED):
            self.changeDeviceStatesPDO(StatuswordStates.OPERATION_ENABLED)

        self.changeOperatingMode(operatingModes.HOMING_MODE)

        for slave in self.slaves:
            data = slave.RxData
            data[slave._controlwordPDOIndex] = 0b11111 # Control word to start homing #TODO: Consider implementing controlword class
            slave.createPDOMessage(data)
        
        self.sendPDO()
        self.receivePDO()

        for slave in self.slaves:
            data = slave.RxData
            data[slave._controlwordPDOIndex] = 0b01111 #TODO: Consider implementing controlword class
            slave.createPDOMessage(data)

        # Send and receive PDOs 3 times so that the three buffer system is cleared of previous values to avoid false 'Target reached' signals
        self.sendPDO()
        self.receivePDO()

        self.sendPDO()
        self.receivePDO()

        self.sendPDO()
        self.receivePDO()

        homing = True
        while homing:
            self.sendPDO()
            self.receivePDO()

            oneSlaveStillHoming = False

            for slave in self.slaves:
                statusword = slave.PDOInput[slave._statuswordPDOIndex]
                if statusword & (1 << 10) != 1 << 10:
                    oneSlaveStillHoming = True
                    break
            
            if not oneSlaveStillHoming:
                homing = False
                break

    def goToPositions(self, positions: list[int], profileAcceleration=10000, profileDeceleration=10000, profileVelocity=1000, blocking=True, printActualPosition=False):
        """Send slaves to the given positions."""
        if not self.assertStatuswordStatePDO(StatuswordStates.OPERATION_ENABLED):
            self.changeDeviceStatesPDO(StatuswordStates.OPERATION_ENABLED)

        self.changeOperatingMode(operatingModes.PROFILE_POSITION_MODE)

        # Put the start PPM motion controlword along with the target position/velocity/acceleration/deceleration in all slave buffers
        for slave, position in zip(self.slaves, positions):
            data = slave.RxData
            data[slave._controlwordPDOIndex] = 0b01111
            data[1] = position
            data[2] = profileAcceleration
            data[3] = profileDeceleration
            data[4] = profileVelocity
            slave.createPDOMessage(data)

        self.sendPDO()
        self.receivePDO()

        # Remove the start PPM motion controlword from all slave buffers (neccessary to avoid faults)
        for slave in self.slaves:
            data = slave.RxData
            data[slave._controlwordPDOIndex] = 0b11111
            slave.createPDOMessage(data)
        
        self.sendPDO()
        self.receivePDO()

        if blocking:
            moving = True

            if printActualPosition:
                print("Actual positions:")
                for slave in self.slaves:
                    print(f"Slave {slave.node}", end='  |  ')


            # Extra sends and recieves to clear the buffer of old target reached statuswords
            self.sendPDO()
            self.receivePDO()

            self.sendPDO()
            self.receivePDO()

            while moving:
                self.sendPDO()
                self.receivePDO()


                oneSlaveMoving = False
                for slave in self.slaves:

                    # Print the actual position if asked
                    if printActualPosition:
                        print(slave.PDOInput[1], end='  |  ')

                    statusword = slave.PDOInput[slave._statuswordPDOIndex]
                    if statusword & (1 << 10) != 1 << 10:
                        oneSlaveMoving = True
                        break

                if printActualPosition:
                    print('')
                                
                if not oneSlaveMoving:
                    moving = False
                    break

    def __del__(self, checkErrorRegisters=True):

        if checkErrorRegisters:
            for slave in self.slaves:
                print("Slave one diagnostics:")
                resp = slave.SDORead(slave.objectDictionary.DIAGNOSIS_HISTORY_DIAGNOSIS_MESSAGE_1)
                print("Diagnosis message 1: ", resp)
                resp = slave.SDORead(slave.objectDictionary.DIAGNOSIS_HISTORY_DIAGNOSIS_MESSAGE_2)
                print("Diagnosis message 2: ", resp)
                resp = slave.SDORead(slave.objectDictionary.DIAGNOSIS_HISTORY_DIAGNOSIS_MESSAGE_3)
                print("Diagnosis message 3: ", resp)
                resp = slave.SDORead(slave.objectDictionary.DIAGNOSIS_HISTORY_DIAGNOSIS_MESSAGE_4)
                print("Diagnosis message 4: ", resp)
                resp = slave.SDORead(slave.objectDictionary.DIAGNOSIS_HISTORY_DIAGNOSIS_MESSAGE_5)
                print("Diagnosis message 5: ", resp)

        self.closeNetworkInterface()

class slave:

    def __init__(self, master: master, node: int, objectDictionary: str):
        """Initializes a slave object with the given Finite State Automation and object dictionary.
        
        Args: 
            node (int): The node number of the slave
            objectDictionary (str): The object dictionary that the slave uses
        
        Notes:
            - This object shouldn't be used directly, it should be created by the master class.
            - The object dictionary should be a string that corresponds to the object dictionary of the slave."""

        self.HAL = master.HAL
        self.node = node
        self.state = None
        self.currentRxPDOMap = None
        self.currentRxPDOPackFormat = None
        self.currentTxPDOMap = None
        self.currentTxPDOPackFormat = None
        self.RxData = None

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

    ### State methods ###
    def assertNetworkState(self, state: int) -> bool:
        return self.HAL.assertNetworkState(self, state)
    
    def getNetworkState(self):
        return self.HAL.getNetworkState(self)

    def setNetworkState(self, state: int):

        if isinstance(state, Enum) and not isinstance(state, StatuswordStates):
            state = state.value 

        return self.HAL.setNetworkState(self, state)

    # b. Applies to device states
    def assertDeviceState(self, state: int) -> bool:
        return self.HAL.assertDeviceState(self, state)

    def getDeviceState(self):
        return self.HAL.getDeviceState(self)

    def setDeviceState(self, state: int, mode = "automated"):
        """Set the device state of an individual slave. If the mode is default,
        try to set the state regardless of current state. If the mode is automated,
        automatically find the correct set of transtiions and set the state."""

        if mode.lower() == 'default': 
            self.HAL.setDeviceState(self, state)
            return None
        
        elif mode.lower() == 'automated':
            statusword = self.getDeviceState()
            deviceState = getStatuswordState(statusword)
            desiredState = state

            controlwords = getStateTransitions(deviceState, getStatuswordState(desiredState))
            for controlword in controlwords:
                self.HAL.setDeviceState(self, controlword)
            
            if self.assertDeviceState(desiredState):
                return None
    
    ### Communication methods ###
    ## SDO Methods ##
    def SDORead(self, address: tuple):

        if isinstance(address, Enum):
            address = address.value.value

        return self.HAL.SDORead(self, address)

    def SDOWrite(self, address: tuple, value: int, completeAccess=False):
        self.HAL.SDOWrite(self, address, value, completeAccess)

    ## PDO Methods ##
    def choosePDOMap(self, syncManager, PDOAddress):
        raise NotImplementedError

    def createPDOMessage(self, data: list[int]):
        """"Create a PDO rx (outgoing) message to this slave with the given data. This will overwrite all data currently
        in RxData"""
        packFormat = ''
        for address in self.currentRxPDOMap:
            packFormat += address[2]
        
        self.HAL.addPDOMessage(self, packFormat, data)
        self.RxData = data

    def fetchPDOData(self):
        self.HAL.updateSoftSlavePDOData(self)

    def initializePDOVars(self):
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

    def changeOperatingMode(self, operatingMode: int | operatingModes):
        """Change the RxPDO output to switch the operating mode of the slave on the next master.SendPDO() call."""

        if self.currentRxPDOMap == None:
            raise ValueError("No PDO map was assigned to the slave (or at least at a software level).")

        if isinstance(operatingMode, Enum):
            operatingMode = operatingMode.value

        setOperatingModePDOIndex = None
        operationModeIndex, operationModeSubIndex, *_ = self.objectDictionary.MODES_OF_OPERATION.value.value
        for i, address in enumerate(self.currentRxPDOMap):
            if address[0] == operationModeIndex and address[1] == operationModeSubIndex:
                setOperatingModePDOIndex = i
        
        if setOperatingModePDOIndex == None:
            raise RuntimeError("Can't change operating mode with PDO because the current RxPDO map doesn't contain the MODES_OF_OPERATION address.")
        
        if self.RxData == None: # Catch edge case that happens if user wants to change the operating mode first without creating a PDO message first
            self.RxData = [0] * len(self.currentRxPDOMap)   # This could be bad, I'm trusting that maxon has it setup such that PDOs with all zeros or the lack of data results in no changes on the slave
        
        self.RxData[setOperatingModePDOIndex] = operatingMode
        self.createPDOMessage(self.RxData)
        
    # Homing Mode (HMM) #
    def performHoming(self):
        """Requires PDO, operating mode homing, and device state operation enabled. Change the RxPDO output to tell the slave to begin the homing
        operation on the next master.SendPDO() call."""
        

    # Profile Position Mode (PPM) #



        
    
    ### Configuration methods ###

    def setWatchDog(self, timeout_ms: float):
        self.HAL.setWatchDog(self, timeout_ms)
