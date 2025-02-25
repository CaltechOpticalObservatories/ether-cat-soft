import sys
import os
import pysoem

from enum import Enum
path = os.path.dirname(__file__)
path = path.replace("HAL/pysoem", "")
sys.path.append(path)

from helpers import *
from HAL.pysoem.pysoemHelpers import *

class pysoemHAL:

    def __init__(self, ifname: str):
        self.ifname = ifname
        self.master = pysoem.Master()

    ### Network interface methods ###
    def openNetworkInterface(self):
        """Opens the network interface with the given interface name."""
        self.master.open(self.ifname)  # pysoem doesn't return anything, so we can't check if it was successful

    def closeNetworkInterface(self):
        """Closes the network interface."""
        self.master.close()

    ### SDO methods ###
    def SDORead(self, slaveInstance, address: tuple):
        """Reads a Service Data Object (SDO) from a slave."""
        slave = self.master.slaves[slaveInstance.node]

        if isinstance(address, Enum):
            address = address.value.value

        index, subIndex, packFormat, *_ = address

        response = struct.unpack('<' + packFormat, slave.sdo_read(index, subIndex))

        if len(response) == 1:
            return response[0]

        return response
    
    def SDOWrite(self, slaveInstance, address: tuple, data, completeAccess=False):
        """Writes a Service Data Object (SDO) to a slave."""
        slave = self.master.slaves[slaveInstance.node]

        if isinstance(address, Enum):
            address = address.value.value

        index, subIndex, packFormat, *_ = address
        slave.sdo_write(index, subIndex, struct.pack('<' + packFormat, data), ca=completeAccess)

    ### Slave configuration methods ###
    def initializeSlaves(self):
        """Creates slave objects for each slave and assigns them to self.slaves. Returns the number of slaves."""
        numSlaves = self.master.config_init()
        self.slaves = self.master.slaves

        # Print out the slave information
        self.print_slave_info()

        return numSlaves

    def print_slave_info(self):
        """Prints out detailed information for each slave."""
        print("Slave Information:")
        for slave in self.master.slaves:
            # Inspect the available attributes using dir()
            print("Available attributes:", dir(slave))  # Prints out all attributes of the slave

            try:
                # Access slave attributes
                slave_id = getattr(slave, 'id', 'N/A')  # Use 'id' as the identifier
                slave_name = getattr(slave, 'name', f"Slave {slave_id}")  # Default to 'Slave <id>' if 'name' doesn't exist
                manufacturer = getattr(slave, 'man', 'N/A')  # Manufacturer ID
                revision = getattr(slave, 'rev', 'N/A')  # Revision number
                state = getattr(slave, 'state', 'N/A')  # Current state of the slave

                # Print out the slave information
                print(f"Slave ID: {slave_id} - Name: {slave_name}, Manufacturer ID: {manufacturer}, Revision: {revision}, State: {state}")
            except Exception as e:
                print(f"Error accessing slave attributes: {e}")

        print("\nTotal slaves:", len(self.master.slaves))
    
    def addConfigurationFunc(self, slaveInstance, configFunc):
        """Adds a configuration function to the slave.
        Inputs:
            slave: high level master.py.slave instance
            configFunc: function
                The function to be run on the slave.
        """
        self.master.slaves[slaveInstance.node].config_func = configFunc
    
    def configureSlaves(self):
        """Configures the slaves"""
        self.master.config_map()
        
    def setWatchDog(self, slaveInstance, timeout: float):
        """Sets the watchdog timeout for the slave.
        Inputs:
            slave: high level master.py.slave instance
            timeout: float
                The timeout in milliseconds
        """
        
        self.master.slaves[slaveInstance.node].set_watchdog('pdi', timeout)
        self.master.slaves[slaveInstance.node].set_watchdog('processdata', timeout)

        return 1 
    
    ### Network state methods ###
    # 1. Apply to all slaves
    def assertNetworkWideState(self, state: int) -> bool:

        if self.master.state_check(state) == state:
            return True
        
        return False
    
    def getNetworkWideState(self):
        self.master.read_state() # Cursed abstraction by pysoem, slaves can't refresh their own state :(
        states = []
        for i, slave in enumerate(self.master.slaves):
            states += [slave.state]
        
        return states
    
    def setNetworkWideState(self, state: int):

        if isinstance(state, Enum):
            state = state.value
        
        self.master.state = state
        self.master.write_state()

    # 2. Apply to individual slaves
    def assertNetworkState(self, slaveInstance, state: int) -> bool:
        self.master.read_state()
        if self.master.slaves[slaveInstance.node].state == state:
            return True
        
        return False
    
    def getNetworkState(self, slaveInstance):
        self.master.read_state() # Cursed, the slaves can't refresh their own state

        return self.master.slaves[slaveInstance.node].state
    
    def setNetworkState(self, slaveInstance, state: int):
        self.master.slaves[slaveInstance.node].state = state
        self.master.slaves[slaveInstance.node].write_state()

    ### Device state methods ###
    def assertDeviceState(self, slaveInstance, state: StatuswordStates | int) -> bool:

        if isinstance(state, Enum): # Handle the pythonic class and int type
            state = state.value

        statusword = self.SDORead(slaveInstance, slaveInstance.objectDictionary.STATUSWORD)
        maskedWord = statusword & STATUSWORD_STATE_BITMASK
        maskedWord = maskedWord & state
        return maskedWord == state
    
    def getDeviceState(self, slaveInstance):
        """Returns the statusword of the slave."""
        return self.SDORead(slaveInstance, slaveInstance.objectDictionary.STATUSWORD)

    
    def setDeviceState(self, slaveInstance, state: stateCommands | int):
        self.SDOWrite(slaveInstance, slaveInstance.objectDictionary.CONTROLWORD, state)

    ### PDO methods ###
    def sendProcessData(self):
        self.master.send_processdata()
    
    def receiveProcessData(self):
        self.master.receive_processdata(timeout=2000)
    
    def updateSoftSlavePDOData(self, slaveInstance):
        """Puts the PDO data from the lower level pysoem.slave into the 
        higher level slave."""
        slaveInstance.PDOInput = self.master.slaves[slaveInstance.node].input
    
    def addPDOMessage(self, slaveInstance, packFormat, data):
        """Adds a PDO message to the slave's PDO buffer."""
        self.master.slaves[slaveInstance.node].output = struct.pack('<' + packFormat, *data)

    def __del__(self):
        self.closeNetworkInterface()