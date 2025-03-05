from collections import OrderedDict

import pysoem
import struct
from enum import Enum
from logging import getLogger
from .helpers import STATUSWORD_STATE_BITMASK

class EthercatBus:

    def __init__(self, ifname: str):
        self.ifname = ifname
        self.pysoem_master = pysoem.Master()

    ### Network interface methods ###
    #TODO replace these with decorators that automate this, Bus user shall not need to worry about interface state.
    def openNetworkInterface(self):
        """Opens the network interface with the given interface name."""
        self.pysoem_master.open(self.ifname)  # pysoem doesn't return anything, so we can't check if it was successful

    def closeNetworkInterface(self):
        """Closes the network interface."""
        self.pysoem_master.close()

    ### SDO methods ###
    def SDORead(self, slaveInstance, address: tuple):
        """Reads a Service Data Object (SDO) from a slave."""
        slave = self.pysoem_master.slaves[slaveInstance.node]

        if isinstance(address, Enum):
            address = address.value.value

        index, subIndex, packFormat, *_ = address

        response = struct.unpack('<' + packFormat, slave.sdo_read(index, subIndex))

        if len(response) == 1:
            return response[0]

        return response

    def SDOWrite(self, slaveInstance, address: tuple, data, completeAccess=False):
        """Writes a Service Data Object (SDO) to a slave."""
        slave = self.pysoem_master.slaves[slaveInstance.node]

        if isinstance(address, Enum):
            address = address.value.value

        index, subIndex, packFormat, *_ = address
        slave.sdo_write(index, subIndex, struct.pack('<' + packFormat, data), ca=completeAccess)

    ### Slave configuration methods ###
    def initialize_slaves(self):
        """Creates slave objects for each slave and assigns them to self.slaves. Returns the number of slaves."""
        numSlaves = self.pysoem_master.config_init()
        self.slaves = self.pysoem_master.slaves
        getLogger(__name__).info(self.slave_info(as_string=True))
        return numSlaves

    def slave_info(self, as_string=False):
        """Gathers detailed information for each slave."""

        keys = ('id', 'name', 'manufacturer', 'revision', 'state')
        attribs = ('id', 'name', 'man', 'rev', 'state')
        defaults = ('N/A', '""', 'N/A', 'N/A', 'N/A')
        data = OrderedDict()
        for slave_ndx, slave in enumerate(self.pysoem_master.slaves):
            # Inspect the available attributes using dir()
            data[slave_ndx] = OrderedDict()
            data[slave_ndx]['attributes'] = dir(slave)

            for key, attrib, default in zip(keys, attribs, defaults):
                try:
                    data[slave_ndx][key] = getattr(slave, attrib, default)  # Revision number
                except Exception as e:
                    data[slave_ndx][key] = f"<Error {e} for '{x}' attribute>"

        if as_string:
            fmt = ("Available attributes: {attributes}\n"
                   "ID: {id} - Name: {name}, Manufacturer ID: {manufacturer}, Revision: {revision}, State: {state}")
            string = ("Slave Information:"+
             '\n----\n'.join( [fmt.format(**rec) for rec in data.values()])+
             '\nTotal slaves: {len(self.pysoem_master.slaves)}')

        return string if as_string else data

    def configureSlaves(self):
        """Configures the slaves"""
        self.pysoem_master.config_map()

    #TODO Is this EPOS4 Specific?
    def setWatchDog(self, slaveInstance, timeout: float):
        """Sets the watchdog timeout for the slave.
        Inputs:
            slave: high level master.py.slave instance
            timeout: float
                The timeout in milliseconds
        """

        self.pysoem_master.slaves[slaveInstance.node].set_watchdog('pdi', timeout)
        self.pysoem_master.slaves[slaveInstance.node].set_watchdog('processdata', timeout)

        return 1

        ### Network state methods ###

    # 1. Apply to all slaves
    def assertNetworkWideState(self, state: int) -> bool:

        if self.pysoem_master.state_check(state) == state:
            return True

        return False

    def getNetworkWideState(self):
        self.pysoem_master.read_state()  # Cursed abstraction by pysoem, slaves can't refresh their own state :(
        states = []
        for i, slave in enumerate(self.pysoem_master.slaves):
            states += [slave.state]

        return states

    def setNetworkWideState(self, state: Enum| int):

        if isinstance(state, Enum):
            state = state.value

        self.pysoem_master.state = state
        self.pysoem_master.write_state()

    # 2. Apply to individual slaves
    def assertNetworkState(self, slaveInstance, state: int) -> bool:
        self.pysoem_master.read_state()
        if self.pysoem_master.slaves[slaveInstance.node].state == state:
            return True

        return False

    def getNetworkState(self, slaveInstance):
        self.pysoem_master.read_state()  # Cursed, the slaves can't refresh their own state

        return self.pysoem_master.slaves[slaveInstance.node].state

    def setNetworkState(self, slaveInstance, state: int):
        self.pysoem_master.slaves[slaveInstance.node].state = state
        self.pysoem_master.slaves[slaveInstance.node].write_state()

    ### Device state methods ###
    def assertDeviceState(self, slaveInstance, state: Enum | int) -> bool:

        if isinstance(state, Enum):  # Handle the pythonic class and int type
            state = state.value

        statusword = self.SDORead(slaveInstance, slaveInstance.objectDictionary.STATUSWORD)
        maskedWord = statusword & STATUSWORD_STATE_BITMASK
        maskedWord = maskedWord & state
        return maskedWord == state

    def getDeviceState(self, slaveInstance):
        """Returns the statusword of the slave."""
        return self.SDORead(slaveInstance, slaveInstance.objectDictionary.STATUSWORD)

    def setDeviceState(self, slaveInstance, state: Enum | int):
        self.SDOWrite(slaveInstance, slaveInstance.objectDictionary.CONTROLWORD, state)

    ### PDO methods ###
    def sendProcessData(self):
        self.pysoem_master.send_processdata()

    def receiveProcessData(self):
        self.pysoem_master.receive_processdata(timeout=2000)

    def updateSoftSlavePDOData(self, slaveInstance):
        """Puts the PDO data from the lower level pysoem.slave into the
        higher level slave."""
        slaveInstance.PDOInput = self.pysoem_master.slaves[slaveInstance.node].input

    def addPDOMessage(self, slaveInstance, packFormat, data):
        """Adds a PDO message to the slave's PDO buffer."""
        self.pysoem_master.slaves[slaveInstance.node].output = struct.pack('<' + packFormat, *data)

    def __del__(self):
        self.pysoem_master.close()
