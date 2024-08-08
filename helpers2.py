import pysoem
import ctypes
import struct
import typing

from enum import Enum

# Create custom types for explicit referencing in function calls
EPOS4Obj = typing.NewType("EPOS4Obj", tuple)
network_state = typing.NewType("network_state", int)

STATUSWORD_STATE_BITMASK = 0b1101111
class StatuswordStates(Enum):
    NOT_READY_TO_SWITCH_ON = 0
    SWITCH_ON_DISABLED = 0b1000000
    READY_TO_SWITCH_ON = 0b0100001
    SWITCHED_ON = 0b0100011
    OPERATION_ENABLED = 0b0100111
    QUICK_STOP_ACTIVE = 0b0000111
    FAULT_REACTION_ACTIVE = 0b0001111
    FAULT = 0b0001000

class networkManagementStates(Enum):
    NONE = 0x00
    INIT = 0x01
    PRE_OP = 0x02
    STATE_BOOT = 0x03
    SAFE_OP = 0x04
    OPERATIONAL = 0x08
    ERROR = 0x10
    ACK = 0x10

class PDOOutput:

    class MapOne(ctypes.Structure):
        statuswordState = None
        networkManagementState = None
        
        _pack_ = 1
        _fields_ = [
            
            ("CONTROLWORD", ctypes.c_uint16,),
            ("TARGET_POSITION", ctypes.c_int32),
            ("PROFILE_ACCELERATION", ctypes.c_uint32),
            ("PROFILE_DECELERATION", ctypes.c_uint32),
            ("PROFILE_VELOCITY", ctypes.c_uint32),
            ("MODES_OF_OPERATION", ctypes.c_int8),
            ("PHYSICAL_OUTPUTS", ctypes.c_uint32)

        ]
        
class PDOInput:

    class MapOne(ctypes.Structure):

        _pack_ = 1
        _fields_ = [
            ("STATUSWORD", ctypes.c_uint16),
        ]
def concatenateBits(nums: list[int], dataLengths: list[int] | int) -> int:
    """Combines multiple numbers together through bit concatenation into a single number with a total
    number of bits equal to the sum of the lengths of each data type rounded to the nearest 2^n."""

    if isinstance(dataLengths, int):
        dataLengths = [dataLengths]

    result = 0
    for i, num in enumerate(nums): # for each number
        result <<= dataLengths[i] # shift the result to the left by the length of the current number
        result |= num # bitwise OR the current number with the result
    return result

def makePDOMapping(objects: EPOS4Obj | list[EPOS4Obj], raiseErrorWhenOverLengthLimit = True) -> list[int]:
    """Create a 32 bit unsigned integer that can be used to map an object to a PDO. Integer
    is created by concatenating bits from the index, subindex, and length of the data type."""

    # If only a single command is entered, put it in a 1 element list
    if isinstance(objects, (Enum, tuple)):
        objects = [objects]

    # Handle use case where tuples are the input by forcing tuples in all use cases
    for i, objectToMap in enumerate(objects):
        if isinstance(objectToMap, Enum):
            objects[i] = objectToMap.value

    mappingIntegers = []
    totalLength = 0
    for objectToMap in objects:
        index, subIndex, ctype, *_ = objectToMap # The *_ is future proofing so that additions to type EPOS4Obj don't break the function
        length = ctypes.sizeof(ctype) * 8 # Multiply by 8 becuase sizeof uses bytes as units but bits are needed

        mappingIntegers += [concatenateBits([index, subIndex, length], [16, 8, 8])]
        totalLength += length

    if totalLength > 8 * 40 and raiseErrorWhenOverLengthLimit: # Maximum PDO length for EtherCAT is 40 bytes
        raise ValueError(f"Length of datatypes associated with requested addresses total {totalLength} bits, exceeding the 40 byte maximum for EtherCAT by {totalLength - 40 * 8} bits.") 
    
    return mappingIntegers

def assertStatuswordState(statusword: int, state: StatuswordStates) -> bool:
    """Asserts that the current state of the statusword matches the desired state."""
    maskedWord = (statusword & STATUSWORD_STATE_BITMASK) 
    maskedWord = maskedWord & state.value # if its all the same nothing in the masked work should change
    return maskedWord == state.value # compare the masked and anded word to the state value

def getStatuswordState(statusword: int) -> str:
    """Returns the current state name of the statusword."""
    for e in StatuswordStates: # for each statusword state
        if e.value == (statusword & STATUSWORD_STATE_BITMASK): # if the state matches the statusword
            return e.name # return the name of the state
    return "UNKNOWN STATE"

def assertNetworkManagementState(master: pysoem.CdefMaster, pysoemSTATE: int) -> bool:
    """Asserts that the NMT state matches that of the requested state."""
    if master.state_check(pysoemSTATE) == pysoemSTATE:
        return True
    else:
        return False
    
def getNetworkManagementState(master: pysoem.CdefMaster) -> str:
    """Returns the current state name of the network management state"""
    realState = master.state_check(pysoem.NONE_STATE)
    for state in networkManagementStates:
        if state.value == realState:
            return state.name
    return "UNKNOWN NMT STATE"

def SDORead(slave: pysoem.CdefSlave, address: EPOS4Obj | tuple) -> int:
    """SDO read to a single slave instance. The object should be of type EPOS4Obj 
    which are only accessible in the ObjDict class. For testing, using a tuple of 
    the form: (index, subindex, ctype) is fine."""
    
    # Handle annoying ObjDict.CMD throwing error because it wants ObjDict.CMD.value
    if isinstance(address, Enum):
        address = address.value # VSCode shows as unreachable, but it is ?!
    
    index, subIndex, ctype, *_ = address
    response = ctype.from_buffer_copy(slave.sdo_read(index=index, subindex=subIndex)).value
    return response

def SDOWrite(slave: pysoem.CdefSlave, data, address: EPOS4Obj | tuple, completeAccess=False) -> None:
    """SDO write to a single slave instance. The object should be of type EPOS4Obj 
    which are only accessible in the ObjDict class. For testing, using a tuple of 
    the form: (index, subindex, ctype) is fine.
    
    Notes:
     - Only supports single data point at the moment
    """

    # Handle annoying ObjDict.CMD throwing error because it wants ObjDict.CMD.value
    if isinstance(address, Enum):
        address = address.value
    index, subIndex, _, packFormat, *_ = address

    # Convert data to bytes with little endian format
    byteData = struct.pack('<' + packFormat, data)

    # Write data
    slave.sdo_write(index=index, subindex=subIndex, data=byteData)    

class EPOSObjectPrinter:
    """Object used for printing command information from the firmware spec guide"""

    def __init__(self, name: str, index: int, subIndex:int, humanReadableDataType: str, firmwareGuideObjectSection: str,
                 searchMarker: str = None, stateRequirements: dict = {'deviceState': None, 'NMTState': None}):
        
        self.name = name
        self.index = index
        self.subIndex = subIndex
        self.humanREadableDataType = humanReadableDataType
        self.firmwareGuideObjectSection = firmwareGuideObjectSection
        self.searchMarker = searchMarker
        self.requirements = stateRequirements

        match humanReadableDataType.upper():
            case "BOOLEAN" | "BOOL":
                packFormat = 'c'
                requiredctype = 'ctypes.c_bool'
            case "INTEGER8" | "INT8" | "SINT":
                packFormat = 'b'
                requiredctype = 'ctypes.c_int8'
            case "INTEGER16" | "INT16" | "INT":
                packFormat = 'h'
                requiredctype = 'ctypes.c_int16'
            case "INTEGER32" | "INT32" | "DINT":
                packFormat = 'i'
                requiredctype = 'ctypes.c_int32'
            case "INTEGER64" | "INT64" | "LINT":
                packFormat = 'q'
                requiredctype = 'ctypes.c_int64'
            case "UNSIGNED8" | "UINT8" | "USINT": 
                packFormat = 'B'
                requiredctype = 'ctypes.c_uint8'
            case "UNSIGNED16" | "UINT16" | "UINT":
                packFormat = 'H'
                requiredctype = 'ctypes.c_uint16'
            case "UNSIGNED32" | "UINT32" | "UDINT":
                packFormat = 'I'
                requiredctype = 'ctypes.c_uint32'
            case "UNSIGNED64" | "UINT64" | "ULINT":
                packFormat = 'Q'
                requiredctype = 'ctypes.c_uint64'
            case "REAL" | "FLOAT":
                packFormat = 'f'
                requiredctype = 'ctypes.c_float'
            case "VISIBLE_STRING":
                raise NotImplementedError
            case "OCTET_STRING":
                raise NotImplementedError
            case _:
                raise ValueError("Data type not found in EtherCAT conversion table.")

        self.packFormat = packFormat
        self.ctype = requiredctype
        #self.length = ctypes.sizeof(self.ctype) * 8 # Store bit length, not byte length

    def __repr__(self):
        """Setup to print a line statement that can be copied and pasted into the ObjDict class"""
        if self.searchMarker != None:
            return f"""{self.name} = EPOS4Obj(({hex(self.index)}, {hex(self.subIndex)}, {self.ctype}, '{self.packFormat}')) # {self.firmwareGuideObjectSection} % SEARCH MARKER: {self.searchMarker}"""

        return f"""{self.name} = EPOS4Obj(({hex(self.index)}, {hex(self.subIndex)}, {self.ctype}, '{self.packFormat}')) # {self.firmwareGuideObjectSection}"""

class ObjDict(Enum):
    """Object dictionary containing the same information as the Firmware Specification guide
    for the EPOS4 Micro. See section 6.2 for more details"""

    # EXAMPLE_CMD = (INDEX, SUBINDEX, C_DATA_TYPE, PACK_FORMAT_CHAR)

    NUMBER_OF_MAPPED_OBJECTS_IN_RXPDO_1 = EPOS4Obj((0x1600, 0x0, ctypes.c_uint8, 'B')) # 6.2.19.1 % SEARCH MARKER: Receive PDO mapping 1
    FIRST_MAPPED_OBJECT_IN_RXPDO_1 = EPOS4Obj((0x1600, 0x1, ctypes.c_uint32, 'I')) # 6.2.19.2 % SEARCH MARKER: Receive PDO 1 mapping
    SECOND_MAPPED_OBJECT_IN_RXPDO_1 = EPOS4Obj((0x1600, 0x2, ctypes.c_uint32, 'I')) # 6.2.19.2
    THIRD_MAPPED_OBJECT_IN_RXPDO_1 = EPOS4Obj((0x1600, 0x3, ctypes.c_uint32, 'I')) # 6.2.19.2
    FOURTH_MAPPED_OBJECT_IN_RXPDO_1 = EPOS4Obj((0x1600, 0x4, ctypes.c_uint32, 'I')) # 6.2.19.2
    FITH_MAPPED_OBJECT_IN_RXPDO_1 = EPOS4Obj((0x1600, 0x5, ctypes.c_uint32, 'I')) # 6.2.19.2
    SIXTH_MAPPED_OBJECT_IN_RXPDO_1 = EPOS4Obj((0x1600, 0x6, ctypes.c_uint32, 'I')) # 6.2.19.2
    SEVENTH_MAPPED_OBJECT_IN_RXPDO_1 = EPOS4Obj((0x1600, 0x7, ctypes.c_uint32, 'I')) # 6.2.19.2
    EIGHTH_MAPPED_OBJECT_IN_RXPDO_1 = EPOS4Obj((0x1600, 0x8, ctypes.c_uint32, 'I')) # 6.2.19.2
    NINETH_MAPPED_OBJECT_IN_RXPDO_1 = EPOS4Obj((0x1600, 0x9, ctypes.c_uint32, 'I')) # 6.2.19.2
    TENTH_MAPPED_OBJECT_IN_RXPDO_1 = EPOS4Obj((0x1600, 0xa, ctypes.c_uint32, 'I')) # 6.2.19.2
    ELEVENTH_MAPPED_OBJECT_IN_RXPDO_1 = EPOS4Obj((0x1600, 0xb, ctypes.c_uint32, 'I')) # 6.2.19.2
    TWELVETH_MAPPED_OBJECT_IN_RXPDO_1 = EPOS4Obj((0x1600, 0xc, ctypes.c_uint32, 'I')) # 6.2.19.2
    NUMBER_OF_MAPPED_OBJECTS_IN_RXPDO_2 = EPOS4Obj((0x1601, 0x0, ctypes.c_uint8, 'B')) # 6.2.20.1 % SEARCH MARKER: Receive PDO 2 mapping
    FIRST_MAPPED_OBJECT_IN_RXPDO_2 = EPOS4Obj((0x1601, 0x1, ctypes.c_uint32, 'I')) # 6.2.20.2 % SEARCH MARKER: Receive PDO 2 mapping
    SECOND_MAPPED_OBJECT_IN_RXPDO_2 = EPOS4Obj((0x1601, 0x2, ctypes.c_uint32, 'I')) # 6.2.20.2 % SEARCH MARKER: Receive PDO 2 mapping
    NUMBER_OF_USED_SYNC_MANAGER_CHANNELS = EPOS4Obj((0x1c00, 0x0, ctypes.c_uint8, 'B')) # 6.2.31.1
    COMMUNICATION_TYPE_SYNC_CHANNEL_0 = EPOS4Obj((0x1c00, 0x1, ctypes.c_uint8, 'B')) # 6.2.31.2
    COMMUNICATION_TYPE_SYNC_CHANNEL_1 = EPOS4Obj((0x1c00, 0x2, ctypes.c_uint8, 'B')) # 6.2.31.3
    COMMUNICATION_TYPE_SYNC_CHANNEL_2 = EPOS4Obj((0x1c00, 0x3, ctypes.c_uint8, 'B')) # 6.2.31.4
    COMMUNICATION_TYPE_SYNC_CHANNEL_3 = EPOS4Obj((0x1c00, 0x4, ctypes.c_uint8, 'B')) # 6.2.31.5
    NUMBER_OF_ASSIGNED_RXPDOS = EPOS4Obj((0x1c12, 0x0, ctypes.c_uint8, 'B')) # 6.2.32.1 % SEARCH MARKER: SYNC manager 2 PDO assignment
    FIRST_ASSIGNED_RXPDO = EPOS4Obj((0x1c12, 0x1, ctypes.c_uint16, 'H')) # 6.2.32.2 % SEARCH MARKER: SYNC manager 2 PDO assignment
    CONTROLWORD = EPOS4Obj((0x6040, 0x00, ctypes.c_uint16, 'H')) # 6.2.94
    STATUSWORD = EPOS4Obj((0x6041, 0x00, ctypes.c_uint16, 'H')) # 6.2.95
    MODES_OF_OPERATION = EPOS4Obj((0x6060, 0x00, ctypes.c_int8, 'b')) # 6.2.100
    MODES_OF_OPERATION_DISPLAY = EPOS4Obj((0x6061, 0x00, ctypes.c_int8, 'b')) # 6.2.101
    TARGET_POSITION = EPOS4Obj((0x607A, 0x00, ctypes.c_int32, 'i')) # 6.2.113
    PROFILE_VELOCITY = EPOS4Obj((0x6081, 0x00, ctypes.c_uint32, 'i')) # 6.2.118
    PROFILE_ACCELERATION = EPOS4Obj((0x6083, 0x00, ctypes.c_uint32, 'i')) # 6.2.119
    PROFILE_DECELERATION = EPOS4Obj((0x6084, 0x00, ctypes.c_uint32, 'i')) # 6.2.120
    PHYSICAL_OUTPUTS = EPOS4Obj((0x60FE, 0x01, ctypes.c_uint32, 'i')) # 6.2.146.1   % SEARCH MARKER: Digital outputs
    NBR_OF_CONFIGURED_MODULES = EPOS4Obj((0xf030, 0x0, ctypes.c_uint8, 'B')) # 6.2.151.1 % SEARCH MARKER: Modular device profile
    MODULE_1 = EPOS4Obj((0xf030, 0x1, ctypes.c_uint32, 'I')) # 6.2.151.2 % SEARCH MARKER: Modular device profile
    TARGET_VE

    def getInfo(identifier: str | int):
        """Search for a command and print all information associated with a particular command name or index.
        
        identifier: 
            can either be a string representing the command name in the firmware spec guide 
            or an int representing the index of a command in the firmware spec guide.
        """
        matchMode = False # False: match by int, True: match by index

        if isinstance(identifier, str):
            matchMode = True

        foundCMD = False
        for command in ObjDict:

            if matchMode and command.name == identifier:
                print(f"{command.name} = {command.value}")
                foundCMD = True
            elif not matchMode and command.value[0] == identifier:
                print(f"{command.name} = {command.value}")
                foundCMD = True

        if not foundCMD:
            print("No command with that identifier was found in the object dictionary. Not all\n commands from the firmware spec guide have been entered. Use the EPOSObject for help adding a command.")


### Error based commands. Implement once the "OCTET_STRING" data type has been implemented. ###
# print(EPOSObjectPrinter("MAXIMUM_MESSAGES", 0x10F3, 0x01, "UNSIGNED8", '6.2.13.1', searchMarker='Diagnosis History'))
# print(EPOSObjectPrinter("NEWEST_MESSAGE", 0x10F3, 0x02, "UNSIGNED8", '6.2.13.2', searchMarker='Diagnosis History'))
# print(EPOSObjectPrinter("NEWEST_ACKNOWLEDGED_MESSAGE", 0x10F3, 0x03, "UNSIGNED8", '6.2.13.3', searchMarker='Diagnosis History'))
# print(EPOSObjectPrinter("NEW_MESSAGES_AVAILABLE", 0x10F3, 0x04, "BOOLEAN", '6.2.13.4', searchMarker='Diagnosis History'))
# print(EPOSObjectPrinter("FLAGS", 0x10F3, 0x05, "UNSIGNED16", '6.2.13.5', searchMarker='Diagnosis History'))
# print(EPOSObjectPrinter("DIAGNOSIS_MESSAGE_1", 0x10F3, 0x06, "OCTET_STRING", '6.2.13.6', searchMarker='Diagnosis History'))
# print(EPOSObjectPrinter("DIAGNOSIS_MESSAGE_2", 0x10F3, 0x07, "OCTET_STRING", '6.2.13.6', searchMarker='Diagnosis History'))
# print(EPOSObjectPrinter("DIAGNOSIS_MESSAGE_3", 0x10F3, 0x08, "OCTET_STRING", '6.2.13.6', searchMarker='Diagnosis History'))
# print(EPOSObjectPrinter("DIAGNOSIS_MESSAGE_4", 0x10F3, 0x09, "OCTET_STRING", '6.2.13.6', searchMarker='Diagnosis History'))
# print(EPOSObjectPrinter("DIAGNOSIS_MESSAGE_5", 0x10F3, 0x0A, "OCTET_STRING", '6.2.13.6', searchMarker='Diagnosis History'))


# class testeposobject:

#     def __init__(self, val):
#         self.val = val

#         testeposobject.outputter(val)

#     def outputter(val):
#         return val

# anObject = testeposobject((1, 2, ctypes.c_uint16, 'i'))

# print(anObject)
# print(anObject.val)
