import pysoem
import time
from helpers2 import *

def EPOS4MicroSetup(axisNumber):
    global master

    device = master.slaves[axisNumber]

    ### Change sync manager 2 to recieve PDO mapping 2 ###
    if not assertNetworkManagementState(master, pysoem.PREOP_STATE):
        print("Can't write sync manager PDO assignements. Not in pre-op sate. Exiting")
        exit(1)

    SDOWrite(device, 0, ObjDict.NUMBER_OF_ASSIGNED_RXPDOS, completeAccess=True)
    SDOWrite(device, 0x1601, ObjDict.FIRST_ASSIGNED_RXPDO, completeAccess=True)
    SDOWrite(device, 1, ObjDict.NUMBER_OF_ASSIGNED_RXPDOS, completeAccess=True)
    assignedRxPDO = SDORead(device, ObjDict.FIRST_ASSIGNED_RXPDO)
    print("Selected PDO Map: ", hex(assignedRxPDO))

    ### Gather and display information about the selected PDO map of sync manager 2 ###
    numMappedObjects = SDORead(device, ObjDict.NUMBER_OF_MAPPED_OBJECTS_IN_RXPDO_2)
    firstObject = SDORead(device, ObjDict.FIRST_MAPPED_OBJECT_IN_RXPDO_2)
    secondObject = SDORead(device, ObjDict.SECOND_MAPPED_OBJECT_IN_RXPDO_2)

    print(f"""RxPDO 2 mapping information: 
Number of mapped objects: {numMappedObjects}
First Object: {hex(firstObject)}
Second Object: {hex(secondObject)}""")


def main():
    global master

    # Create master instance and open it
    master = pysoem.Master()
    master.open('enp87s0')

    numberOfAxes = master.config_init()
    print(f"{numberOfAxes=}")
    if numberOfAxes <= 0:
        print("Unable to connect to eval board. Exiting.")
        exit(1)

    # Assign configuration function to map PDO when transitioning from 
    # Pre-OP to Safe-OP state
    device = master.slaves[0]
    device.config_func = EPOS4MicroSetup

    # Check the network management state
    print(f"NMT State: {getNetworkManagementState(master)}")


    # Make sure device state is switch on disabled
    statusword = SDORead(device, ObjDict.STATUSWORD)
    print(f"Statusword State: {getStatuswordState(statusword)}")
    if not assertStatuswordState(statusword, StatuswordStates.SWITCH_ON_DISABLED):
        print("Device is not in switch on disabled state. Exiting.")
        exit(1)

    ### Gather and display information about the sync managers ###
    # Different available sync channels
    numUsedSyncChannels = SDORead(device, ObjDict.NUMBER_OF_USED_SYNC_MANAGER_CHANNELS)
    ComTypeChan0 = SDORead(device, ObjDict.COMMUNICATION_TYPE_SYNC_CHANNEL_0)
    ComTypeChan1 = SDORead(device, ObjDict.COMMUNICATION_TYPE_SYNC_CHANNEL_1)
    ComTypeChan2 = SDORead(device, ObjDict.COMMUNICATION_TYPE_SYNC_CHANNEL_2)
    ComTypeChan3 = SDORead(device, ObjDict.COMMUNICATION_TYPE_SYNC_CHANNEL_3)

    # State of the sync channel 2 which receives messages from master 
    PDOOnOff = SDORead(device, ObjDict.NUMBER_OF_ASSIGNED_RXPDOS)
    assignedRxPDO = SDORead(device, ObjDict.FIRST_ASSIGNED_RXPDO)

    print(f"""
Number of used sync channels: {numUsedSyncChannels}
Channel 0 value: {ComTypeChan0}  Default Value: 1
Channel 1 value: {ComTypeChan1}  Default Value: 2
Channel 2 value: {ComTypeChan2}  Default Value: 3
Channel 3 value: {ComTypeChan3}  Default Value: 4
""")
    print(f"""Channel 2 Sync Manager:
PDO enabled: {PDOOnOff}
Selected PDO Map: {hex(assignedRxPDO)}""")
    
    ### Gather and display information about the selected PDO map of sync manager 2 ###
    numMappedObjects = SDORead(device, ObjDict.NUMBER_OF_MAPPED_OBJECTS_IN_RXPDO_1)
    firstObject = SDORead(device, ObjDict.FIRST_MAPPED_OBJECT_IN_RXPDO_1)

    print(f"""RxPDO 1 mapping information: 
Number of mapped objects: {numMappedObjects}
First Object: {hex(firstObject)}""")

    master.config_map()
    if not assertNetworkManagementState(master, pysoem.SAFEOP_STATE):
        print("Could not assert safe op state after config map. Exiting.")
        master.close()
        exit(1)
    else:
        print("In op state!!!")
    
    print("Before sending: ")
    print(f"Output proccess data: {device.output}")
    print(f"Input proccess data: {device.input}")
    master.send_processdata()
    master.receive_processdata()

    # Request operational state
    master.state = pysoem.OP_STATE
    master.write_state()

    master.send_processdata()
    master.receive_processdata()

    master.send_processdata()
    master.receive_processdata()

    print(assertNetworkManagementState(master, pysoem.OP_STATE))
    if not assertNetworkManagementState(master, pysoem.OP_STATE):
        print("Could not transition from safe op state to operational state. Exiting")
        print(getNetworkManagementState(master))

        # Checking diagnosis settings
        errorSettings = SDORead(device, (0x10F3, 0x05, ctypes.c_uint16))
        print("Error settings: ", bin(errorSettings))

        newestMessage = SDORead(device, (0x10F3, 0x02, ctypes.c_uint8))
        print("Index of the most recent diagnosis message: ", newestMessage)

        # Checking error/diagnosis messages
        test = device.sdo_read(index=0x10F3, subindex=0x0A)
        print(test, len(test))
        master.close()
        exit(1)
    


    print("Before sending: ")
    print(f"Output proccess data: {device.output}")
    print(f"Input proccess data: {device.input}")
    master.send_processdata()
    master.receive_processdata()

    print("After sending: ")
    print(f"Output proccess data: {device.output}")
    print(f"Input proccess data: {device.input}")

if __name__ == '__main__':
    main()