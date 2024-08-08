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

    ### Use modular device profile to assign the default PPM mode associated maps ###
    #  Rx: 0x1600 Tx: 1A00 ###
    ## Get informatoin about module device profile
    numConfiguredMDPModules = SDORead(device, ObjDict.NBR_OF_CONFIGURED_MODULES)
    MDPModule1 = SDORead(device, ObjDict.MODULE_1)

    print(f"""Number of configured MDP modules: {numConfiguredMDPModules}
Module 1 bits: {MDPModule1}""") # Shows 0 and 0
    
    # SDOWrite(device, 0, ObjDict.NUMBER_OF_ASSIGNED_RXPDOS, completeAccess=True)
    # SDOWrite(device, 0x1600, ObjDict.FIRST_ASSIGNED_RXPDO, completeAccess=True)
    # SDOWrite(device, 1, ObjDict.NUMBER_OF_ASSIGNED_RXPDOS, completeAccess=True)
    # assignedRxPDO = SDORead(device, ObjDict.FIRST_ASSIGNED_RXPDO)
    # print("Selected PDO Map: ", hex(assignedRxPDO))

    ### Attempt to change the PDO mapping ###
    """I'm working with the theory that I cannot check if these change 
    in realtime because we must have a defined order of SDOwrites and
    any interruption will result in a warning/error/fault.

    Assemble 32bit mapping number composed of concatenated index, subindex, and length bits
    for our desired addresses. Our end goal is to assign a PDO corresponding to the MDP default
    since I can't seem to get that to work and I think changing PDO assignment manually is a reasonable
    tradeoff between time, complexity, readability, and leveraging all of the maxon features."""


    ## PPM Rx ##
    # Create all of our map integers to make the SDO fast enough to not get watchdogged/timedout (by slave)
    PPM_MDP_DEFAULT_OBJS = [ObjDict.CONTROLWORD, ObjDict.TARGET_POSITION, ObjDict.PROFILE_ACCELERATION, ObjDict.PROFILE_DECELERATION, ObjDict.PROFILE_VELOCITY, ObjDict.MODES_OF_OPERATION, ObjDict.PHYSICAL_OUTPUTS]
    mappingIntegers = makePDOMapping(PPM_MDP_DEFAULT_OBJS)

    ## Follow described procedure in 6.2.19 for changing PDO map ##
    # a) Write zero 
    SDOWrite(device, 0, ObjDict.NUMBER_OF_MAPPED_OBJECTS_IN_RXPDO_1)

    # b) Write to each sub mailbox
    for i, integer in enumerate(mappingIntegers):
        address = (0x1600, i+1, ctypes.c_uint32, "I")
        SDOWrite(device, integer, address)

    # c) Write the actual number of objects
    SDOWrite(device, len(PPM_MDP_DEFAULT_OBJS), ObjDict.NUMBER_OF_MAPPED_OBJECTS_IN_RXPDO_1)

    numMappedObjects = SDORead(device, ObjDict.NUMBER_OF_MAPPED_OBJECTS_IN_RXPDO_1)
    print(f"RxPDO 1 mapping information:\nNumber of mapped objects: {numMappedObjects}")
    rxpdo1_mapped_objects = [ObjDict.FIRST_MAPPED_OBJECT_IN_RXPDO_1, ObjDict.SECOND_MAPPED_OBJECT_IN_RXPDO_1, ObjDict.THIRD_MAPPED_OBJECT_IN_RXPDO_1, ObjDict.FOURTH_MAPPED_OBJECT_IN_RXPDO_1, 
                             ObjDict.FITH_MAPPED_OBJECT_IN_RXPDO_1, ObjDict.SIXTH_MAPPED_OBJECT_IN_RXPDO_1, ObjDict.SEVENTH_MAPPED_OBJECT_IN_RXPDO_1]
    for i, mappedObject in enumerate(rxpdo1_mapped_objects):
        mapAddress = SDORead(device, mappedObject)
        print(f"Index {hex(i+1)} mapped to {hex(mapAddress)} ")


    ### Commented out to try and first change PDO mapping 1 ###
    # ## Make sure the drive is in PPM mode ###
    # SDOWrite(device, 1, ObjDict.MODES_OF_OPERATION)

    # ## Assign PPM mode device profile ##
    # SDOWrite(device, 0x61000000, ObjDict.MODULE_1, completeAccess=True)
    # # Check if it worked
    # MDPModule1 = SDORead(device, ObjDict.MODULE_1)
    # print("Newly assigned module 1 bits: ", hex(MDPModule1))

    # ## Overwrite to the Rx and Tx maps
    # SDOWrite(device, 1, ObjDict.NBR_OF_CONFIGURED_MODULES)
    # # Check if it worked
    # numConfiguredMDPModules = SDORead(device, ObjDict.NBR_OF_CONFIGURED_MODULES)
    # print("Number of configured modules in the MDP: ", numConfiguredMDPModules)
    
    SDOWrite(device, 0, ObjDict.NUMBER_OF_ASSIGNED_RXPDOS, completeAccess=True)
    SDOWrite(device, 0x1600, ObjDict.FIRST_ASSIGNED_RXPDO, completeAccess=True)
    SDOWrite(device, 1, ObjDict.NUMBER_OF_ASSIGNED_RXPDOS, completeAccess=True)
    assignedRxPDO = SDORead(device, ObjDict.FIRST_ASSIGNED_RXPDO)
    print("Selected PDO Map: ", hex(assignedRxPDO))

    device.set_watchdog('pdi', 6553.5)
    device.set_watchdog('processdata', 6553.5)


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

    else:
        print("In op state!!!")
    
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