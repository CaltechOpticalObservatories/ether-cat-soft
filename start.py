"""
Entry point for LRIS2's soft etherCAT master. Sets up listening socket and initializes the soft master.

Currently assumes that all connected controllers are EPOS4 Micro 24/5.
"""
import yaml
from src.server import etherCATSocketServer
from src.master import master, slave
from src.helpers import makePDOMapping

def EPOS4MicroTRB_12CC_Config(slaveNum):
    global EPOS4MicroMaster

    dev: slave
    dev = EPOS4MicroMaster.slaves[slaveNum]
    PPMRx = [dev.objectDictionary.CONTROLWORD, dev.objectDictionary.TARGET_POSITION,
                    dev.objectDictionary.PROFILE_ACCELERATION, dev.objectDictionary.PROFILE_DECELERATION,
                    dev.objectDictionary.PROFILE_VELOCITY, dev.objectDictionary.MODES_OF_OPERATION, 
                    dev.objectDictionary.PHYSICAL_OUTPUTS]
    PPMTx = [dev.objectDictionary.STATUSWORD, dev.objectDictionary.POSITION_ACTUAL_VALUE, dev.objectDictionary.VELOCITY_ACTUAL_VALUE, 
                      dev.objectDictionary.FOLLOWING_ERROR_ACTUAL_VALUE, dev.objectDictionary.MODES_OF_OPERATION_DISPLAY, dev.objectDictionary.DIGITAL_INPUTS]
    
    ### Create rx and tx map integers
    rxAddressInts = makePDOMapping(PPMRx)
    txAddressInts = makePDOMapping(PPMTx)

    ### Assign rx map ###
    dev.SDOWrite(dev.objectDictionary.NUMBER_OF_MAPPED_OBJECTS_IN_RXPDO_1, 0)
    for i, addressInt in enumerate(rxAddressInts):
        dev.SDOWrite((0x1600, i+1, 'I'), addressInt)
    dev.SDOWrite(dev.objectDictionary.NUMBER_OF_MAPPED_OBJECTS_IN_RXPDO_1, len(PPMRx))

    ### Assign tx map ###
    dev.SDOWrite(dev.objectDictionary.NUMBER_OF_MAPPED_OBJECTS_IN_TXPDO_1, 0)
    for i, addressInt in enumerate(txAddressInts):
        dev.SDOWrite((0x1A00, i+1, 'I'), addressInt)
    dev.SDOWrite(dev.objectDictionary.NUMBER_OF_MAPPED_OBJECTS_IN_TXPDO_1, len(PPMTx))

    dev.currentRxPDOMap = PPMRx
    dev.currentTxPDOMap = PPMTx

    # Digital inputs
    dev.SDOWrite(dev.objectDictionary.DIGITAL_INPUT_CONFIGURATION_DGIN_1, 255) 
    dev.SDOWrite(dev.objectDictionary.DIGITAL_INPUT_CONFIGURATION_DGIN_2, 1) 
    dev.SDOWrite(dev.objectDictionary.DIGITAL_INPUT_CONFIGURATION_DGIN_1, 0) 

    # Set the home offset move distance
    dev.SDOWrite(dev.objectDictionary.HOME_OFFSET_MOVE_DISTANCE, -622080)

    # TODO: Remove this is done for debugging
    print(f"Slave {slaveNum} gains:")
    controllerPGain = dev.SDORead(dev.objectDictionary.CURRENT_CONTROLLER_P_GAIN)
    controllerIGain = dev.SDORead(dev.objectDictionary.CURRENT_CONTROLLER_I_GAIN)

    print(f"P Gain: {controllerPGain},  I Gain: {controllerIGain}")


def main():
    global EPOS4MicroMaster

    with open('settings/masterSettings.yaml', 'r') as f:
        masterSettings = yaml.safe_load(f)

    EPOS4MicroMaster = master(masterSettings['networkInterfaceName'], slaveConfigFuncs=[EPOS4MicroTRB_12CC_Config, EPOS4MicroTRB_12CC_Config, EPOS4MicroTRB_12CC_Config])
    EPOS4MicroMaster.openNetworkInterface()
    EPOS4MicroMaster.initializeSlaves()
    EPOS4MicroMaster.configureSlaves()

    server = etherCATSocketServer(EPOS4MicroMaster)
    server.run()

if __name__ == '__main__':
    main()