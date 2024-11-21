"""
Entry point for LRIS2's soft etherCAT master. Sets up listening socket and initializes the soft master.

Currently assumes that all connected controllers are EPOS4 Micro 24/5.
"""
import yaml
import logging
from src.server import etherCATSocketServer
from src.master import master, slave
from src.helpers import makePDOMapping

# Set up logging for the application startup process
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

def EPOS4MicroTRB_12CC_Config(slaveNum):
    """ Configures an EPOS4 Micro TRB 12CC device """
    global EPOS4MicroMaster

    dev: slave
    dev = EPOS4MicroMaster.slaves[slaveNum]
    logging.debug(f"Configuring slave {slaveNum} (EPOS4 Micro 24/5)")

    # Define the Process Data Objects for PPM (Rx and Tx)
    PPMRx = [
        dev.objectDictionary.CONTROLWORD,
        dev.objectDictionary.TARGET_POSITION,
        dev.objectDictionary.PROFILE_ACCELERATION,
        dev.objectDictionary.PROFILE_DECELERATION,
        dev.objectDictionary.PROFILE_VELOCITY,
        dev.objectDictionary.MODES_OF_OPERATION, 
        dev.objectDictionary.PHYSICAL_OUTPUTS
    ]
    PPMTx = [
        dev.objectDictionary.STATUSWORD,
        dev.objectDictionary.POSITION_ACTUAL_VALUE,
        dev.objectDictionary.VELOCITY_ACTUAL_VALUE, 
        dev.objectDictionary.FOLLOWING_ERROR_ACTUAL_VALUE, 
        dev.objectDictionary.MODES_OF_OPERATION_DISPLAY, 
        dev.objectDictionary.DIGITAL_INPUTS
    ]
    
    # Create rx and tx map integers
    rxAddressInts = makePDOMapping(PPMRx)
    txAddressInts = makePDOMapping(PPMTx)

    # Assign rx map
    dev.SDOWrite(dev.objectDictionary.NUMBER_OF_MAPPED_OBJECTS_IN_RXPDO_1, 0)
    for i, addressInt in enumerate(rxAddressInts):
        dev.SDOWrite((0x1600, i + 1, 'I'), addressInt)
    dev.SDOWrite(dev.objectDictionary.NUMBER_OF_MAPPED_OBJECTS_IN_RXPDO_1, len(PPMRx))

    # Assign tx map
    dev.SDOWrite(dev.objectDictionary.NUMBER_OF_MAPPED_OBJECTS_IN_TXPDO_1, 0)
    for i, addressInt in enumerate(txAddressInts):
        dev.SDOWrite((0x1A00, i + 1, 'I'), addressInt)
    dev.SDOWrite(dev.objectDictionary.NUMBER_OF_MAPPED_OBJECTS_IN_TXPDO_1, len(PPMTx))

    dev.currentRxPDOMap = PPMRx
    dev.currentTxPDOMap = PPMTx

    # Configure Digital Inputs (example)
    dev.SDOWrite(dev.objectDictionary.DIGITAL_INPUT_CONFIGURATION_DGIN_1, 255) 
    dev.SDOWrite(dev.objectDictionary.DIGITAL_INPUT_CONFIGURATION_DGIN_2, 1) 
    dev.SDOWrite(dev.objectDictionary.DIGITAL_INPUT_CONFIGURATION_DGIN_1, 0) 

    # Set the home offset move distance
    dev.SDOWrite(dev.objectDictionary.HOME_OFFSET_MOVE_DISTANCE, -622080)
    logging.debug("Slave configuration complete.")

def main():
    """ Main entry point to initialize the EtherCAT soft master and start the server """
    global EPOS4MicroMaster

    # Load the master settings from YAML file
    try:
        with open('settings/masterSettings.yaml', 'r') as f:
            masterSettings = yaml.safe_load(f)
            logging.info("Loaded master settings.")
    except Exception as e:
        logging.error(f"Error loading master settings: {e}")
        return

    try:
        # Initialize the soft master with the slave configuration functions
        EPOS4MicroMaster = master(
            masterSettings['networkInterfaceName'],
            slaveConfigFuncs=[EPOS4MicroTRB_12CC_Config, EPOS4MicroTRB_12CC_Config, EPOS4MicroTRB_12CC_Config]
        )
        logging.info(f"Initialized master with network interface: {masterSettings['networkInterfaceName']}")

        # Open network interface and initialize slaves
        EPOS4MicroMaster.openNetworkInterface()
        EPOS4MicroMaster.initializeSlaves()
        EPOS4MicroMaster.configureSlaves()
        logging.info("Slaves initialized and configured.")

    except Exception as e:
        logging.error(f"Error initializing the master or slaves: {e}")
        return

    # Start the EtherCAT Socket Server to listen for client connections
    try:
        server = etherCATSocketServer(EPOS4MicroMaster)
        logging.info("EtherCAT Socket Server initialized.")
        server.run()
    except Exception as e:
        logging.error(f"Error starting EtherCAT Socket Server: {e}")
        return

if __name__ == '__main__':
    main()
