"""
Much of the communication protocol is based off of: 
https://docs.python.org/3/howto/sockets.html

Communication protocol:
Bytes 1, 2: Length of the entire message - unsigned 16 bit
Byte 3: Action code - unsigned 8 bit
    - 0: Homing PDO
    - 1: PPM PDO

Iterative/repeating byte pattern:
    The following are bytes are assigned per slave. The pattern is repeated such that the first pattern
    is applied to the slave with index 0, and so on

    PDO PPM:
        byte 4, 5, 6, 7:  Target position 32 bit integer
    
    PDO Homing:
        None


        
DEV NOTES:
Future protocol:
Slave addressing:
2 bytes: Slave node number
1 byte: Message type (reserved for expansion of this protocol)
    0: 

"""
import os
import sys
import socket
import struct
import yaml
import logging

# Set up logging for better debugging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

PATH = os.path.abspath(__file__).replace('src/server.py', '')
sys.path.append(PATH)

from src.master import master, slave
from src.helpers import StatuswordStates


class etherCATSocketServer:

    def __init__(self, softMasterInstance: master):
        self.softMaster = softMasterInstance

        # Load server settings from YAML config
        try:
            with open(f'{PATH}/settings/serverSettings.yaml', 'r') as f:
                serverSettings = yaml.safe_load(f)
            self.host = serverSettings['host']
            self.port = serverSettings['port']
        except Exception as e:
            logging.error(f"Error loading server settings: {e}")
            raise

        # Initialize server socket
        try:
            self.serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.serverSocket.bind((self.host, self.port))
            self.serverSocket.listen(5)
            logging.info(f"Server listening on {self.host}:{self.port}")
        except Exception as e:
            logging.error(f"Error setting up server socket: {e}")
            raise

    def run(self):
        while True:
            connection, address = self.serverSocket.accept()
            logging.info(f"Accepted connection from {address}")
            try:
                buf = connection.recv(2)
                if len(buf) > 0:  # Received a message
                    messageLength = struct.unpack('<H', buf)[0]  # Get message length
                    logging.debug(f"Message length: {messageLength}")

                    buf = connection.recv(1)  # Get action type byte
                    if not buf:
                        logging.error("Failed to receive action type byte")
                        continue
                    actionType = struct.unpack('<B', buf)[0]

                    logging.debug(f"Action type: {actionType}")

                    # Process based on action type
                    match actionType:
                        case 0:
                            self.HomingPDO(connection, messageLength)
                        case 1:
                            self.PPMPDO(connection, messageLength)
                        case 2:
                            self.get_slaves_info(connection)
                        case _:
                            logging.error('Invalid action type received')

            except Exception as e:
                logging.error(f"Error while processing message: {e}")
                connection.close()

    def get_slaves_info(self):
        """Handle request to send slave information to the client."""
        # Start with a base message length (e.g., 2 bytes for the length and 1 byte for action type)
        self.softMaster.getSlavesInfo()

    def PPMPDO(self, connection: socket.socket, messageLength: int, slave_ids: list[int] = None):
        """
        Message Length:  2 bytes
        Action Type:     1 byte
        Slave ID 1:      1 byte
        Target Position 1: 4 bytes
        Slave ID 2:      1 byte
        Target Position 2: 4 bytes
        """
        try:
            logging.info("Moving to target positions")
            targetPositions = []
            slaveIds = []
            remainingLength = messageLength - 3  # Subtract 3 bytes already read (length and action type)
            logging.debug(f"Remaining length to process: {remainingLength}")

            # Read slave IDs and target positions from the connection
            while remainingLength > 0:  # Loop through the message and extract slave IDs and target positions
                # Read the slave ID (1 byte)
                slave_id_buf = connection.recv(1)
                if not slave_id_buf:
                    logging.warning("Failed to receive slave ID byte")
                    break
                slave_id = struct.unpack('<B', slave_id_buf)[0]
                slaveIds.append(slave_id)
                remainingLength -= len(slave_id_buf)
                logging.debug(f"Received slave ID: {slave_id}, remaining length: {remainingLength}")

                # Read the target position (4 bytes)
                target_position_buf = connection.recv(4)
                if not target_position_buf:
                    logging.warning("Failed to receive target position byte")
                    break
                target_position = struct.unpack('<i', target_position_buf)[0]
                targetPositions.append(target_position)
                remainingLength -= len(target_position_buf)
                logging.debug(f"Received target position: {target_position}, remaining length: {remainingLength}")

            # If no slave_ids are provided, use the list of slave IDs we've just read
            if slave_ids is None:
                slave_ids = slaveIds  # Use the slave IDs received in the message

            # Check that the number of positions matches the number of slaves (or slave_ids)
            if len(targetPositions) != len(slave_ids):
                raise ValueError("The number of target positions must match the number of slave IDs provided.")

            # Handle the movement process
            self.softMaster.enablePDO()
            self.softMaster.sendPDO()
            self.softMaster.receivePDO()
            self.softMaster.changeDeviceStatesPDO(StatuswordStates.OPERATION_ENABLED)

            # Use slave_ids to assign target positions to specific slaves
            self.softMaster.goToPositions(targetPositions, slave_ids=slave_ids, printActualPosition=True)

            self.softMaster.changeDeviceStatesPDO(StatuswordStates.QUICK_STOP_ACTIVE)
            self.softMaster.disablePDO()

            logging.info("Done moving")

        except Exception as e:
            logging.error(f"Error in PPMPDO: {e}")


    def HomingPDO(self, connection: socket.socket, messageLength: int):
        try:
            logging.info("Homing all axes")
            self.softMaster.enablePDO()
            self.softMaster.sendPDO()
            self.softMaster.receivePDO()
            self.softMaster.changeDeviceStatesPDO(StatuswordStates.OPERATION_ENABLED)
            self.softMaster.performHoming()
            logging.info("Target reached/homing attained")
            self.softMaster.changeDeviceStatesPDO(StatuswordStates.QUICK_STOP_ACTIVE)
            self.softMaster.disablePDO()
            logging.info("Done homing")
        except Exception as e:
            logging.error(f"Error in HomingPDO: {e}")
