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

PATH = os.path.abspath(__file__).replace('src/server.py', '')
sys.path.append(PATH)

from src.master import master, slave
from src.helpers import StatuswordStates

class etherCATSocketServer:

    def __init__(self, softMasterInstance: master):

        self.softMaster = softMasterInstance

        with open(f'{PATH}/settings/serverSettings.yaml', 'r') as f:
            serverSettings = yaml.safe_load(f)
        
        self.host = serverSettings['host']
        self.port = serverSettings['port']

        self.serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.serverSocket.bind((self.host, self.port))
        self.serverSocket.listen(5)

    def run(self):
        while True:
            connection, address = self.serverSocket.accept()
            buf = connection.recv(2)
            if len(buf) > 0: # Received a message
                messageLength = struct.unpack('<H', buf)[0] # Get message length
                buf = connection.recv(1)  # Get action type code from the 3rd byte
                actionType = struct.unpack('<B', buf)[0]

                match actionType:
                    case 0:
                        self.HomingPDO(connection, messageLength)
                    case 1:
                        self.PPMPDO(connection, messageLength)
                    case _:
                        print('Invalid action type')

    
    def PPMPDO(self, connection: socket.socket, messageLength):

        print("Moving to target positions")
        targetPositions = []        
        while messageLength - 3 > 0: # Subtract message length by 3 to account for the 3 bytes already read
            buf = connection.recv(4)
            targetPositions += [struct.unpack('<i', buf)[0]]

            messageLength -= len(buf)

        self.softMaster.enablePDO()
        self.softMaster.sendPDO()
        self.softMaster.receivePDO()
        self.softMaster.goToPositions(targetPositions)
        self.softMaster.disablePDO()
        print("Done moving")

    def HomingPDO(self, connection, messageLength):
        print("Homing all axes")
        self.softMaster.enablePDO()
        self.softMaster.sendPDO()
        self.softMaster.receivePDO()
        self.softMaster.changeDeviceStatesPDO(StatuswordStates.OPERATION_ENABLED)
        self.softMaster.performHoming()
        print("Target reached/homing attained")
        self.softMaster.disablePDO()
        print("Done homing")