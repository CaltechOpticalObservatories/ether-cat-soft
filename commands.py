import socket
import struct
import time
import argparse

class Commands:
    def __init__(self, host, port):
        self.host = host
        self.port = port

    def send_ppmpdo_message(self, target_position, slave_id=None):
        """Send a PPM PDO message to the server for a specific slave or all slaves."""
        action_type = 1  # Action type for PPM PDO (1)
        
        # Create the message base: [Message Length (2 bytes), Action Type (1 byte)]
        message = struct.pack('<H', 7)  # Message length (7 bytes in total)
        message += struct.pack('<B', action_type)  # Action type (1 byte)

        # Add the slave ID if provided, and then the target position
        if slave_id is not None:
            message += struct.pack('<B', slave_id)  # Slave ID (1 byte)
        message += struct.pack('<i', target_position)  # Target position (4 bytes)

        # Send the message to the server
        self._send_message(message)

    def send_homingpdo_message(self):
        """Send a Homing PDO message to the server."""
        action_type = 0  # Action type for Homing PDO (0)

        # Pack the message according to the protocol:
        # [Message Length (2 bytes), Action Type (1 byte)]
        message = struct.pack('<H', 3)  # Message length (3 bytes in total)
        message += struct.pack('<B', action_type)  # Action type (1 byte)

        # Send the message to the server
        self._send_message(message)

    def send_multiple_ppmpdo_messages(self, target_positions, slave_ids=None, wait_time=1):
        """Send PPM PDO messages for a list of target positions to specific slaves with wait times in between."""
        if slave_ids is None:
            slave_ids = range(len(target_positions))  # Default to move all slaves

        if len(target_positions) != len(slave_ids):
            print("Error: The number of target positions must match the number of slave IDs.")
            return

        for idx, (target_position, slave_id) in enumerate(zip(target_positions, slave_ids)):
            print(f"Moving slave {slave_id} to position {target_position}")
            self.send_ppmpdo_message(target_position, slave_id)
            time.sleep(wait_time)  # Wait for specified time

    def _send_message(self, message):
        """Send a message to the server and optionally receive a response."""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
                client_socket.connect((self.host, self.port))  # Connect to the server
                client_socket.send(message)  # Send the message

                # Optionally, receive a response (e.g., success or error)
                # response = client_socket.recv(1)
                # if response:
                #     print(f"Received response: {response}")
                # else:
                #     print("No response from server")
        except Exception as e:
            print(f"Error sending message: {e}")

    def get_slaves_info(self):
        """Send a request to the server to fetch slave information."""
        try:
            # Send a specific request to the server for slave information
            request_message = struct.pack('<H', 5)  # For example: message length (5 bytes)
            request_message += struct.pack('<B', 0x02)  # Action Type (e.g., 0x02 = Get Slave Info)
            self._send_message(request_message)

        except Exception as e:
            print(f"Error requesting slave info: {e}")

def main():
    parser = argparse.ArgumentParser(description="Send commands to the server.")
    parser.add_argument('host', type=str, help='Server host IP address')
    parser.add_argument('port', type=int, help='Server port')
    parser.add_argument('--send-ppmpdo', type=int, help='Target position for PPM PDO message')
    parser.add_argument('--send-homingpdo', action='store_true', help='Send Homing PDO message')
    parser.add_argument('--send-multiple-ppmpdo', nargs='+', type=int, metavar='POSITION', help='Send multiple PPM PDO messages with target positions')
    parser.add_argument('--slave-ids', nargs='+', type=int, metavar='SLAVE_ID', help='List of slave IDs to send PPM PDO messages to')
    parser.add_argument('--wait-time', type=float, default=1, help='Wait time (in seconds) between sending messages')
    parser.add_argument('--get-slaves-info', action='store_true', help='Retrieve and print information about slaves')

    args = parser.parse_args()

    commands = Commands(args.host, args.port)

    # Sending PPM PDO message to a specific slave or node
    if args.send_ppmpdo is not None:
        if args.slave_ids:
            for slave_id in args.slave_ids:
                print(f"Sending PPM PDO message to slave {slave_id} with target position {args.send_ppmpdo}")
                commands.send_ppmpdo_message(args.send_ppmpdo, slave_id)
        else:
            print(f"Sending PPM PDO message to all slaves with target position {args.send_ppmpdo}")
            commands.send_ppmpdo_message(args.send_ppmpdo)

    # Sending Homing PDO message
    if args.send_homingpdo:
        print("Sending Homing PDO message")
        commands.send_homingpdo_message()

    # Sending multiple PPM PDO messages
    if args.send_multiple_ppmpdo:
        if args.slave_ids:
            if len(args.send_multiple_ppmpdo) != len(args.slave_ids):
                print("Error: The number of target positions must match the number of slave IDs.")
            else:
                print(f"Sending multiple PPM PDO messages with positions: {args.send_multiple_ppmpdo} and slave IDs: {args.slave_ids}")
                commands.send_multiple_ppmpdo_messages(args.send_multiple_ppmpdo, args.slave_ids, args.wait_time)
        else:
            print(f"Sending multiple PPM PDO messages to all slaves with positions: {args.send_multiple_ppmpdo}")
            commands.send_multiple_ppmpdo_messages(args.send_multiple_ppmpdo)

    # Retrieving slave info
    if args.get_slaves_info:
        print("Fetching slaves information:")
        commands.get_slaves_info()

if __name__ == '__main__':
    main()
