import socket
import struct
import time
import argparse

class Commands:
    def __init__(self, host, port):
        self.host = host
        self.port = port

    def send_ppmpdo_message(self, target_position):
        """Send a PPM PDO message to the server."""
        action_type = 1  # Action type for PPM PDO (1)

        # Pack the message according to the protocol:
        # [Message Length (2 bytes), Action Type (1 byte), Target Position (4 bytes)]
        message = struct.pack('<H', 7)  # Message length (7 bytes in total)
        message += struct.pack('<B', action_type)  # Action type (1 byte)
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

    def send_multiple_ppmpdo_messages(self, target_positions, wait_time):
        """Send PPM PDO messages for a list of target positions with wait times in between."""
        for idx, target_position in enumerate(target_positions):
            print(f"Moving to P{idx + 1}: {target_position}")
            self.send_ppmpdo_message(target_position)
            time.sleep(wait_time)  # Wait for specified time

    def _send_message(self, message):
        """Send a message to the server and optionally receive a response."""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
                client_socket.connect((self.host, self.port))  # Connect to the server
                client_socket.send(message)  # Send the message

                # # Optionally, receive a response (e.g., success or error)
                # response = client_socket.recv(1)
                # if response:
                #     print(f"Received response: {response}")
                # else:
                #     print("No response from server")
        except Exception as e:
            print(f"Error sending message: {e}")

def main():
    parser = argparse.ArgumentParser(description="Send commands to the server.")
    parser.add_argument('host', type=str, help='Server host IP address')
    parser.add_argument('port', type=int, help='Server port')
    parser.add_argument('--send-ppmpdo', type=int, help='Target position for PPM PDO message')
    parser.add_argument('--send-homingpdo', action='store_true', help='Send Homing PDO message')
    parser.add_argument('--send-multiple-ppmpdo', nargs='+', type=int, metavar='POSITION', help='Send multiple PPM PDO messages with target positions')
    parser.add_argument('--wait-time', type=float, default=1, help='Wait time (in seconds) between sending messages')

    args = parser.parse_args()

    commands = Commands(args.host, args.port)

    if args.send_ppmpdo is not None:
        print(f"Sending PPM PDO message to position {args.send_ppmpdo}")
        commands.send_ppmpdo_message(args.send_ppmpdo)

    if args.send_homingpdo:
        print("Sending Homing PDO message")
        commands.send_homingpdo_message()

    if args.send_multiple_ppmpdo:
        # Convert the input list of positions and send them
        target_positions = args.send_multiple_ppmpdo
        print(f"Sending multiple PPM PDO messages with positions: {target_positions} and wait time {args.wait_time}s")
        commands.send_multiple_ppmpdo_messages(target_positions, args.wait_time)

if __name__ == '__main__':
    main()
