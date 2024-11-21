import socket
import struct
import time

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

    def send_multiple_ppmpdo_messages(self, target_position1, target_position2, target_position3, wait_time):
        """Send PPM PDO messages for three target positions with wait times in between."""
        # Move to target position P1
        print(f"Moving to P1: {target_position1}")
        self.send_ppmpdo_message(target_position1)
        time.sleep(wait_time)  # Wait for specified time

        # Move to target position P2
        print(f"Moving to P2: {target_position2}")
        self.send_ppmpdo_message(target_position2)
        time.sleep(wait_time)  # Wait for specified time

        # Move to target position P3
        print(f"Moving to P3: {target_position3}")
        self.send_ppmpdo_message(target_position3)
        time.sleep(wait_time)  # Wait for specified time

    def _send_message(self, message):
        """Send a message to the server and optionally receive a response."""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
                client_socket.connect((self.host, self.port))  # Connect to the server
                client_socket.send(message)  # Send the message

                # Optionally, receive a response (e.g., success or error)
                response = client_socket.recv(1)
                if response:
                    print(f"Received response: {response}")
                else:
                    print("No response from server")
        except Exception as e:
            print(f"Error sending message: {e}")

# Usage example
if __name__ == '__main__':
    host = '131.215.193.25'  # Server IP address
    port = 7469  # Server port
    target_position1 = 1000  # Example target position for PPM PDO
    target_position2 = 2000  # Another example target position
    target_position3 = 3000  # Another example target position
    wait_time = 2  # Time to wait (in seconds) between each move

    # Create an instance of the Commands class
    commands = Commands(host, port)

    # Send multiple PPM PDO messages with wait times
    commands.send_multiple_ppmpdo_messages(target_position1, target_position2, target_position3, wait_time)

    # Send Homing PDO message (if needed)
    # commands.send_homingpdo_message()
