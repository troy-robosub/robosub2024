import socket
import json
import argparse

def connect_to_dvl(ip, port):
    """Connect to the DVL using TCP."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((ip, port))
        s.settimeout(1)
        return s
    except socket.error as err:
        print(f"Error connecting to DVL: {err}")
        return None

def send_command(s, command):
    """Send a command to the DVL and receive the response."""
    try:
        # Convert command to JSON string and send it
        s.sendall((json.dumps(command) + "\n").encode('utf-8'))
        
        # Receive the response
        response = s.recv(4096).decode('utf-8')
        
        # Convert response to JSON
        response_json = json.loads(response)
        return response_json
    except socket.error as err:
        print(f"Error sending command or receiving response: {err}")
        return None

def main():
    parser = argparse.ArgumentParser(description="Send command to DVL")
    parser.add_argument("-c", "--command", required=True, help="Command to send to the DVL")
    args = parser.parse_args()

    # DVL connection parameters
    DVL_IP = '192.168.137.101'  # Update this with your DVL's IP address
    DVL_PORT = 16171

    # Connect to the DVL
    dvl_socket = connect_to_dvl(DVL_IP, DVL_PORT)
    if not dvl_socket:
        print("Failed to connect to DVL")
        return

    # Example command to send
    command = {"command": args.command}
    
    # Send the command and get the response
    response = send_command(dvl_socket, command)
    
    # Print the response
    if response:
        print("Response from DVL:")
        print(json.dumps(response, indent=2))

    # Close the socket connection
    dvl_socket.close()

if __name__ == "__main__":
    main()
