from pymavlink import mavutil
from pymavlink.mavutil import mavlink_connection
from typing import NoReturn

PORT: str = 'COM3'
BAUDRATE: int = 115200


def main() -> NoReturn:
    try:
        print(f"Connecting to {PORT} at {BAUDRATE} baud...")
        connection: mavlink_connection = mavutil.mavlink_connection(PORT, baud=BAUDRATE)
        
        print("Waiting for HEARTBEAT from flight controller...")
        connection.wait_heartbeat()
        print(f"Connection successful! System: {connection.target_system}, Component: {connection.target_component}")
        
        print("\nReceiving messages (Press Ctrl+C to stop)...")
        while True:
            msg = connection.recv_match(blocking=True)
            if msg:
                print(f"Message type: {msg.get_type()}")
                
    except KeyboardInterrupt:
        print("\nConnection closed by user")
    except Exception as e:
        print(f"Error: {e}")
        print("\nTroubleshooting tips:")
        print("1. Check if the correct COM port is specified")
        print("2. Verify the baud rate matches your controller settings")
        print("3. Ensure the USB cable is properly connected")
        print("4. Check if another program is using the port")


if __name__ == "__main__":
    main()
