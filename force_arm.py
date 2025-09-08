#!/usr/bin/env python3
from pymavlink import mavutil
import time

conn = mavutil.mavlink_connection('COM3', baud=9600)
conn.wait_heartbeat()
print(f"Connected: System {conn.target_system}")

def request_streams():
    streams = [
        (mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS, 2),
        (mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 2),
        (mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 2),
        (mavutil.mavlink.MAV_DATA_STREAM_POSITION, 2),
        (mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 4),
        (mavutil.mavlink.MAV_DATA_STREAM_EXTRA2, 4),
        (mavutil.mavlink.MAV_DATA_STREAM_EXTRA3, 2),
    ]
    
    for stream_id, rate in streams:
        conn.mav.request_data_stream_send(
            conn.target_system,
            conn.target_component,
            stream_id,
            rate,
            1
        )
    print("Telemetry streams requested")

request_streams()
time.sleep(2)

print("\nChecking frame parameters:")
for param in ['FRAME_CLASS', 'FRAME_TYPE', 'ARMING_CHECK']:
    conn.mav.param_request_read_send(
        conn.target_system,
        conn.target_component,
        param.encode('utf-8'),
        -1
    )
    msg = conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
    if msg:
        print(f"{param}: {msg.param_value}")

print("\nForce ARM with bypass:")
conn.mav.command_long_send(
    conn.target_system,
    conn.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 2989, 0, 0, 0, 0, 0
)

ack = conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
if ack and ack.result == 0:
    print("ARMED")
else:
    print(f"Failed: {ack.result if ack else 'timeout'}")

conn.close()
