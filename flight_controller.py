from __future__ import annotations
import time
import threading
import csv
from dataclasses import dataclass, field
from typing import Optional, TextIO
from datetime import datetime
from pathlib import Path
from pymavlink import mavutil
from pymavlink.mavutil import mavlink_connection


@dataclass
class ControllerConfig:
    port: str = 'COM3'
    baudrate: int = 9600
    log_dir: Path = field(default_factory=lambda: Path('logs'))
    log_interval: float = 1.0


class FlightController:
    def __init__(self, config: ControllerConfig):
        self.config = config
        self.connection: Optional[mavlink_connection] = None
        self.csv_file: Optional[TextIO] = None
        self.csv_writer: Optional[csv.DictWriter] = None
        self.running: bool = False
        self._lock = threading.Lock()
        self._setup_logging()

    def _setup_logging(self) -> None:
        self.config.log_dir.mkdir(exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        csv_path = self.config.log_dir / f'telemetry_{timestamp}.csv'
        
        self.csv_file = open(csv_path, 'w', newline='')
        fieldnames = ['timestamp', 'roll', 'pitch', 'yaw', 'altitude', 
                     'ground_speed', 'battery_voltage', 'gps_lat', 'gps_lon', 'mode']
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=fieldnames)
        self.csv_writer.writeheader()
        print(f"Logging to: {csv_path}")

    def connect(self) -> bool:
        try:
            print(f"Connecting to {self.config.port} at {self.config.baudrate} baud...")
            self.connection = mavutil.mavlink_connection(
                self.config.port, 
                baud=self.config.baudrate
            )
            self.connection.wait_heartbeat()
            print(f"Connected: System {self.connection.target_system}, "
                  f"Component {self.connection.target_component}")
            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            return False

    def arm(self) -> None:
        if not self.connection:
            print("Not connected")
            return
        
        print("Arming...")
        self.connection.arducopter_arm()
        msg = self.connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if msg and msg.result == 0:
            print("Armed successfully")
        else:
            print(f"Arm failed: {msg.result if msg else 'timeout'}")

    def disarm(self) -> None:
        if not self.connection:
            print("Not connected")
            return
        
        print("Disarming...")
        self.connection.arducopter_disarm()
        msg = self.connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if msg and msg.result == 0:
            print("Disarmed successfully")
        else:
            print(f"Disarm failed: {msg.result if msg else 'timeout'}")

    def set_mode(self, mode_name: str) -> None:
        if not self.connection:
            print("Not connected")
            return
        
        mode_mapping = self.connection.mode_mapping()
        if mode_name not in mode_mapping:
            print(f"Unknown mode: {mode_name}")
            print(f"Available: {', '.join(mode_mapping.keys())}")
            return
        
        mode_id = mode_mapping[mode_name]
        print(f"Setting mode to {mode_name} (ID: {mode_id})")
        self.connection.set_mode(mode_id)
        
        msg = self.connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if msg and msg.result == 0:
            print(f"Mode set to {mode_name}")
        else:
            print(f"Mode change failed: {msg.result if msg else 'timeout'}")

    def set_pitch(self, pwm: int) -> None:
        if not self.connection:
            print("Not connected")
            return
        
        if not (1000 <= pwm <= 2000 or pwm == 0):
            print(f"Invalid PWM: {pwm} (must be 1000-2000 or 0)")
            return
        
        self.connection.mav.rc_channels_override_send(
            self.connection.target_system,
            self.connection.target_component,
            0, pwm, 0, 0, 0, 0, 0, 0
        )
        print(f"Pitch set to PWM: {pwm}")

    def start_telemetry_logging(self) -> None:
        self.running = True
        thread = threading.Thread(target=self._log_telemetry, daemon=True)
        thread.start()
        print("Telemetry logging started")

    def _log_telemetry(self) -> None:
        data = {}
        
        while self.running and self.connection:
            msg = self.connection.recv_match(blocking=False)
            if msg:
                msg_type = msg.get_type()
                
                if msg_type == 'ATTITUDE':
                    data['roll'] = round(msg.roll, 4)
                    data['pitch'] = round(msg.pitch, 4)
                    data['yaw'] = round(msg.yaw, 4)
                
                elif msg_type == 'VFR_HUD':
                    data['altitude'] = round(msg.alt, 2)
                    data['ground_speed'] = round(msg.groundspeed, 2)
                
                elif msg_type == 'BATTERY_STATUS':
                    if msg.voltages[0] not in (65535, -1):
                        data['battery_voltage'] = round(msg.voltages[0] / 1000.0, 2)
                
                elif msg_type == 'GLOBAL_POSITION_INT':
                    data['gps_lat'] = round(msg.lat / 1e7, 6)
                    data['gps_lon'] = round(msg.lon / 1e7, 6)
                
                elif msg_type == 'HEARTBEAT':
                    data['mode'] = mavutil.mode_string_v10(msg)
            
            if time.time() % self.config.log_interval < 0.01 and data:
                with self._lock:
                    data['timestamp'] = datetime.now().isoformat()
                    self.csv_writer.writerow(data)
                    self.csv_file.flush()
            
            time.sleep(0.001)

    def stop(self) -> None:
        self.running = False
        if self.csv_file:
            self.csv_file.close()
        if self.connection:
            self.connection.close()
        print("Controller stopped")


def main() -> None:
    config = ControllerConfig(
        port='COM3',
        baudrate=9600,
        log_dir=Path('logs')
    )
    
    controller = FlightController(config)
    
    if not controller.connect():
        return
    
    controller.start_telemetry_logging()
    
    commands = {
        'arm': controller.arm,
        'disarm': controller.disarm,
        'pitch': lambda: controller.set_pitch(
            int(input("PWM (1000-2000, 1500=neutral, 0=release): "))
        ),
        'mode': lambda: controller.set_mode(
            input("Mode name: ").upper()
        ),
        'help': lambda: print("Commands: arm, disarm, pitch, mode, quit"),
        'quit': lambda: None
    }
    
    print("\nFlight Controller Ready")
    print("Commands: arm, disarm, pitch, mode, help, quit")
    
    try:
        while True:
            cmd = input("\n> ").strip().lower()
            if cmd == 'quit':
                break
            
            action = commands.get(cmd)
            if action:
                action()
            else:
                print(f"Unknown command: {cmd}")
    
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        controller.stop()


if __name__ == "__main__":
    main()
