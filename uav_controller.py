from __future__ import annotations
import time
import threading
import csv
from dataclasses import dataclass, field
from typing import Optional, TextIO
from collections import deque
from datetime import datetime
from pathlib import Path
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from pymavlink import mavutil
from pymavlink.mavutil import mavlink_connection


@dataclass
class Config:
    port: str = 'COM3'
    baudrate: int = 9600
    log_dir: Path = field(default_factory=lambda: Path('logs'))
    log_interval: float = 1.0
    plot_window: int = 200
    update_interval: int = 100


class UAVController:
    def __init__(self, config: Config):
        self.config = config
        self.connection: Optional[mavlink_connection] = None
        self.csv_file: Optional[TextIO] = None
        self.csv_writer: Optional[csv.DictWriter] = None
        self.running: bool = False
        self._lock = threading.Lock()
        self.telemetry_data = {
            'roll': deque(maxlen=config.plot_window),
            'pitch': deque(maxlen=config.plot_window),
            'yaw': deque(maxlen=config.plot_window),
            'altitude': deque(maxlen=config.plot_window),
            'ground_speed': deque(maxlen=config.plot_window),
            'battery_voltage': deque(maxlen=config.plot_window),
            'time': deque(maxlen=config.plot_window)
        }
        self.start_time = time.time()
        self._setup_logging()

    def _setup_logging(self) -> None:
        self.config.log_dir.mkdir(exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        csv_path = self.config.log_dir / f'telemetry_{timestamp}.csv'
        
        self.csv_file = open(csv_path, 'w', newline='')
        fieldnames = ['timestamp', 'roll', 'pitch', 'yaw', 'altitude', 
                     'ground_speed', 'battery_voltage', 'mode']
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
            print(f"Connected: System {self.connection.target_system}, Component {self.connection.target_component}")
            self._request_streams()
            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            return False

    def _request_streams(self) -> None:
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
            self.connection.mav.request_data_stream_send(
                self.connection.target_system,
                self.connection.target_component,
                stream_id, rate, 1
            )

    def arm(self) -> None:
        if not self.connection:
            return
        print("Arming...")
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 2989, 0, 0, 0, 0, 0
        )
        ack = self.connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=30)
        if ack and ack.result == 0:
            print("Armed successfully")
        else:
            print(f"Arm failed: {ack.result if ack else 'timeout'}")

    def disarm(self) -> None:
        if not self.connection:
            return
        print("Disarming...")
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        ack = self.connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if ack and ack.result == 0:
            print("Disarmed successfully")
        else:
            print(f"Disarm failed: {ack.result if ack else 'timeout'}")

    def set_mode(self, mode_name: str) -> None:
        if not self.connection:
            return
        mode_mapping = self.connection.mode_mapping()
        if mode_name not in mode_mapping:
            print(f"Available modes: {', '.join(mode_mapping.keys())}")
            return
        mode_id = mode_mapping[mode_name]
        print(f"Setting mode to {mode_name}")
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id, 0, 0, 0, 0, 0
        )
        ack = self.connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if ack and ack.result == 0:
            print(f"Mode set to {mode_name}")

    def set_pitch(self, pwm: int) -> None:
        if not self.connection:
            return
        if not (1000 <= pwm <= 2000 or pwm == 0):
            print(f"Invalid PWM: {pwm}")
            return
        self.connection.mav.rc_channels_override_send(
            self.connection.target_system,
            self.connection.target_component,
            0, pwm, 0, 0, 0, 0, 0, 0
        )
        print(f"Pitch set to PWM: {pwm}")

    def status(self) -> None:
        if not self.connection:
            return
        msg = self.connection.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg:
            mode = mavutil.mode_string_v10(msg)
            armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            print(f"Mode: {mode}, Armed: {'Yes' if armed else 'No'}")

    def start_telemetry(self) -> None:
        self.running = True
        thread = threading.Thread(target=self._telemetry_loop, daemon=True)
        thread.start()

    def _telemetry_loop(self) -> None:
        csv_data = {}
        last_log = time.time()
        
        while self.running and self.connection:
            msg = self.connection.recv_match(blocking=False)
            if msg:
                current_time = time.time() - self.start_time
                msg_type = msg.get_type()
                
                with self._lock:
                    if msg_type == 'ATTITUDE':
                        csv_data['roll'] = round(msg.roll, 4)
                        csv_data['pitch'] = round(msg.pitch, 4)
                        csv_data['yaw'] = round(msg.yaw, 4)
                        self.telemetry_data['roll'].append(msg.roll)
                        self.telemetry_data['pitch'].append(msg.pitch)
                        self.telemetry_data['yaw'].append(msg.yaw)
                        self.telemetry_data['time'].append(current_time)
                    
                    elif msg_type == 'VFR_HUD':
                        csv_data['altitude'] = round(msg.alt, 2)
                        csv_data['ground_speed'] = round(msg.groundspeed, 2)
                        self.telemetry_data['altitude'].append(msg.alt)
                        self.telemetry_data['ground_speed'].append(msg.groundspeed)
                    
                    elif msg_type == 'BATTERY_STATUS':
                        if msg.voltages[0] not in (65535, -1):
                            voltage = round(msg.voltages[0] / 1000.0, 2)
                            csv_data['battery_voltage'] = voltage
                            self.telemetry_data['battery_voltage'].append(voltage)
                    
                    elif msg_type == 'HEARTBEAT':
                        csv_data['mode'] = mavutil.mode_string_v10(msg)
                
                if time.time() - last_log >= self.config.log_interval and csv_data:
                    csv_data['timestamp'] = datetime.now().isoformat()
                    self.csv_writer.writerow(csv_data)
                    self.csv_file.flush()
                    last_log = time.time()
            
            time.sleep(0.001)

    def plot_telemetry(self) -> None:
        fig, axes = plt.subplots(2, 2, figsize=(12, 8))
        ax_attitude, ax_altitude, ax_speed, ax_battery = axes.flatten()
        
        lines = {
            'roll': ax_attitude.plot([], [], 'r-', label='Roll')[0],
            'pitch': ax_attitude.plot([], [], 'g-', label='Pitch')[0],
            'yaw': ax_attitude.plot([], [], 'b-', label='Yaw')[0],
            'altitude': ax_altitude.plot([], [], 'm-', label='Altitude')[0],
            'speed': ax_speed.plot([], [], 'c-', label='Ground Speed')[0],
            'battery': ax_battery.plot([], [], 'orange', label='Battery')[0]
        }
        
        for ax, title, ylabel in [
            (ax_attitude, 'Attitude', 'Radians'),
            (ax_altitude, 'Altitude', 'Meters'),
            (ax_speed, 'Ground Speed', 'm/s'),
            (ax_battery, 'Battery Voltage', 'Volts')
        ]:
            ax.set_title(title)
            ax.set_xlabel('Time (s)')
            ax.set_ylabel(ylabel)
            ax.legend()
            ax.grid(True)
        
        ax_attitude.set_ylim(-3.5, 3.5)
        
        def update(frame):
            with self._lock:
                if self.telemetry_data['time']:
                    time_data = list(self.telemetry_data['time'])
                    
                    for ax in [ax_attitude, ax_altitude, ax_speed, ax_battery]:
                        if time_data:
                            ax.set_xlim(max(0, time_data[-1] - 10), time_data[-1] + 1)
                    
                    lines['roll'].set_data(time_data, list(self.telemetry_data['roll']))
                    lines['pitch'].set_data(time_data, list(self.telemetry_data['pitch']))
                    lines['yaw'].set_data(time_data, list(self.telemetry_data['yaw']))
                    
                    if self.telemetry_data['altitude']:
                        lines['altitude'].set_data(time_data[:len(self.telemetry_data['altitude'])], 
                                                 list(self.telemetry_data['altitude']))
                        if len(self.telemetry_data['altitude']) > 1:
                            ax_altitude.relim()
                            ax_altitude.autoscale_view()
                    
                    if self.telemetry_data['ground_speed']:
                        lines['speed'].set_data(time_data[:len(self.telemetry_data['ground_speed'])], 
                                              list(self.telemetry_data['ground_speed']))
                        ax_speed.relim()
                        ax_speed.autoscale_view()
                    
                    if self.telemetry_data['battery_voltage']:
                        lines['battery'].set_data(time_data[:len(self.telemetry_data['battery_voltage'])], 
                                                list(self.telemetry_data['battery_voltage']))
                        ax_battery.relim()
                        ax_battery.autoscale_view()
            
            return lines.values()
        
        ani = animation.FuncAnimation(fig, update, interval=self.config.update_interval, 
                                     blit=False, cache_frame_data=False)
        plt.tight_layout()
        plt.show()

    def stop(self) -> None:
        self.running = False
        if self.csv_file:
            self.csv_file.close()
        if self.connection:
            self.connection.close()


def main():
    config = Config()
    controller = UAVController(config)
    
    if not controller.connect():
        return
    
    controller.start_telemetry()
    
    commands = {
        'arm': controller.arm,
        'disarm': controller.disarm,
        'status': controller.status,
        'pitch': lambda: controller.set_pitch(int(input("PWM (1000-2000, 0=release): "))),
        'mode': lambda: controller.set_mode(input("Mode: ").upper()),
        'plot': controller.plot_telemetry,
        'help': lambda: print("arm, disarm, status, pitch, mode, plot, quit")
    }
    
    print("\nUAV Controller Ready")
    print("Commands: arm, disarm, status, pitch, mode, plot, quit")
    
    try:
        while True:
            cmd = input("\n> ").strip().lower()
            if cmd == 'quit':
                break
            action = commands.get(cmd)
            if action:
                action()
            else:
                print(f"Unknown: {cmd}")
    except KeyboardInterrupt:
        pass
    finally:
        controller.stop()


if __name__ == "__main__":
    main()
