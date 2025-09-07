from __future__ import annotations
import time
import threading
from dataclasses import dataclass, field
from typing import Optional
from collections import deque
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from pymavlink import mavutil
from pymavlink.mavutil import mavlink_connection


@dataclass
class TelemetryConfig:
    port: str = 'COM3'
    baudrate: int = 9600
    max_data_points: int = 200
    update_interval: int = 100
    time_window: float = 10.0


@dataclass
class TelemetryData:
    roll: deque[float] = field(default_factory=lambda: deque(maxlen=200))
    pitch: deque[float] = field(default_factory=lambda: deque(maxlen=200))
    yaw: deque[float] = field(default_factory=lambda: deque(maxlen=200))
    attitude_time: deque[float] = field(default_factory=lambda: deque(maxlen=200))
    
    altitude: deque[float] = field(default_factory=lambda: deque(maxlen=200))
    ground_speed: deque[float] = field(default_factory=lambda: deque(maxlen=200))
    vfr_time: deque[float] = field(default_factory=lambda: deque(maxlen=200))
    
    battery_voltage: deque[float] = field(default_factory=lambda: deque(maxlen=200))
    battery_time: deque[float] = field(default_factory=lambda: deque(maxlen=200))
    
    gps_lat: deque[float] = field(default_factory=lambda: deque(maxlen=200))
    gps_lon: deque[float] = field(default_factory=lambda: deque(maxlen=200))
    gps_alt: deque[float] = field(default_factory=lambda: deque(maxlen=200))
    gps_time: deque[float] = field(default_factory=lambda: deque(maxlen=200))
    
    attitude_count: int = 0
    vfr_hud_count: int = 0
    gps_count: int = 0
    battery_count: int = 0
    total_messages: int = 0


class TelemetryReader:
    def __init__(self, config: TelemetryConfig):
        self.config = config
        self.data = TelemetryData()
        self.connection: Optional[mavlink_connection] = None
        self.start_time: float = time.time()
        self.running: bool = False
        self._lock = threading.Lock()

    def connect(self) -> bool:
        try:
            print(f"Connecting to {self.config.port} at {self.config.baudrate} baud...")
            self.connection = mavutil.mavlink_connection(
                self.config.port, 
                baud=self.config.baudrate
            )
            self.connection.wait_heartbeat()
            print(f"Heartbeat received! System: {self.connection.target_system}, "
                  f"Component: {self.connection.target_component}")
            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            return False

    def start_reading(self) -> None:
        self.running = True
        self.start_time = time.time()
        thread = threading.Thread(target=self._read_all_telemetry, daemon=True)
        thread.start()

    def stop_reading(self) -> None:
        self.running = False
        if self.connection:
            self.connection.close()

    def _read_all_telemetry(self) -> None:
        while self.running and self.connection:
            msg = self.connection.recv_match(blocking=True, timeout=0.1)
            if msg:
                current_time = time.time() - self.start_time
                msg_type = msg.get_type()
                
                with self._lock:
                    self.data.total_messages += 1
                    
                    if msg_type == 'ATTITUDE':
                        self.data.attitude_count += 1
                        self.data.roll.append(msg.roll)
                        self.data.pitch.append(msg.pitch)
                        self.data.yaw.append(msg.yaw)
                        self.data.attitude_time.append(current_time)
                    
                    elif msg_type == 'VFR_HUD':
                        self.data.vfr_hud_count += 1
                        self.data.altitude.append(msg.alt)
                        self.data.ground_speed.append(msg.groundspeed)
                        self.data.vfr_time.append(current_time)
                    
                    elif msg_type == 'GLOBAL_POSITION_INT':
                        self.data.gps_count += 1
                        self.data.gps_lat.append(msg.lat / 1e7)
                        self.data.gps_lon.append(msg.lon / 1e7)
                        self.data.gps_alt.append(msg.alt / 1000)
                        self.data.gps_time.append(current_time)
                        if not self.data.altitude:
                            self.data.altitude.append(msg.relative_alt / 1000)
                            self.data.vfr_time.append(current_time)
                    
                    elif msg_type == 'BATTERY_STATUS':
                        self.data.battery_count += 1
                        if msg.voltages[0] != 65535 and msg.voltages[0] != -1:
                            self.data.battery_voltage.append(msg.voltages[0] / 1000.0)
                            self.data.battery_time.append(current_time)
                    
                    elif msg_type == 'SYS_STATUS':
                        if hasattr(msg, 'voltage_battery') and msg.voltage_battery != 65535:
                            self.data.battery_voltage.append(msg.voltage_battery / 1000.0)
                            self.data.battery_time.append(current_time)


class TelemetryPlotter:
    def __init__(self, reader: TelemetryReader):
        self.reader = reader
        self.fig, (self.ax_attitude, self.ax_alt_speed, self.ax_battery) = plt.subplots(
            3, 1, figsize=(12, 10)
        )
        self.lines = {}
        self._setup_plots()

    def _setup_plots(self) -> None:
        self.fig.suptitle("ArduPilot Telemetry", fontsize=14, fontweight='bold')
        
        self.lines['roll'], = self.ax_attitude.plot([], [], 'r-', label='Roll', linewidth=1.5)
        self.lines['pitch'], = self.ax_attitude.plot([], [], 'g-', label='Pitch', linewidth=1.5)
        self.lines['yaw'], = self.ax_attitude.plot([], [], 'b-', label='Yaw', linewidth=1.5)
        self.ax_attitude.set_ylim(-3.5, 3.5)
        self.ax_attitude.set_ylabel('Radians')
        self.ax_attitude.legend(loc='upper right')
        self.ax_attitude.grid(True, alpha=0.3)
        self.ax_attitude.set_title('Attitude (Roll, Pitch, Yaw)')
        
        self.ax_alt_speed_2 = self.ax_alt_speed.twinx()
        self.lines['altitude'], = self.ax_alt_speed.plot([], [], 'g-', label='Altitude (m)', linewidth=1.5)
        self.lines['ground_speed'], = self.ax_alt_speed_2.plot([], [], 'r--', label='Ground Speed (m/s)', linewidth=1.5)
        self.ax_alt_speed.set_ylabel('Altitude (m)', color='g')
        self.ax_alt_speed_2.set_ylabel('Ground Speed (m/s)', color='r')
        self.ax_alt_speed.tick_params(axis='y', labelcolor='g')
        self.ax_alt_speed_2.tick_params(axis='y', labelcolor='r')
        self.ax_alt_speed.grid(True, alpha=0.3)
        self.ax_alt_speed.set_title('Altitude & Ground Speed')
        lines1, labels1 = self.ax_alt_speed.get_legend_handles_labels()
        lines2, labels2 = self.ax_alt_speed_2.get_legend_handles_labels()
        self.ax_alt_speed.legend(lines1 + lines2, labels1 + labels2, loc='upper right')
        
        self.lines['battery'], = self.ax_battery.plot([], [], 'm-', label='Battery Voltage (V)', linewidth=1.5)
        self.ax_battery.set_ylabel('Voltage (V)')
        self.ax_battery.set_xlabel('Time (s)')
        self.ax_battery.legend(loc='upper right')
        self.ax_battery.grid(True, alpha=0.3)
        self.ax_battery.set_title('Battery Voltage')
        
        plt.tight_layout()

    def update(self, frame: int) -> tuple:
        with self.reader._lock:
            data = self.reader.data
            current_time = time.time() - self.reader.start_time
            time_window = self.reader.config.time_window
            x_min = max(0, current_time - time_window)
            x_max = current_time
            
            for ax in [self.ax_attitude, self.ax_alt_speed, self.ax_battery]:
                ax.set_xlim(x_min, x_max)
            
            if data.attitude_time and data.roll:
                self.lines['roll'].set_data(list(data.attitude_time), list(data.roll))
                self.lines['pitch'].set_data(list(data.attitude_time), list(data.pitch))
                self.lines['yaw'].set_data(list(data.attitude_time), list(data.yaw))
            
            if data.vfr_time and data.altitude:
                self.lines['altitude'].set_data(list(data.vfr_time), list(data.altitude))
                if len(data.altitude) > 1:
                    alt_range = max(data.altitude) - min(data.altitude)
                    alt_center = (max(data.altitude) + min(data.altitude)) / 2
                    self.ax_alt_speed.set_ylim(alt_center - alt_range - 5, alt_center + alt_range + 5)
            
            if data.vfr_time and data.ground_speed:
                self.lines['ground_speed'].set_data(list(data.vfr_time), list(data.ground_speed))
                if data.ground_speed:
                    max_speed = max(data.ground_speed)
                    self.ax_alt_speed_2.set_ylim(0, max(10, max_speed * 1.2))
            
            if data.battery_time and data.battery_voltage:
                self.lines['battery'].set_data(list(data.battery_time), list(data.battery_voltage))
                if len(data.battery_voltage) > 1:
                    batt_min = min(data.battery_voltage)
                    batt_max = max(data.battery_voltage)
                    self.ax_battery.set_ylim(batt_min - 1, batt_max + 1)
            
            status = (f"Messages: {data.total_messages} | "
                     f"ATT: {data.attitude_count} | "
                     f"HUD: {data.vfr_hud_count} | "
                     f"GPS: {data.gps_count} | "
                     f"BAT: {data.battery_count}")
            
            self.fig.suptitle(f"ArduPilot Telemetry - {status}", fontsize=10)
        
        return tuple(self.lines.values())

    def start(self) -> None:
        ani = animation.FuncAnimation(
            self.fig,
            self.update,
            interval=self.reader.config.update_interval,
            blit=False,
            cache_frame_data=False
        )
        plt.show()


def main() -> None:
    config = TelemetryConfig(
        port='COM3',
        baudrate=9600,
        max_data_points=200,
        time_window=10.0
    )
    
    reader = TelemetryReader(config)
    
    if not reader.connect():
        print("Failed to establish connection")
        return
    
    try:
        reader.start_reading()
        time.sleep(0.5)
        
        print("Starting telemetry visualization...")
        print("Messages types: ATTITUDE, VFR_HUD, GLOBAL_POSITION_INT, BATTERY_STATUS")
        print("Close plot window to exit")
        
        plotter = TelemetryPlotter(reader)
        plotter.start()
        
    except KeyboardInterrupt:
        print("\nShutdown requested")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        reader.stop_reading()
        print(f"\nStatistics:")
        print(f"  Total messages: {reader.data.total_messages}")
        print(f"  ATTITUDE: {reader.data.attitude_count}")
        print(f"  VFR_HUD: {reader.data.vfr_hud_count}")
        print(f"  GPS: {reader.data.gps_count}")
        print(f"  BATTERY: {reader.data.battery_count}")


if __name__ == "__main__":
    main()
