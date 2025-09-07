# Flight Controller Testing Guide

## Prerequisites Check

```bash
python --version
pip list | grep pymavlink
ls /dev/tty* | grep -E "(USB|ACM)"
```

## Phase 1: Connection Test

### Step 1.1: Identify Port
```bash
# Windows
mode | findstr COM

# Linux
dmesg | grep tty
ls -la /dev/ttyUSB*
```

### Step 1.2: Basic Connection
```bash
cd E:\data\UAV\bort\lab5\Co-Processor
python
```
```python
from pymavlink import mavutil
conn = mavutil.mavlink_connection('COM3', baud=9600)
conn.wait_heartbeat()
print(f"Connected: {conn.target_system}")
exit()
```

Expected: Heartbeat received, system ID displayed

## Phase 2: Telemetry Verification

### Step 2.1: Run Test Script
```bash
python test_connection.py
```

Expected output:
```
Connecting to COM3 at 9600 baud...
Waiting for HEARTBEAT from flight controller...
Connection successful! System: 1, Component: 1
Message type: ATTITUDE
Message type: VFR_HUD
```

Press Ctrl+C after 10 messages

### Step 2.2: Test Telemetry Plotter
```bash
python telemetry_plotter.py
```

Verify:
- [ ] Graphs appear
- [ ] Data updates in real-time
- [ ] Message counters increment
- [ ] No shape mismatch errors

Close window after 30 seconds

## Phase 3: Command Testing

### Step 3.1: Launch Controller
```bash
python flight_controller.py
```

Expected:
```
Connecting to COM3 at 9600 baud...
Connected: System 1, Component 1
Logging to: logs/telemetry_20250107_143022.csv
Telemetry logging started

Flight Controller Ready
Commands: arm, disarm, pitch, mode, help, quit

>
```

### Step 3.2: Test Mode Query
```
> mode
Mode name: STABILIZE
```

Expected: Mode change acknowledgment

### Step 3.3: Test Disarm (Safe)
```
> disarm
```

Expected: 
```
Disarming...
Disarmed successfully
```

### Step 3.4: Test Pitch Override
```
> pitch
PWM (1000-2000, 1500=neutral, 0=release): 1500
```

Expected:
```
Pitch set to PWM: 1500
```

### Step 3.5: Verify Logging
Open new terminal:
```bash
tail -f logs/telemetry_*.csv
```

Expected: CSV data updating every second

### Step 3.6: Exit
```
> quit
```

## Phase 4: Autostart Testing (Linux/WSL)

### Step 4.1: Setup Environment
```bash
cd ~/lab5/Co-Processor
python3 -m venv lab_5
source lab_5/bin/activate
pip install -r requirements.txt
```

### Step 4.2: Configure Script
```bash
nano start_controller.sh
```

Update paths:
```bash
SCRIPT_DIR="/home/$USER/lab5/Co-Processor"
VENV_PATH="$SCRIPT_DIR/lab_5"
PYTHON_SCRIPT="$SCRIPT_DIR/flight_controller.py"
```

### Step 4.3: Test Manual Start
```bash
chmod +x start_controller.sh
./start_controller.sh
```

Verify:
```bash
tail -f logs/startup.log
ps aux | grep flight_controller
cat flight_controller.pid
```

### Step 4.4: Kill Test Process
```bash
kill $(cat flight_controller.pid)
```

### Step 4.5: Configure Crontab
```bash
crontab -e
```

Add:
```
@reboot /home/$USER/lab5/Co-Processor/start_controller.sh >> /home/$USER/lab5/logs/cron.log 2>&1
```

### Step 4.6: Verify Crontab
```bash
crontab -l
```

### Step 4.7: Test Reboot
```bash
sudo reboot
```

After reboot:
```bash
ps aux | grep flight_controller
tail -f ~/lab5/logs/cron.log
ls -la ~/lab5/logs/telemetry_*.csv
```

## Phase 5: Validation Checklist

### Connection
- [ ] Port identified correctly
- [ ] Heartbeat received
- [ ] No timeout errors

### Telemetry
- [ ] ATTITUDE messages received
- [ ] VFR_HUD messages received
- [ ] CSV file created
- [ ] Data logged every second

### Commands
- [ ] Disarm works
- [ ] Mode change acknowledged
- [ ] Pitch PWM accepted
- [ ] Commands show in logs

### Autostart
- [ ] Script executable
- [ ] Manual start works
- [ ] PID file created
- [ ] Crontab entry added
- [ ] Starts after reboot
- [ ] Logs redirect properly

## Troubleshooting

### No Connection
```bash
sudo chmod 666 /dev/ttyUSB0
sudo adduser $USER dialout
logout && login
```

### Wrong Baudrate
Try: 57600, 115200, 9600

### No Messages
```python
msg = conn.recv_match()
print(msg)
```

### Crontab Not Running
```bash
systemctl status cron
journalctl -u cron -n 50
```

### Process Already Running
```bash
pkill -f flight_controller
rm flight_controller.pid
```

## Performance Metrics

Monitor during test:
```bash
htop -p $(cat flight_controller.pid)
du -sh logs/
iostat -x 1
```

Expected:
- CPU < 5%
- Memory < 50MB
- Log growth < 1MB/hour
