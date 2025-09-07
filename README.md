# Flight Controller Implementation Guide

## Files Created

1. **flight_controller.py** - Main control script with all features
2. **start_controller.sh** - Bash script for automatic startup
3. **crontab_config.txt** - Crontab configuration examples

## Features Implemented

### 1. ARM/DISARM Commands
```python
controller.arm()     # Arms the controller
controller.disarm()  # Disarms the controller
```

### 2. Automatic Telemetry Logging
- Logs to CSV file in `logs/` directory
- Filename: `telemetry_YYYYMMDD_HHMMSS.csv`
- Records: timestamp, roll, pitch, yaw, altitude, ground_speed, battery_voltage, GPS, mode
- Automatic flush every second

### 3. Flight Mode Changes
```python
controller.set_mode('GUIDED')   # Change to GUIDED mode
controller.set_mode('LOITER')   # Change to LOITER mode
controller.set_mode('STABILIZE') # Change to STABILIZE mode
```

### 4. Pitch Channel Control
```python
controller.set_pitch(1700)  # Forward pitch
controller.set_pitch(1500)  # Neutral
controller.set_pitch(1300)  # Backward pitch
controller.set_pitch(0)     # Release override
```

## Setup Instructions

### Windows (Development)

1. **Install dependencies:**
```bash
cd E:\data\UAV\bort\lab5\Co-Processor
pip install -r requirements.txt
```

2. **Run manually:**
```bash
python flight_controller.py
```

### Linux (Production/Autostart)

1. **Copy files to Linux system:**
```bash
scp -r E:\data\UAV\bort\lab5\Co-Processor user@linux-host:~/lab5/
```

2. **Make startup script executable:**
```bash
chmod +x ~/lab5/Co-Processor/start_controller.sh
```

3. **Edit paths in start_controller.sh:**
```bash
nano ~/lab5/Co-Processor/start_controller.sh
# Update SCRIPT_DIR, VENV_PATH, PYTHON_SCRIPT paths
```

4. **Configure crontab for autostart:**
```bash
crontab -e
# Add this line (adjust paths):
@reboot ~/lab5/Co-Processor/start_controller.sh >> ~/lab5/logs/cron.log 2>&1
```

5. **Verify crontab:**
```bash
crontab -l
```

## Usage

### Interactive Mode
```
Commands:
- arm      : Arm the controller
- disarm   : Disarm the controller  
- pitch    : Control pitch channel (enter PWM value)
- mode     : Change flight mode (enter mode name)
- help     : Show available commands
- quit     : Exit program
```

### Check Logs

**Telemetry logs:**
```bash
ls logs/telemetry_*.csv
tail -f logs/telemetry_*.csv
```

**Startup logs:**
```bash
tail -f logs/startup.log
tail -f logs/cron.log
```

**Process status:**
```bash
ps aux | grep flight_controller
cat flight_controller.pid
```

## Port Configuration

Update port settings in flight_controller.py:
```python
config = ControllerConfig(
    port='COM3',        # Windows: COM3, Linux: /dev/ttyUSB0
    baudrate=9600,      # Match your controller
    log_dir=Path('logs')
)
```

## Troubleshooting

1. **Permission denied on Linux:**
```bash
sudo usermod -a -G dialout $USER
sudo chmod 666 /dev/ttyUSB0
```

2. **Crontab not working:**
```bash
# Check cron service
sudo service cron status
sudo service cron start

# Check logs
grep CRON /var/log/syslog
```

3. **Virtual environment issues:**
```bash
# Recreate venv
python3 -m venv lab_5
source lab_5/bin/activate
pip install -r requirements.txt
```

4. **Kill running process:**
```bash
kill $(cat flight_controller.pid)
pkill -f flight_controller.py
```

## Testing Checklist

- [ ] Manual connection test
- [ ] ARM/DISARM commands work
- [ ] Mode changes confirmed
- [ ] Pitch control responsive
- [ ] CSV logging creates files
- [ ] Startup script runs manually
- [ ] Crontab executes on reboot
- [ ] Logs redirect properly (>> log.txt 2>&1)

## Safety Notes

- Remove propellers when testing ARM command
- Start in STABILIZE mode for safety
- Monitor battery voltage in logs
- Set up kill switch for emergency stop
