import time
import threading
from pymavlink import mavutil
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque


master = mavutil.mavlink_connection('COM3', baud=9600)
master.wait_heartbeat()
print("Heartbeat received")

max_len = 200
roll_data = deque(maxlen=max_len)
pitch_data = deque(maxlen=max_len)
yaw_data = deque(maxlen=max_len)
time_data = deque(maxlen=max_len)

start_time = time.time()


def read_attitude():
    while True:
        msg = master.recv_match(type='ATTITUDE', blocking=True)
        if msg:
            t = time.time() - start_time
            roll_data.append(msg.roll)
            pitch_data.append(msg.pitch)
            yaw_data.append(msg.yaw)
            time_data.append(t)


thread = threading.Thread(target=read_attitude)
thread.daemon = True
thread.start()

fig, ax = plt.subplots()
line_roll, = ax.plot([], [], label='Roll')
line_pitch, = ax.plot([], [], label='Pitch')
line_yaw, = ax.plot([], [], label='Yaw')

ax.set_ylim(-3.5, 3.5)
ax.set_xlim(0, max_len)
ax.set_xlabel('Time (s)')
ax.set_ylabel('Radians')
ax.legend()
ax.grid(True)


def update(frame):
    if not time_data:
        return line_roll, line_pitch, line_yaw
    ax.set_xlim(max(0, time_data[0]), time_data[-1])
    line_roll.set_data(time_data, roll_data)
    line_pitch.set_data(time_data, pitch_data)
    line_yaw.set_data(time_data, yaw_data)
    return line_roll, line_pitch, line_yaw


ani = animation.FuncAnimation(fig, update, interval=100, cache_frame_data=False)
plt.tight_layout()
plt.show()
