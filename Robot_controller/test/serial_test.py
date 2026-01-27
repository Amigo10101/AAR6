import serial
import struct
import time
import threading
import tkinter as tk
from tkinter import ttk

# Serial settings
SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 1500000
NUMBER_OF_JOINTS = 7

# Robot commands
CMD_NONE          = 0
CMD_ACK           = 1
CMD_START         = 2
CMD_ESTOP         = 3
CMD_ESTOP_RELEASE = 4
CMD_HOME          = 5
CMD_DEHOME        = 6
CMD_DISABLE       = 7

# Map robot states to human-readable names
ROBOT_STATES = {
    0: "ERROR",
    1: "WAIT_PC",
    2: "START",
    3: "ESTOP",
    4: "DISABLED",
    5: "HOMING",
    6: "HOMED",
    7: "STANDSTILL",
    8: "MOVING",
    9: "EXECUTING"
}

class RobotInterface:
    def __init__(self, port, baudrate):
        self.ser = serial.Serial(port, baudrate, timeout=0.001)
        self.lock = threading.Lock()
        self.latest_state = None
        self.running = True
        self.packet_count = 0
        self.thread = threading.Thread(target=self.read_loop, daemon=True)
        self.thread.start()

    def read_loop(self):
        while self.running:
            result = self.read_packet()
            if result:
                motors_data, robot_state = result
                self.packet_count += 1
                with self.lock:
                    self.latest_state = robot_state
                # Print first motor info to log
                if motors_data:
                    print(f"Motor0: pos={motors_data[0]['pos']:.3f}, "
                          f"vel={motors_data[0]['vel']:.3f}, "
                          f"Robot state: {ROBOT_STATES.get(robot_state, 'Unknown')}")
            time.sleep(0.002)

    def send_command(self, cmd):
        buf = bytearray()
        buf.append(0xAA)
        buf.append(0x55)
        buf += b'\x01\x00'  # payload length = 1
        buf.append(cmd)
        chk = cmd
        buf.append(chk)
        self.ser.write(buf)
        print(f"Sent robot command: {cmd}")

    def read_packet(self):
        """Read a single packet from MCU"""
        try:
            while True:
                b = self.ser.read(1)
                if b == b'\xAA':
                    b2 = self.ser.read(1)
                    if b2 == b'\x55':
                        break
            length_bytes = self.ser.read(2)
            if len(length_bytes) < 2:
                return None
            payload_len = length_bytes[0] | (length_bytes[1] << 8)
            payload = self.ser.read(payload_len)
            if len(payload) < payload_len:
                return None
            checksum = self.ser.read(1)
            if not checksum:
                return None
            chk = 0
            for byte in payload:
                chk ^= byte
            if chk != checksum[0]:
                return None

            motors = []
            for i in range(NUMBER_OF_JOINTS):
                offset = i * 12
                if offset + 12 <= len(payload):
                    pos, vel, acc = struct.unpack('<fff', payload[offset:offset+12])
                    motors.append({'pos': pos, 'vel': vel, 'acc': acc})

            robot_state = payload[NUMBER_OF_JOINTS*12] if payload_len > NUMBER_OF_JOINTS*12 else None
            return motors, robot_state
        except Exception:
            return None

    def stop(self):
        self.running = False
        self.thread.join()
        self.ser.close()


class RobotGUI:
    def __init__(self, robot):
        self.robot = robot
        self.root = tk.Tk()
        self.root.title("Robot Control")

        # Command selection
        ttk.Label(self.root, text="Select Command:").grid(row=0, column=0, padx=5, pady=5)
        self.cmd_var = tk.StringVar()
        self.cmd_combo = ttk.Combobox(self.root, textvariable=self.cmd_var, state="readonly")
        self.cmd_combo['values'] = [
            "NONE", "ACK", "START", "ESTOP", "ESTOP_RELEASE", "HOME", "DEHOME", "DISABLE"
        ]
        self.cmd_combo.grid(row=0, column=1, padx=5, pady=5)
        ttk.Button(self.root, text="Send", command=self.send_command).grid(row=0, column=2, padx=5, pady=5)

        # Robot state display
        ttk.Label(self.root, text="Robot State:").grid(row=1, column=0, padx=5, pady=5)
        self.state_label = ttk.Label(self.root, text="N/A", width=20)
        self.state_label.grid(row=1, column=1, padx=5, pady=5)

        # Start update loop
        self.update_state()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.mainloop()

    def send_command(self):
        cmd_name = self.cmd_var.get()
        cmd_map = {
            "NONE": CMD_NONE,
            "ACK": CMD_ACK,
            "START": CMD_START,
            "ESTOP": CMD_ESTOP,
            "ESTOP_RELEASE": CMD_ESTOP_RELEASE,
            "HOME": CMD_HOME,
            "DEHOME": CMD_DEHOME,
            "DISABLE": CMD_DISABLE
        }
        if cmd_name in cmd_map:
            self.robot.send_command(cmd_map[cmd_name])

    def update_state(self):
        with self.robot.lock:
            state = self.robot.latest_state
        if state is not None:
            # Highlight ERROR and ESTOP in red
            if state in (0, 3):  # ERROR or ESTOP
                self.state_label.config(text=ROBOT_STATES.get(state, f"Unknown ({state})"),
                                        foreground="red")
            else:
                self.state_label.config(text=ROBOT_STATES.get(state, f"Unknown ({state})"),
                                        foreground="black")
        self.root.after(100, self.update_state)

    def on_close(self):
        self.robot.stop()
        self.root.destroy()


if __name__ == "__main__":
    robot = RobotInterface(SERIAL_PORT, BAUDRATE)
    gui = RobotGUI(robot)
