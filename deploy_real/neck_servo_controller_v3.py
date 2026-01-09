The logic is 95% perfect, but there is one critical syntax error inside your redis_worker that will cause the script to crash the moment it receives data.

In Python, json.load() is for reading files, while json.loads() (load string) is for reading data from memory (like Redis bytes).

The Fix

Look at this section in your redis_worker:

code
Python
download
content_copy
expand_less
# ... inside redis_worker ...
if data:
    # ERROR IS HERE: json.load expects a file, not a string/bytes
    cmd = json.load(data)

Change it to:

code
Python
download
content_copy
expand_less
if data:
    cmd = json.loads(data) # <--- Added 's'
Other Minor Cleanup (Optional but Recommended)

Since you moved the Redis logic to redis_worker, the old setup code and deprecated methods are just "dead code" (clutter). They won't break anything, but removing them makes the script cleaner.

You can remove def setup_redis(self): entirely (the thread does the connection now).

You can remove def pub_neck_state(self, state):.

You can remove def get_neck_from_redis(self):.

In __init__, you can remove self.redis_client = None.

Final Verified Code

Here is the corrected version. I fixed the json.loads bug and removed the dead code for you. This is ready to run.

code
Python
download
content_copy
expand_less
#!/usr/bin/env python3
import argparse
import json
import time
import numpy as np
import redis
import threading
from rich import print
from dynamixel_sdk import *


class NeckServoController:
    # Control table addresses from: https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/
    ADDR_TORQUE_ENABLE = 64
    ADDR_GOAL_POSITION = 116
    ADDR_PRESENT_POSITION = 132
    ADDR_OPERATING_MODE = 11
    ADDR_PROFILE_VELOCITY = 112
    ADDR_DRIVE_MODE = 10
    ADDR_PROFILE_ACCELERATION = 108
    ADDR_HOMING_OFFSET = 20
    ADDR_MIN_POSITION_LIMIT = 52
    ADDR_MAX_POSITION_LIMIT = 48
    
    OPERATING_MODE_POSITION = 3          # Position Control Mode (single turn, 0-4095)
    OPERATING_MODE_EXTENDED_POSITION = 4 # Extended Position Control Mode (multi turn)
    
    # [yaw, pitch]
    DXL_IDS = [0, 1]
    ENABLE_PITCH_MOTOR = False
    BAUDRATE = 2000000
    PROTOCOL_VERSION = 2.0

    # in dxl world: 0 -> 0 deg, 4095 -> 360 deg
    DXL_MIN_POSITION = 0
    DXL_MAX_POSITION = 4095

    # Per motor configuration: index 0 = yaw, index 1 = pitch
    # Center positions (what calibration sets current position to)
    MOTOR_CENTER_DEG = [180.0, 145.0]  # Yaw: 180 deg, Pitch: 145 deg
    
    # Movement ranges (min, max) in degrees
    MOTOR_RANGE_DEG = [
        (90.0, 270.0),    # Yaw
        (105.0, 230.0),   # Pitch
    ]
    
    @staticmethod
    def deg_to_ticks(deg):
        """Convert degrees to Dynamixel ticks (0-4095 for 0-360)"""
        return int((deg / 360.0) * 4096)
    
    RAD_TO_DXL = 4096 / (2 * np.pi)
    
    def __init__(self, args):
        self.args = args
        self.port_name = args.port
        self.target_fps = args.target_fps
        self.calibrate_on_start = args.calibrate
        
        self.port_handler = None
        self.packet_handler = None
        
        # following are shared thread variables 
        self.latest_neck_command = [0.0, 0.0]
        self.current_neck_state = [0.0, 0.0]
        self.running = True
        
        self.neck_state_key = "state_neck_unitree_g1_with_hands"
        self.neck_action_key = "action_neck_unitree_g1_with_hands"
    
    def redis_worker(self):
        """thread for network comm, to not block the motor control loop"""
        print(f"[magenta]Connecting to Host Redis at {self.args.redis_ip}...[/]")
        try:
            r = redis.Redis(host=self.args.redis_ip, port=6379, db=0, socket_timeout=5.0)
            r.ping()
            print("[green]Redis Client Connected to Host PC![/]")
        except Exception as e:
            print(f"[red]Redis Client connection to Host PC failed: {e}[/]")
            return
        
        while self.running:
            try:
                # Publish state (from g1 -> PC)
                state_json = json.dumps(self.current_neck_state)
                r.set(self.neck_state_key, state_json)

                # Read command (pc -> g1)
                data = r.get(self.neck_action_key)
                if data:
                    cmd = json.loads(data)
                    if len(cmd) >= 2:
                        self.latest_neck_command = [float(cmd[0]), float(cmd[1])]
            except Exception as e:
                print(f"[red]Redis Worker Error: {e}[/]")
                time.sleep(1)

            time.sleep(0.01)

    def _get_active_motor_ids(self):
        """Return list of motor IDs that are enabled"""
        if self.ENABLE_PITCH_MOTOR:
            return self.DXL_IDS
        else:
            return [self.DXL_IDS[0]]
    
    def _get_motor_name(self, idx):
        """Get motor name from index"""
        return "Yaw" if idx == 0 else "Pitch"
    
    def _is_motor_enabled(self, idx):
        """Check if motor at given index is enabled"""
        if idx == 0:
            return True
        elif idx == 1:
            return self.ENABLE_PITCH_MOTOR
        return False
        
    def read_current_position(self, dxl_id):
        """Read current position from a Dynamixel servo"""
        position, result, error = self.packet_handler.read4ByteTxRx(self.port_handler, dxl_id, self.ADDR_PRESENT_POSITION)
        if result != COMM_SUCCESS:
            print(f"[red]Failed to read position from motor {dxl_id}[/]")
            return None
        return position
    
    def read_homing_offset(self, dxl_id):
        """Read current homing offset from a Dynamixel servo"""
        offset, result, error = self.packet_handler.read4ByteTxRx(self.port_handler, dxl_id, self.ADDR_HOMING_OFFSET)
        if result != COMM_SUCCESS:
            print(f"[red]Failed to read homing offset from motor {dxl_id}: {self.packet_handler.getTxRxResult(result)}[/]")
            return None
        if offset > 0x7FFFFFFF: 
            offset = offset - 0x100000000
        return offset
    
    def write_homing_offset(self, dxl_id, offset):
        """Write homing offset to a Dynamixel servo (torque must be disabled)"""
        if offset < 0:
            offset = offset + 0x100000000
        result, error = self.packet_handler.write4ByteTxRx(self.port_handler, dxl_id, self.ADDR_HOMING_OFFSET, int(offset))
        if result != COMM_SUCCESS:
            print(f"[red]Failed to write homing offset to motor {dxl_id}: {self.packet_handler.getTxRxResult(result)}[/]")
            return False
        if error != 0:
            print(f"[red]Error writing homing offset to motor {dxl_id}: {self.packet_handler.getRxPacketError(error)}[/]")
            return False
        return True
    
    def set_torque(self, dxl_id, enable):
        """Enable or disable torque for a motor"""
        result, error = self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, self.ADDR_TORQUE_ENABLE, 1 if enable else 0)
        if result != COMM_SUCCESS:
            print(f"[red]Failed to {'enable' if enable else 'disable'} torque for motor {dxl_id}[/]")
            return False
        return True
    
    def calibrate(self):
        """Calibrate motors so their current physical position becomes the defined center."""
        print("[bold cyan]Starting calibration...[/]")
        print("[yellow]Make sure motors are manually positioned at desired center position![/]")
        
        for idx, dxl_id in enumerate(self.DXL_IDS):
            if not self._is_motor_enabled(idx):
                print(f"[yellow]Skipping {self._get_motor_name(idx)} motor (ID {dxl_id}) - disabled[/]")
                continue
            
            motor_name = self._get_motor_name(idx)
            target_center_deg = self.MOTOR_CENTER_DEG[idx]
            target_center_ticks = self.deg_to_ticks(target_center_deg)
            
            print(f"\n[cyan]Calibrating {motor_name} motor (ID {dxl_id}) to {target_center_deg} deg...[/]")
            
            if not self.set_torque(dxl_id, False):
                print(f"[red]Failed to disable torque for {motor_name} motor, skipping calibration[/]")
                continue
            
            current_offset = self.read_homing_offset(dxl_id)
            if current_offset is None:
                print(f"[red]Failed to read current homing offset for {motor_name} motor[/]")
                continue
            print(f"  Current homing offset: {current_offset}")
            
            if not self.write_homing_offset(dxl_id, 0):
                print(f"[red]Failed to reset homing offset for {motor_name} motor[/]")
                continue
            
            time.sleep(0.05)
            
            raw_position = self.read_current_position(dxl_id)
            if raw_position is None:
                print(f"[red]Failed to read raw position for {motor_name} motor[/]")
                self.write_homing_offset(dxl_id, current_offset)
                continue
            print(f"  Raw position (no offset): {raw_position} ({raw_position * 360.0 / 4096:.1f} deg)")
            
            new_offset = target_center_ticks - raw_position
            print(f"  New homing offset: {new_offset} (to make position = {target_center_deg} deg)")
            
            if not self.write_homing_offset(dxl_id, new_offset):
                print(f"[red]Failed to write new homing offset for {motor_name} motor[/]")
                self.write_homing_offset(dxl_id, current_offset)
                continue
            
            time.sleep(0.05)
            
            verify_position = self.read_current_position(dxl_id)
            if verify_position is not None:
                verify_deg = verify_position * 360.0 / 4096
                print(f"  [green]Verified position after calibration: {verify_position} ({verify_deg:.1f} deg)[/]")
            
            print(f"[green]{motor_name} motor calibrated to {target_center_deg} deg successfully![/]")
        
        print("\n[bold green]Calibration complete![/]")
    
    def setup_dynamixel(self):
        """Initialize Dynamixel communication"""
        print(f"[cyan]Initializing Dynamixel on {self.port_name}...[/]")
        
        self.port_handler = PortHandler(self.port_name)
        self.packet_handler = PacketHandler(self.PROTOCOL_VERSION)
        
        if not self.port_handler.openPort():
            raise RuntimeError(f"Failed to open port {self.port_name}")
        print(f"Port {self.port_name} opened")
        
        if not self.port_handler.setBaudRate(self.BAUDRATE):
            raise RuntimeError(f"Failed to set baudrate to {self.BAUDRATE}")
        print(f"Baudrate set to {self.BAUDRATE}")
        
        if self.calibrate_on_start:
            self.calibrate()
        
        for idx, dxl_id in enumerate(self.DXL_IDS):
            motor_name = self._get_motor_name(idx)
            
            if not self._is_motor_enabled(idx):
                print(f"[yellow]Skipping {motor_name} motor (ID {dxl_id}) - disabled[/]")
                continue
            
            min_deg, max_deg = self.MOTOR_RANGE_DEG[idx]
            min_ticks = self.deg_to_ticks(min_deg)
            max_ticks = self.deg_to_ticks(max_deg)
            
            if not self.set_torque(dxl_id, False):
                raise RuntimeError(f"Failed to disable torque for {motor_name} motor")

            result, error = self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, self.ADDR_DRIVE_MODE, 4)
            if error:
                raise RuntimeError(f"Failed to set the drive mode for {motor_name} motor")

            result, error = self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, self.ADDR_OPERATING_MODE, self.OPERATING_MODE_POSITION)
            if error:
                raise RuntimeError(f"Failed to set the operating mode for {motor_name} motor")
            print(f"  {motor_name}: Position Control Mode (single turn) set")
            
            result, error = self.packet_handler.write4ByteTxRx(self.port_handler, dxl_id, self.ADDR_MIN_POSITION_LIMIT, min_ticks)
            if error:
                raise RuntimeError(f"Failed to set min position limit for {motor_name} motor")
            
            result, error = self.packet_handler.write4ByteTxRx(self.port_handler, dxl_id, self.ADDR_MAX_POSITION_LIMIT, max_ticks)
            if error:
                raise RuntimeError(f"Failed to set max position limit for {motor_name} motor")
            print(f"  {motor_name}: Position limits set [{min_ticks}, {max_ticks}] ({min_deg} deg-{max_deg} deg)")
            
            result, error = self.packet_handler.write4ByteTxRx(self.port_handler, dxl_id, self.ADDR_PROFILE_ACCELERATION, 150)
            if error:
                raise RuntimeError(f"Failed to set the acceleration profile for {motor_name} motor")

            result, error = self.packet_handler.write4ByteTxRx(self.port_handler, dxl_id, self.ADDR_PROFILE_VELOCITY, 250)
            if error:
                raise RuntimeError(f"Failed to set the velocity profile for {motor_name} motor")
                        
            if not self.set_torque(dxl_id, True):
                print(f"[red]Failed to enable {motor_name} motor (ID {dxl_id})[/]")
            else:
                print(f"{motor_name} motor (ID {dxl_id}) enabled")

    def dxl_to_rad_position(self, dxl_pos, motor_index):
        """Convert Dynamixel position to radians relative to motor's center"""
        motor_deg = (dxl_pos / 4096.0) * 360.0
        center_deg = self.MOTOR_CENTER_DEG[motor_index]
        
        if motor_index == 0:  # Yaw motor
            deg = motor_deg - center_deg
        elif motor_index == 1:  # Pitch motor
            deg = center_deg - motor_deg
        else:
            deg = motor_deg - center_deg
        
        return deg * (np.pi / 180.0)
    
    def rad_to_dxl_position(self, rad, motor_index):
        """Convert radians to Dynamixel position units with saturation."""
        deg = rad * (180.0 / np.pi)
        center_deg = self.MOTOR_CENTER_DEG[motor_index]
        min_deg, max_deg = self.MOTOR_RANGE_DEG[motor_index]
        
        if motor_index == 0:  # Yaw motor
            motor_deg = center_deg + deg
        elif motor_index == 1:  # Pitch motor
            motor_deg = center_deg - deg
        else:
            raise ValueError(f"Invalid motor_index: {motor_index}")
        
        if motor_deg < min_deg:
            motor_deg = min_deg
        elif motor_deg > max_deg:
            motor_deg = max_deg
        
        return self.deg_to_ticks(motor_deg)

    def set_position(self, neck_data):
        """Set servo positions from [yaw, pitch] in radians."""
        yaw, pitch = neck_data
        
        yaw_pos = self.rad_to_dxl_position(yaw, motor_index=0)
        result, error = self.packet_handler.write4ByteTxRx(self.port_handler, self.DXL_IDS[0], self.ADDR_GOAL_POSITION, yaw_pos)
        if result != COMM_SUCCESS:
            print(f"[red]Failed to set position for yaw motor: {result}[/]")
        
        if self.ENABLE_PITCH_MOTOR:
            pitch_pos = self.rad_to_dxl_position(pitch, motor_index=1)
            result, error = self.packet_handler.write4ByteTxRx(self.port_handler, self.DXL_IDS[1], self.ADDR_GOAL_POSITION, pitch_pos)
            if result != COMM_SUCCESS:
                print(f"[red]Failed to set position for pitch motor: {result}[/]")
    
    def get_current_neck_state(self):
        """Reads hardware and updates self.current_neck_state"""
        yaw_dxl = self.read_current_position(self.DXL_IDS[0])
        
        if yaw_dxl is None:
            print("[red]Failed to read yaw motor position, returning last known state[/]")
            return self.current_neck_state
        
        yaw_rad = self.dxl_to_rad_position(yaw_dxl, motor_index=0)
        
        self.current_neck_state = [float(yaw_rad), 0.0]
        
        if self.ENABLE_PITCH_MOTOR:
            pitch_dxl = self.read_current_position(self.DXL_IDS[1])
            if pitch_dxl is None:
                print("[red]Failed to read pitch motor position, returning last known state[/]")
                return self.current_neck_state
            pitch_rad = self.dxl_to_rad_position(pitch_dxl, motor_index=1)
        else:
            pitch_rad = 0.0
        
        self.current_neck_state = [float(yaw_rad), float(pitch_rad)]
        return self.current_neck_state
        
    def cleanup(self):
        """Disable motors and close port"""
        print("\n[yellow]Shutting down...[/]")
        
        print("[yellow]Returning to center position...[/]")
        try:
            self.set_position([0.0, 0.0])
            time.sleep(0.8)
        except Exception as e:
            print(f"[red]Cleanup: failed to send center position: {e}[/]")

        for idx, dxl_id in enumerate(self.DXL_IDS):
            if not self._is_motor_enabled(idx):
                continue
            try:
                self.set_torque(dxl_id, False)
            except Exception as e:
                print(f"[red]Cleanup: error while disabling torque for motor {dxl_id}: {e}[/]")
            
        try:
            self.port_handler.closePort()
        except Exception as e:
            print(f"[red]Cleanup: error while closing port: {e}[/]")
        else:
            print("[green]Motors returned to center, disabled, and port closed[/]")
        
    def run(self):
        """Main control loop"""
        print("[bold cyan]Starting Neck Servo Controller...[/]\n")
        
        self.setup_dynamixel()
        
        t = threading.Thread(target=self.redis_worker, daemon=True)
        t.start()
        
        print(f"\n[bold green]Controller running at {self.target_fps} Hz[/]")
        
        dt = 1.0 / self.target_fps
        min_sleep = 0.02
        step = 0
        print_interval = 25
        
        try:
            while True:
                t_start = time.time()

                self.get_current_neck_state()
                
                # a worker thread updates these
                target_yaw = self.latest_neck_command[0]
                target_pitch = self.latest_neck_command[1] if self.ENABLE_PITCH_MOTOR else 0.0
                
                self.set_position([target_yaw, target_pitch])
                
                step += 1
                if step % print_interval == 0:
                    yaw_deg = np.degrees(target_yaw)
                    pitch_deg = np.degrees(target_pitch)
                    yaw_pos = self.rad_to_dxl_position(target_yaw, motor_index=0)
                    pitch_info = ""
                    if self.ENABLE_PITCH_MOTOR:
                        pitch_pos = self.rad_to_dxl_position(target_pitch, motor_index=1)
                        pitch_info = f", pitch={pitch_deg:+6.1f} deg (pos:{pitch_pos:4d})"
                    print(f"Neck: yaw={yaw_deg:+6.1f} deg (pos:{yaw_pos:4d}){pitch_info}", end="\r")
                
                elapsed = time.time() - t_start
                sleep_time = max(min_sleep, dt - elapsed)
                time.sleep(sleep_time)
                    
        except KeyboardInterrupt:
            print("\n")
        finally:
            self.running = False
            self.cleanup()


def main():
    parser = argparse.ArgumentParser(description="Neck Servo Controller")
    parser.add_argument("--port", type=str, default="/dev/ttyUSB0",help="Dynamixel port")
    parser.add_argument("--redis_ip", type=str, required=True,help="Redis server IP (Host PC IP)")
    parser.add_argument("--target_fps", type=int, default=50,help="Control loop frequency")
    parser.add_argument("--calibrate", action="store_true",help="Calibrate motors on startup (Yaw->180 deg, Pitch->145 deg)")
    args = parser.parse_args()
    
    controller = NeckServoController(args)
    controller.run()


if __name__ == "__main__":
    main()