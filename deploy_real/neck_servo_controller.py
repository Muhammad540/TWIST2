#!/usr/bin/env python3
"""
Neck Servo Controller
Reads neck commands from Redis and controls Dynamixel servos.

Usage:
    conda activate gmr
    sudo chmod 777 /dev/ttyUSB0 or ttyUSB1 depending on where your motor control board is connected
    python neck_servo_controller.py

Hardware setup:
    - Dynamixel ID 0: Yaw motor
    - Dynamixel ID 1: Pitch motor
    - Baud rate: 2,000,000 (2Mbps)

NOTE:
    Suppose the value of the neck pitch we get from Redis is x rad, and the value of the neck yaw we get from Redis is y rad (these values in 
    the redis are calculated by utilizing the smplx data of spine and head from PICO headset, more specifically following formula is used:
        Extract neck angle from smplx_data for our designed head
        dof 0: yaw
        dof 1: pitch
        if smplx_data is None:
            return 0.0, 0.0
        spine_rotation = smplx_data['Spine3'][1] # wxyz
        # spine_rotation = smplx_data['Neck'][1] # wxyz # not work, as neck is kind aligned to head.
        head_rotation = smplx_data['Head'][1] # wxyz
        # Convert to rotation objects
        spine_rotation = R.from_quat(spine_rotation, scalar_first=True)
        head_rotation = R.from_quat(head_rotation, scalar_first=True)
        # compute the rpy of the head in local frame (relative to spine)
        # Get the relative rotation: head relative to spine
        relative_rotation = spine_rotation.inv() * head_rotation
        # Convert to Euler angles (roll, pitch, yaw)
        roll, pitch, yaw = relative_rotation.as_euler('xyz', degrees=True)
        # print(f"roll: {roll:.0f}, pitch: {pitch:.0f}, yaw: {yaw:.0f}", end="\r")
        # return float(roll), float(pitch), float(yaw)
        neck_yaw = - pitch # 10~150, middlle 90
        neck_pitch = roll # 8~125, middle 60
        # offset
        # neck_yaw -= 10
        # neck_pitch -= 8
        # clip to above 0   
        # neck_yaw = max(neck_yaw, 0)
        # neck_pitch = max(neck_pitch, 0)
        # degree to radian
        neck_yaw = np.deg2rad(neck_yaw)
        neck_pitch = np.deg2rad(neck_pitch)
        return neck_yaw, neck_pitch
    )

    I manually tested the servo motor and found the following mapping between the motor position and the real world orientation of the neck: 
    1. pitch range for motor in deg: [105, 145, 230]

    145 looking straight
    105 looking up
    230 looking down 

    If the commanded values for pitch are greater than this WE MUST saturate.

    2. yaw range for motor in deg: [50, 180, 300]

    50 looking left (ccw)
    180 looking straight
    300 looking right (cw)

    if the commanded values for yaw are greater than this WE MUST staurate.

    I have configured the homing values of pitch motor and yaw motor so that the above mentioned values are correct. 
    One observation i had was if the we go beyond 360 the motor would suddenly start rotating. Secondly, for the pitch motor if we go 
    beyond the specified limit then that would cause motor current to overload because physically due to the construction of the neck it is not possible to move the motors beyond those points. 
    If we force, they hit the neck parts and rquire more current since we are then forcing the motors.

    We need to somehow map the x rad, y rad to the motor position.
"""
import argparse
import json
import time
import numpy as np
import redis
from rich import print
from dynamixel_sdk import *


class NeckServoController:
    # following values are taken from: https://emanual.robotis.com/docs/en/dxl/x/xc330-m288/
    ADDR_TORQUE_ENABLE = 64
    ADDR_GOAL_POSITION = 116
    ADDR_PRESENT_POSITION = 132
    ADDR_OPERATING_MODE = 11
    ADDR_PROFILE_VELOCITY = 112
    ADDR_DRIVE_MODE = 10
    ADDR_PROFILE_ACCELERATION = 108
    
    # [yaw, pitch]
    DXL_IDS = [0, 1]  
    BAUDRATE = 2000000
    PROTOCOL_VERSION = 2.0

    DXL_MIN_POSITION = 0
    DXL_MAX_POSITION = 4095

    PITCH_MOTOR_RANGE_DEG = (105.0, 230.0)  # (min, max)
    YAW_MOTOR_RANGE_DEG = (50.0, 300.0)     # (min, max)
    
    PITCH_STRAIGHT_DEG = 145.0
    YAW_STRAIGHT_DEG = 180.0
    
    RAD_TO_DXL = 4096 / (2 * np.pi)
    
    def __init__(self, args):
        self.args = args
        self.port_name = args.port
        self.target_fps = args.target_fps
        
        self.port_handler = None
        self.packet_handler = None
        self.redis_client = None
        self.last_neck_data = [0.0, 0.0]
        
        
    def read_current_position(self, dxl_id):
        """Read current position from a Dynamixel servo"""
        position, result, error = self.packet_handler.read4ByteTxRx(
            self.port_handler, dxl_id, self.ADDR_PRESENT_POSITION
        )
        if result != COMM_SUCCESS:
            print(f"[red]Failed to read position from motor {dxl_id}[/]")
            return None
        return position
    
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
        
        for idx, dxl_id in enumerate(self.DXL_IDS):
            motor_name = "Yaw" if idx == 0 else "Pitch"
            
            # NOTE: Torque must be disabled to change operating mode
            result, error = self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, self.ADDR_TORQUE_ENABLE, 0)
            if (error):
                raise RuntimeError(f"Failed to set disable torque for {motor_name} motor")

            result, error = self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, self.ADDR_DRIVE_MODE, 4)
            if (error):
                raise RuntimeError(f"Failed to set the drive mode for {motor_name} motor")

            # setting Extended Position Control Mode
            result, error = self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, self.ADDR_OPERATING_MODE, 4)
            if (error):
                raise RuntimeError(f"Failed to set the operating mode for {motor_name} motor")
            
            # Set profile velocity for smooth motion (higher = slower, smoother)
            result, error = self.packet_handler.write4ByteTxRx(self.port_handler, dxl_id, self.ADDR_PROFILE_ACCELERATION, 150)
            if (error):
                raise RuntimeError(f"Failed to set the acceleration profile for {motor_name} motor")

            result, error = self.packet_handler.write4ByteTxRx(self.port_handler, dxl_id, self.ADDR_PROFILE_VELOCITY, 300)
            if (error):
                raise RuntimeError(f"Failed to set the velocity profile for {motor_name} motor")
                        
            result, error = self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, self.ADDR_TORQUE_ENABLE, 1)
            if result != COMM_SUCCESS:
                print(f"[red]Failed to enable {motor_name} motor (ID {dxl_id})[/]")
            else:
                print(f"{motor_name} motor (ID {dxl_id}) enabled")
        
    def setup_redis(self):
        """Initialize Redis connection"""
        self.redis_client = redis.Redis(
            host=self.args.redis_ip,
            port=6379,
            db=0,
            socket_timeout=0.05
        )
        self.redis_client.ping()
        print(f"Redis connected at {self.args.redis_ip}:6379")
        
    def rad_to_dxl_position(self, rad, motor_index):
        """
        Convert radians to Dynamixel position units with saturation.
        
        Args:
            rad: Angle in radians (SMPL derived neck angle)
            motor_index: 0 for yaw, 1 for pitch
            
        Returns:
            Dynamixel position units (0-4095), clamped to safe range
        """
        # Convert radians to degrees
        deg = rad * (180.0 / np.pi)
        
        if motor_index == 0:  # Yaw motor
            # VR/SMPL positive yaw = looking left (CCW) -> motor position should DECREASE
            # VR/SMPL negative yaw = looking right (CW) -> motor position should INCREASE
            # Motor convention: higher pos = rotate right, lower pos = rotate left
            # Therefore subtract the human yaw from the straight-ahead reference.
            motor_deg = self.YAW_STRAIGHT_DEG + deg  # Straight position (180°) - human yaw
            
            # Debug print to verify mapping
            if abs(deg) > 0.1:
                direction = "right (CW)" if deg > 0 else "left (CCW)"
                print(f"[cyan]Yaw: SMPL={deg:+.1f}° ({direction}) -> Motor={motor_deg:.1f}°[/]")
            
            # Saturate to physical limits: [50°, 300°]
            if motor_deg < self.YAW_MOTOR_RANGE_DEG[0]:
                print(f"[yellow]Yaw motor saturated: {motor_deg:.1f}° -> {self.YAW_MOTOR_RANGE_DEG[0]:.1f}° (left limit)[/]")
                motor_deg = self.YAW_MOTOR_RANGE_DEG[0]
            elif motor_deg > self.YAW_MOTOR_RANGE_DEG[1]:
                print(f"[yellow]Yaw motor saturated: {motor_deg:.1f}° -> {self.YAW_MOTOR_RANGE_DEG[1]:.1f}° (right limit)[/]")
                motor_deg = self.YAW_MOTOR_RANGE_DEG[1]
                
        elif motor_index == 1:  # Pitch motor
            # SMPL positive pitch = looking up = decrease motor position
            # SMPL negative pitch = looking down = increase motor position
            motor_deg = self.PITCH_STRAIGHT_DEG - deg  # Straight position (145°) - SMPL pitch
            
            # Debug print to verify mapping
            if abs(deg) > 0.1:
                direction = "up" if deg > 0 else "down"
                print(f"[cyan]Pitch: SMPL={deg:+.1f}° ({direction}) -> Motor={motor_deg:.1f}°[/]")
            
            # Saturate to physical limits: [105°, 230°]
            if motor_deg < self.PITCH_MOTOR_RANGE_DEG[0]:
                print(f"[yellow]Pitch motor saturated: {motor_deg:.1f}° -> 105.0° (up limit)[/]")
                motor_deg = self.PITCH_MOTOR_RANGE_DEG[0]
            elif motor_deg > self.PITCH_MOTOR_RANGE_DEG[1]:
                print(f"[yellow]Pitch motor saturated: {motor_deg:.1f}° -> {self.PITCH_MOTOR_RANGE_DEG[1]:.1f}° (down limit)[/]")
                motor_deg = self.PITCH_MOTOR_RANGE_DEG[1]
                
        else:
            raise ValueError(f"Invalid motor_index: {motor_index}")
        
        # Convert degrees to Dynamixel position units (0-4095 for 0-360°)
        # XC330-M288 uses 4096 steps per revolution (0-4095)
        dxl_pos = int((motor_deg / 360.0) * self.DXL_MAX_POSITION)
        
        # Final safety clamp to Dynamixel range
        dxl_pos = max(self.DXL_MIN_POSITION, min(self.DXL_MAX_POSITION, dxl_pos))
        
        return dxl_pos

    def set_position(self, neck_data):
        """
        Set servo positions from [yaw, pitch] in radians.
        """
        yaw, pitch = neck_data
        
        yaw_pos = self.rad_to_dxl_position(yaw, motor_index=0)
        pitch_pos = self.rad_to_dxl_position(pitch, motor_index=1)
        
        positions = [yaw_pos, pitch_pos]
        
        for i, dxl_id in enumerate(self.DXL_IDS):
            result, error = self.packet_handler.write4ByteTxRx(self.port_handler, dxl_id, self.ADDR_GOAL_POSITION, positions[i])
            if result != COMM_SUCCESS:
                print(f"[red]Failed to set position for motor {dxl_id}: {result}[/]")
            
    def get_neck_from_redis(self):
        """Read neck command from Redis"""
        try:
            data = self.redis_client.get("action_neck_unitree_g1_with_hands")
            if data is not None:
                neck_data = json.loads(data)
                if len(neck_data) >= 2:
                    self.last_neck_data = neck_data[:2]
                    return self.last_neck_data
        except Exception as e:
            print(f"[red]Error getting neck data from Redis: {e}[/]")
            return [0.0, 0.0]
        return self.last_neck_data
        
    def cleanup(self):
        """Disable motors and close port"""
        print("\n[yellow]Shutting down...[/]")
        
        print("[yellow]Returning to center position...[/]")
        try:
            self.set_position([0.0, 0.0])
            time.sleep(0.8)
        except Exception as e:
            print(f"[red]Cleanup: failed to send center position: {e}[/]")

        for dxl_id in self.DXL_IDS:
            try:
                result, error = self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, self.ADDR_TORQUE_ENABLE, 0)
                if result != COMM_SUCCESS:
                    print(f"[red]Failed to disable torque for motor {dxl_id}: {result}[/]")
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
        
        self.setup_redis()
        self.setup_dynamixel()
        
        print(f"\n[bold green]Controller running at {self.target_fps} Hz[/]")
        
        dt = 1.0 / self.target_fps
        step = 0
        print_interval = 25
        
        try:
            while True:
                t_start = time.time()
                # Two steps:
                # 1. get neck from redis
                # 2. set motor position
                neck_data = self.get_neck_from_redis()
                self.set_position(neck_data)
                
                step += 1
                if step % print_interval == 0:
                    yaw_deg = np.degrees(neck_data[0])
                    pitch_deg = np.degrees(neck_data[1])
                    yaw_pos = self.rad_to_dxl_position(neck_data[0], motor_index=0)
                    pitch_pos = self.rad_to_dxl_position(neck_data[1], motor_index=1)
                    print(f"Neck: yaw={yaw_deg:+6.1f}° (pos:{yaw_pos:4d}), pitch={pitch_deg:+6.1f}° (pos:{pitch_pos:4d})", end="\r")
                
                elapsed = time.time() - t_start
                if elapsed < dt:
                    time.sleep(dt - elapsed)
                    
        except KeyboardInterrupt:
            print("\n")
        finally:
            self.cleanup()


def main():
    parser = argparse.ArgumentParser(description="Neck Servo Controller")
    parser.add_argument("--port", type=str, default="/dev/ttyUSB0",
                        help="Dynamixel port")
    parser.add_argument("--redis_ip", type=str, default="localhost",
                        help="Redis server IP")
    parser.add_argument("--target_fps", type=int, default=50,
                        help="Control loop frequency")
    args = parser.parse_args()
    
    controller = NeckServoController(args)
    controller.run()


if __name__ == "__main__":
    main()