from pymavlink import mavutil
import time
import serial.tools.list_ports
import numpy as np

class MavlinkController:
    def __init__(self, connection_string='/dev/ttyACM0', baudrate=57600):
        print(f"\nConnecting to {connection_string}...")

        try:
            self.master = mavutil.mavlink_connection(connection_string, baud=baudrate)
        except Exception as e:
            print(f"Connection error: {e}")
            return

        # Wait for heartbeat
        print("Waiting for Heartbeat...")
        self.master.wait_heartbeat()
        print("Heartbeat received. Connected to system %d, component %d" %
            (self.master.target_system, self.master.target_component))
        
    def get_EKF_speed(self):
        msg = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=True)

        # North, east, down in m/s
        vx, vy, vz = msg.vx, msg.vy, msg.vz

        return vx, vy, vz
    
    def get_heading(self):
        msg = self.master.recv_match(type='SCALED_IMU2', blocking=True)

        xmag, ymag = msg.xmag, msg.ymag
        heading_rad = np.arctan2(ymag, xmag)
        heading_deg = np.rad2deg(heading_rad) % 360

        return heading_deg, heading_rad

    # Don't know if it works
    def get_heading_VRF_HUD(self):
        msg = self.master.recv_match(type='VFR_HUD', blocking=True)
        heading_deg = msg.heading  # В градусах (0–360)

        return heading_deg
    
    def get_speed_direction(self):
        vx, vy, _ = self.get_EKF_speed()

        speed = np.sqrt(vx**2 + vy**2)
        direction = np.arctan2(vy, vx)

        return speed, direction

    def _angle_substraction_deg(self, a, b):
        diff = (a - b) % 360
        return diff
    
    def __example(self):
        speed, direction = self.get_speed_direction()
        heading = self.get_heading()
        
        lidar_angle = self._angle_substraction_deg(direction, heading)

    def _move_drone(self, time_to_move, pitch=0, roll=0, thrust=0):
        '''
        from 1000 to 2000, where: 
        1001:1499 - back/left;
        1501:1999 - forward/right
        '''
        for name, val in {'roll': roll, 'pitch': pitch, 'thrust': thrust}.items():
            if not (1000 <= val <= 2000):
                raise ValueError(f"{name} имеет недопустимое значение: {val}")
            

        self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                thrust,
                0,
                pitch,
                roll,
                0,
                0, 0, 0)
        
        time.sleep(time_to_move)
        
        self._stop_drone()

    def _stop_drone(self):
        self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                0,
                0,
                0,
                0,
                0,
                0, 0, 0)

    def stop_drone(self):
        try:
            # Get mode mappings
            mode_mapping = self.master.mode_mapping()
            if not mode_mapping:
                raise Exception("Failed to get mode mapping from autopilot.")

            self._move_drone(2, 1200)
              

        except KeyboardInterrupt:
            print("\nInterrupted by user. Exiting...")
        except Exception as e:
            print(f"Error: {e}")

