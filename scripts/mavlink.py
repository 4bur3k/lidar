from pymavlink import mavutil
import time
import serial.tools.list_ports
import numpy as np

class MavlinkController:
    def __init__(self, connection_string='/dev/ttyS0', baudrate=115200):
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
        
        params_to_set = {
        "AVOID_ENABLE": 7,
        "AVOID_MARGIN": 2.0,
        "AVOID_BEHAVE": 1
        }
        time.sleep(0.2)  # Подождём немного между запросами

        for param_name, param_value in params_to_set.items():
            print(f" Устанавливаем {param_name} = {param_value}")
            self.master.mav.param_set_send(
                self.master.target_system,
                self.master.target_component,
                param_name.encode('utf-8'),
                float(param_value),
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )
            time.sleep(0.2)  # Подождём немного между запросами
        
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

    def _move_drone(self, pitch=0, roll=0, thrust=0):
        '''
        from 1000 to 2000, where: 
        1001:1499 - back/left;
        1501:1999 - forward/right
        '''
        # for name, val in {'roll': roll, 'pitch': pitch, 'thrust': thrust}.items():
        #     if not (1000 <= val <= 2000):
        #         raise ValueError(f"{name} имеет недопустимое значение: {val}")
        
        self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                thrust,
                roll,
                pitch,
                0,
                0,
                0, 0, 0)
        

    def _return_controll(self):
        self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                0,
                0,
                0,
                0,
                0,
                0, 0, 0)

    def stop_drone(self, pitch, roll):
        try:
            if pitch != 1500 or roll != 1500:
                self._move_drone(pitch, roll)
            else:
                self._return_controll()
        

        except KeyboardInterrupt:
            print("\nInterrupted by user. Exiting...")
        except Exception as e:
            print(f"Error: {e}")

    def if_avoidence_enabled(self):
        chan_7_value = 0
        try:
            chan_7_value = self.master.recv_match(type='RC_CHANNELS', blocking = True).chan7_raw
        except BaseException as e:
            print('Error:', e)
            
        if chan_7_value > 1200:
            return True
        else:
            self._return_controll()
            return False
    
    def send_distance_sensors(self, sensor_readigs, RATE=10):
        
        for orientation, dist in sensor_readigs:
            print(orientation, dist)
            self.master.mav.distance_sensor_send(
                time_boot_ms = int(time.time() * 1000) & 0xFFFFFFFF,
                min_distance = 10,     # минимальная дальность (см)
                max_distance = 1000,   # максимальная дальность (см)
                current_distance = int(dist),
                type = 0,
                id = 0,
                orientation = orientation,
                covariance = 0
            )
            
        time.sleep(1.0/RATE)