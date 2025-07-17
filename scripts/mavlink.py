from pymavlink import mavutil
import time
import serial.tools.list_ports

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
        
    # def _list_com_ports(self):
    #     """Lists available COM ports"""
    #     ports = list(serial.tools.list_ports.comports())
    #     if not ports:
    #         print("COM ports not found")
    #         return None
    #     print("\nAvailable COM ports:")
    #     for port in ports:
    #         print(f"{port.device}: {port.description}")
    #     return ports[0].device if ports else None

    # def _get_curr_mode(self):
    #     print("Waiting for Heartbeat...")
    #     self.master.wait_heartbeat()
    #     print("Heartbeat received. Connected to system %d, component %d" %
    #         (self.master.target_system, self.master.target_component))
        
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
                0,
                0,
                pitch,
                roll,
                thrust,
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

