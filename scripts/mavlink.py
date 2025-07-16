from pymavlink import mavutil
import time
import serial.tools.list_ports

class MavlinkController:
    def __init__(self, connection_string='/dev/ttyACM0', baudrate=57600):
        print(f"\nConnecting to {connection_string}...")

        try:
            self.master = mavutil.mavlink_connection(connection_string, baud=baudrate)
        except Exception as e:
            print(f"**********Connection error: {e}")
            return

        # Wait for heartbeat
        print("Waiting for Heartbeat...")
        self.master.wait_heartbeat()
        print("Heartbeat received. Connected to system %d, component %d" %
            (self.master.target_system, self.master.target_component))
        
    def list_com_ports(self):
        """Lists available COM ports"""
        ports = list(serial.tools.list_ports.comports())
        if not ports:
            print("COM ports not found")
            return None
        print("\nAvailable COM ports:")
        for port in ports:
            print(f"{port.device}: {port.description}")
        return ports[0].device if ports else None

    def get_curr_mode(self):
        print("Waiting for Heartbeat...")
        self.master.wait_heartbeat()
        print("Heartbeat received. Connected to system %d, component %d" %
            (self.master.target_system, self.master.target_component))
        
    def stop_drone(self):
        try:
            # Get mode mappings
            mode_mapping = self.master.mode_mapping()
            if not mode_mapping:
                raise Exception("Failed to get mode mapping from autopilot.")

            # Set to LOITER
            # loiter_mode = 'LOITER'
            # if loiter_mode not in mode_mapping:
            #     raise Exception(f"Mode {loiter_mode} not supported by this autopilot.")
            # self.master.set_mode(mode_mapping[loiter_mode])
            # print(f"Switched to {loiter_mode} mode.")
            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                0,
                0,
                1200,
                0,
                0,
                0, 0, 0)
            print('PIZDAAAAAAAAAA')
            # Wait 3 seconds
            time.sleep(2)
            print('VSE NORMAL')
            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                0,
                0,
                0,
                0,
                0,
                0, 0, 0)

            # # Set to GUIDED
            # guided_mode = 'STABILIZE'
            # if guided_mode not in mode_mapping:
            #     raise Exception(f"Mode {guided_mode} not supported by this autopilot.")
            # self.master.set_mode(mode_mapping[guided_mode])
            # print(f"Switched to {guided_mode} mode.")

        except KeyboardInterrupt:
            print("\nInterrupted by user. Exiting...")
        except Exception as e:
            print(f"Error: {e}")


    # if __name__ == "__main__":
    #     default_port = 'COM7'  # Replace with detected port if needed
    #     stop_drone(connection_string=default_port)
