from pymavlink import mavutil
import time
import math
import serial.tools.list_ports  # For listing available serial ports

# --- SETTINGS ---
def list_com_ports():
    """Lists available COM ports"""
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("COM ports not found")
        return None
    print("\nAvailable COM ports:")
    for port in ports:
        print(f"{port.device}: {port.description}")
    return ports[0].device if ports else None

# Automatically find the first available COM port
default_port = 'COM7'
if default_port:
    connection_string = default_port
else:
    connection_string = 'COM7'  # Fallback if auto-detection fails

print(f"\nUsing port: {connection_string}")

# Initial fake coordinates
lat_start = 55.7539
lon_start = 37.6208
alt_start = 150 # in meters

# --- END OF SETTINGS ---


# --- Connect to the flight controller ---
print(f"Connecting to {connection_string}...")
try:
    master = mavutil.mavlink_connection(connection_string, baud=57600)
except Exception as e:
    print(f"Connection error: {e}")
    exit()

# Wait for the first heartbeat to confirm the connection
print("Waiting for Heartbeat...")
master.wait_heartbeat()
print("Heartbeat received! Connection established.")



# --- Main loop ---
i = 0
while True:
    try:
        # --- 1. Process all incoming messages to update state ---
        while True:
            msg = master.recv_match(blocking=False)
            if not msg:
                break
            
            # Update mode from the last HEARTBEAT
            if msg.get_type() == 'HEARTBEAT':
                if master.mode_mapping():
                    current_mode = master.mode_mapping().get(msg.custom_mode, str(msg.custom_mode))


        radius = 0.0001
        lat = lat_start + radius * math.sin(math.radians(i))
        lon = lon_start + radius * math.cos(math.radians(i))
        
        master.mav.gps_input_send(
            time_usec=int(time.time() * 1e6), gps_id=0, ignore_flags=0,
            time_week_ms=0, time_week=0, fix_type=3, lat=int(lat * 1e7),
            lon=int(lon * 1e7), alt=alt_start, hdop=1.5, vdop=1.5,
            vn=0.0, ve=1.0, vd=0.0, speed_accuracy=0.5, horiz_accuracy=1.5,
            vert_accuracy=1.5, satellites_visible=10, yaw=0
        )
        
        i += 1
        time.sleep(0.1) # Reduced delay for more frequent commands

    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")
        # Release override before exiting
        master.mav.rc_channels_override_send(
            master.target_system, master.target_component, *([0]*8)
        )
        print("RC override released.")
        break
    except Exception as e:
        print(f"An error occurred: {e}")
        break

print("Connection closed.") 