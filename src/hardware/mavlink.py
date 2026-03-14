from pymavlink import mavutil

import time
import threading

import numpy as np

import logging

logging.basicConfig(
    filename='log/lidar.log',
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
)

logger = logging.getLogger(__name__)


class MavlinkController:
    '''
    Controller by mavlink protocole. 
    
    Manages connection to flight controller via specific port, read telemetry (e.t.c), write params and send RC commands
    
    Life cycle: 
        1. Create instance 
        2. call ```conncet()``` - make connection and start reading thread
        3. Use methods
        4. Call ```close()``` 
    
    Example
    ```
    drone = MavlinkController("/dev/ttyS0", baudrate=115200)
    drone.connect()

    drone.send_rc(
        pitch=0.3,   # вперед
        roll=0.0,
        throttle=0.0
    )
    
    time.sleep(1)
    
    drone.stop()
    drone.close()

    ```
    '''

    def __init__(self, connection_string="/dev/ttyS0", baudrate=115200):
        '''
        Args:
            connection_string: path to port or TCP-address 
                               (e.g., "/dev/ttyS0" or "tcp:192.168.1.1:5760").
            baudrate:          Скорость соединения в бодах. Должна совпадать
                               с настройкой полётного контроллера.
        '''

        self.connection_string = connection_string
        self.baudrate = baudrate

        self.master = None
        self.running = False

        self.telemetry = {}

        logger.info(f"Controller initialized. Port={connection_string} Baud={baudrate}")

    # -----------------------------
    # CONNECTION
    # -----------------------------

    def connect(self):
        '''
        Establishes a connection to the flight controller and starts the background reader thread.

        Blocks until the first heartbeat packet is received from the controller.
        On success, launches _message_loop in a daemon thread.

        Raises:
            Exception: Any connection error is re-raised after logging.
        '''

        logger.info(f"Connecting to {self.connection_string}")

        try:
            self.master = mavutil.mavlink_connection(
                self.connection_string,
                baud=self.baudrate
            )

            logger.info("Waiting for heartbeat...")
            self.master.wait_heartbeat()

            logger.info(
                f"Heartbeat received. System={self.master.target_system} "
                f"Component={self.master.target_component}"
            )

        except Exception as e:
            logger.error(f"Connection failed: {e}")
            raise

        self.running = True

        self.reader_thread = threading.Thread(
            target=self._message_loop,
            daemon=True
        )

        self.reader_thread.start()

        logger.info("MAVLink reader thread started")
        
        # Short pause to allow the thread to start receiving messages
        # before the caller begins reading telemetry
        # Not shure we really need it
        time.sleep(0.2)

    # -----------------------------
    # MESSAGE LOOP
    # -----------------------------

    def _message_loop(self):
        '''
        Background loop for receiving MAVLink messages.
        '''

        logger.info("Starting MAVLink message loop")

        while self.running:

            try:

                msg = self.master.recv_match(blocking=True)

                if msg is None:
                    continue

                msg_type = msg.get_type()

                self.telemetry[msg_type] = msg

            except Exception as e:
                logger.error(f"MAVLink read error: {e}")

    # -----------------------------
    # PARAMS
    # -----------------------------

    def set_param(self, name, value):
        '''
        Sets a flight controller parameter via MAVLink PARAM_SET.
        
        Args:
            name:  Parameter name in ArduPilot/PX4 format (e.g. "AVOID_ENABLE").
            value: Numeric parameter value. Will be cast to float.
        '''
        logger.info(f"Setting parameter {name} = {value}")

        try:

            self.master.mav.param_set_send(
                self.master.target_system,
                self.master.target_component,
                name.encode(),
                float(value),
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )
            
            time.sleep(0.2)
        except Exception as e:
            logger.error(f"Failed to set param {name}: {e}")

    # -----------------------------
    # TELEMETRY
    # -----------------------------
    
    def get_rc_channel(self, channel):
        '''
        Returns the raw PWM value of the specified RC channel.

        Args:
            channel: Channel number (1–18).

        Returns:
            PWM value in the range 1000–2000, or None if
            the RC_CHANNELS message has not been received yet.
        '''
        
        msg = self.telemetry.get("RC_CHANNELS")

        if msg is None:
            return None

        attr = f"chan{channel}_raw"

        if hasattr(msg, attr):
            return getattr(msg, attr)

        return None
    
    def is_switch_on(self, channel, threshold=1200) -> bool:
        '''
        Checks whether the switch on the specified RC channel is active.
        
        A switch is considered on if its PWM value exceeds the threshold.
        Typical switch values are ~1000 (off) and ~2000 (on).

        Args:
            channel:   RC channel number (1–18).
            threshold: PWM threshold above which the switch is considered on.
                       Default is 1200.

        Returns:
            True if the channel is above the threshold, False if below or data unavailable.

        Example
        ```
        if drone.is_switch_on(7):
            print("Avoidance ON")

        if drone.is_switch_on(5):
            print("Autonomous mode")
            
        ```
        '''

        value = self.get_rc_channel(channel)

        if value is None:
            return False

        return value > threshold
    
    def get_sticks(self) -> dict:
        '''
        Returns the current stick positions of the remote controller.

        Reads channels 1–4 from RC_CHANNELS and returns them as a named dictionary
        for convenient access.

        Returns:
            Dictionary with keys roll, pitch, throttle, yaw containing raw PWM values,
            or None if telemetry has not been received yet.
        
        '''
        channels = self.get_rc_channels()

        if channels is None:
            return None

        sticks = {
            "roll": channels[1],
            "pitch": channels[2],
            "throttle": channels[3],
            "yaw": channels[4]
        }

        return sticks

    def get_speed(self):

        msg = self.telemetry.get("LOCAL_POSITION_NED")

        if msg is None:
            logger.warning("LOCAL_POSITION_NED not available")
            return None

        vx, vy, vz = msg.vx, msg.vy, msg.vz

        logger.info(f"Speed: vx={vx:.2f} vy={vy:.2f} vz={vz:.2f}")

        return vx, vy, vz

    def get_heading(self):

        msg = self.telemetry.get("ATTITUDE")

        if msg is None:
            logger.warning("ATTITUDE message not available")
            return None

        yaw = msg.yaw

        logger.info(f"Heading yaw={yaw}")

        return yaw

    # -----------------------------
    # RC CONTROL
    # -----------------------------

    def _to_pwm(self, value):
        '''
        Converts a normalized value [-1.0, 1.0] to a PWM signal [1000, 2000].

        Neutral position (0.0) maps to 1500 µs.
        Values outside [-1, 1] are clipped.

        Args:
            value: Normalized value from -1.0 (full back/left)
                   to 1.0 (full forward/right).

        Returns:
            Integer PWM value in the range [1000, 2000].
        '''
        
        value = np.clip(value, -1, 1)
        return int(1500 + value * 500)

    def send_rc(self, roll=0, pitch=0, throttle=0, yaw=0):
        '''
        Отправляет RC_CHANNELS_OVERRIDE с заданными значениями стиков.

        Все параметры принимаются в нормализованном виде [-1.0, 1.0]
        и автоматически конвертируются в PWM через _to_pwm().
        Каналы 5–8 не переопределяются (передаются как 0).

        Args:
            roll:     Крен.    |  -1.0 = влево,  | 0 = нейтраль, | 1.0 = вправо.
            pitch:    Тангаж.  |  -1.0 = назад,  | 0 = нейтраль, | 1.0 = вперёд.
            throttle: Газ.     |  -1.0 = минимум,| 0 = нейтраль, | 1.0 = максимум.
            yaw:      Рыскание.|  -1.0 = влево,  | 0 = нейтраль, | 1.0 = вправо.

        Example:
        ```
            drone.send_rc(pitch=0.3)        # плавно вперёд
            drone.send_rc(throttle=1.0)     # полный газ
            drone.send_rc()                 # все стики в нейтраль
        ```
        '''

        roll_pwm = self._to_pwm(roll)
        pitch_pwm = self._to_pwm(pitch)
        throttle_pwm = self._to_pwm(throttle)
        yaw_pwm = self._to_pwm(yaw)

        logger.info(
            f"RC override: roll={roll_pwm} pitch={pitch_pwm} "
            f"throttle={throttle_pwm} yaw={yaw_pwm}"
        )

        try:

            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                roll_pwm,
                pitch_pwm,
                throttle_pwm,
                yaw_pwm,
                0, 0, 0, 0
            )

        except Exception as e:
            logger.error(f"Failed to send RC override: {e}")

    def stop(self):
        """
        Снимает RC-override, возвращая управление пульту оператора.

        Передаёт нули во все каналы RC_CHANNELS_OVERRIDE — это стандартный
        способ освободить каналы и вернуть управление физическому пульту.
        Не завершает соединение — для этого используйте close().
        """
        
        logger.info("Stopping drone (release RC override)")

        try:

            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                0, 0, 0, 0, 0, 0, 0, 0
            )

        except Exception as e:
            logger.error(f"Stop command failed: {e}")

    # -----------------------------
    # SEND COMAND
    # -----------------------------

    def send_distance(self, distance_cm, orientation):

        logger.info(
            f"Sending distance sensor: distance={distance_cm} orientation={orientation}"
        )

        try:

            self.master.mav.distance_sensor_send(
                int(time.time()*1000) & 0xFFFFFFFF,
                10,
                1000,
                int(distance_cm),
                0,
                0,
                orientation,
                0
            )

        except Exception as e:
            logger.error(f"Distance sensor send failed: {e}")

    # -----------------------------
    # CLOSE
    # -----------------------------

    def close(self):

        logger.info("Closing MAVLink connection")

        self.running = False

        if self.master:
            self.master.close()
            

            