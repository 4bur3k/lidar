import ydlidar

import threading
import queue

import yaml
import logging

logging.basicConfig(
    filename='log/lidar.log', 
    level=logging.INFO,      
    format='%(asctime)s [%(levelname)s] %(message)s',
)

class LidarConnection:
    '''
    # Args:
    - specific_port: str - port lidar connected to
    - specific_freq: float - freq lidar working (default for yd 10.0)
    
    # Example: 
    ```
    lidar = LidarConnection()
    
    lidar.start()

    try:
        while True:
            points = lidar.get_scan()
            if points is not None:
               
                print(f"Got {len(points)} points")
    finally:
        lidar.stop()
        
    ```
    '''
    def __init__(self, specific_port=None, specific_freq=None):
        logging.info('Connecting to lidar...')

        ports = ydlidar.lidarPortList()
        port = "/dev/ttyUSB0" if specific_port is None else specific_port
        for key, value in ports.items():
            port = value

        freq = specific_freq if specific_freq is not None else 10.0

        self.laser = ydlidar.CYdLidar()
        self.laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
        self.laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 115200)
        self.laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
        self.laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
        self.laser.setlidaropt(ydlidar.LidarPropScanFrequency, freq)
        self.laser.setlidaropt(ydlidar.LidarPropSampleRate, 3)
        self.laser.setlidaropt(ydlidar.LidarPropSingleChannel, True)
        self.laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0)
        self.laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0)
        self.laser.setlidaropt(ydlidar.LidarPropMaxRange, 16.0)
        self.laser.setlidaropt(ydlidar.LidarPropMinRange, 0.08)
        self.laser.setlidaropt(ydlidar.LidarPropIntenstiy, False)

        self.scan = ydlidar.LaserScan()
        self.data_queue = queue.Queue(maxsize=10)  # буфер на 10 сканов
        self._stop_event = threading.Event()
        self._thread = None

        logging.info('Connected successfully')

    def _scan_loop(self):
        
        ret = self.laser.initialize()
        if not ret:
            logging.error("Failed to initialize lidar")
            return

        ret = self.laser.turnOn()
        if not ret:
            logging.error("Failed to turn on lidar")
            return

        while not self._stop_event.is_set():
            r = self.laser.doProcessSimple(self.scan)
            if r:
                # Если очередь полна — выбрасываем старый скан, кладём новый
                if self.data_queue.full():
                    try:
                        self.data_queue.get_nowait()
                    except queue.Empty:
                        pass
                self.data_queue.put(self.scan.points)  # или весь scan

        self.laser.turnOff()
        self.laser.disconnecting()

    def start(self):
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._scan_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._thread:
            self._thread.join()

    def get_scan(self, timeout=1.0):
        """Получить последний скан из основного потока."""
        try:
            return self.data_queue.get(timeout=timeout)
        except queue.Empty:
            return None
    
    