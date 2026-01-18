# SDK for EVA project copter controll

# How to run
* Prepare env
1. python3 -m venv venv - создание вируатльного окружения для питона
2. source venv/bin/activate - активация венва
3. https://github.com/YDLIDAR/YDLidar-SDK/blob/master/doc/howto/how_to_build_and_install.md - set up ydlidar
4. pip install -r requirements.txt - установка зависимостей
5. pip install catkin_pkg - без этого не работает лидар
6. sudo chmod 666 /dev/ttyUSB0  - включение порта

* Start service
1. python3 src/protectionService.py

# Tools
1. YD SDK: https://github.com/YDLIDAR/YDLidar-SDK/tree/master
  * API: https://github.com/YDLIDAR/YDLidar-SDK/blob/master/doc/YDLIDAR_SDK_API_for_Developers.md
  
  *  Protocol: https://github.com/YDLIDAR/YDLidar-SDK/blob/master/doc/YDLidar-SDK-Communication-Protocol.md
  