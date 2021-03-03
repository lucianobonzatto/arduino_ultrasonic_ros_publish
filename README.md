# arduino_ultrasonic_ros_publish
This repository aims to utility multiple ultrasonic sensors in a Arduino and publish the ranges in a ROS topic.

Sources:
1) https://www.intorobotics.com/how-to-use-sensor_msgs-range-ros-for-multiple-sensors-with-rosserial/
2) https://github.com/surabhi96/Library-navigating-robot/wiki/Ultrasonic-sensor-with-ROS


------------------------------------------------------------------------------------------
STEP BY STEP TO USE WITH ROS:
------------------------------------------------------------------------------------------

1 - Upload the code to your arduino


2 - Run ROSCORE on a terminal

  OBS: For each new open terminal run
    $ cd ~/catkin_ws/
    $ source devel/setup.bash
  
  Open ROSCORE
  $ roscore


3 - Run this command to find out which of the serial ports the Arduino is connected to
  % ls /dev/ttyACM* (New terminal)
  or
  % ls /dev/ttyUSB*


4 - Run the rosserial_python
  $ rosrun rosserial_python serial_node.py /dev/ttyACM0


5 - List active topics
  $ rostopic list


6 - Run rostopic to view what's being posted on ROS topics
  $ rostopic echo ultrasonic_1 (New terminal)
  $ rostopic echo ultrasonic_2 (New terminal)
  $ rostopic echo ultrasonic_3 (New terminal)
  ...
  
7 - To save datas in bagfile
  $ cd rosbag ...
  $ rosbag record -a
 
8 - To convert rosbag to csv
  $ rosrun rosbag_to_csv rosbag_to_csv.py



------------------------------------------------------------------------------------------
PINS - COMMON TRIGGER
------------------------------------------------------------------------------------------

 --- TRIGGER PINS ---
 - Sensor 1, 2, 3, 4, 5 e 6 são ligados em comum ao pino 4 do Arduino


 --- ECHO PINS ---

 - Sensor 1 (Placa) - Pin 02 (Arduino) - FE (Robô)
 - Sensor 2 (Placa) - Pin 03 (Arduino) - FC (Robô)
 - Sensor 3 (Placa) - Pin 18 (Arduino) - FD (Robô)
 - Sensor 4 (Placa) - Pin 19 (Arduino) - TD (Robô)
 - Sensor 5 (Placa) - Pin 20 (Arduino) - TC (Robô)
 - Sensor 6 (Placa) - Pin 21 (Arduino) - TE (Robô)

