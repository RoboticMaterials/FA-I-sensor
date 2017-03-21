*******************
** BUILD DEPENDS **
*******************

Depends on rosserial-arduino and rosserial. In indigo can be installed with

sudo apt-get install ros-indigo-rosserial-arduino
sudo apt-get install ros-indigo-rosserial


*******************
**  RUN DEPENDS  **
*******************

Requires the following to be able to echo topic. Port and Baud may vary.

rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=9600 