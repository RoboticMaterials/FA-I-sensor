(cd src; git clone https://github.com/ros-drivers/rosserial.git )
catkin build
source devel/setup.bash
echo Using '~/Arduino/libraries' as sketchbook directory.
rm -rf ~/Arduino/libraries/ros_lib
rosrun rosserial_arduino make_libraries.py ~/Arduino/libraries/ force_proximity_ros
