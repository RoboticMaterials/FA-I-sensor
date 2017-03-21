catkin build
source devel/setup.bash
echo Using '~/Arduino/libraries' as sketchbook directory.
rosrun rosserial_arduino make_libraries.py ~/Arduino/libraries/ force_proximity_ros
