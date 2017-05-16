# Vatroslav-ROS
TO RUN THE CODE:

1) install ROS Kinetic ( http://wiki.ros.org/kinetic/Installation/Ubuntu )
2) install Boost library ( $ sudo apt-get install libboost-all-dev )
3) install Loki library ( $ sudo apt-get install libloki-dev )
4) install Advantech CAN driver ( downloadt.advantech.com/download/downloadsr.aspx?File_Id=1-16BU145 )
5) compile ( $ catkin_make or $ catkin_make --force-cmake)
6) run ( $ rosrun ~/catkin_ws/src/VIV/launch/pok4.launch)
