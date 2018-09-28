# Instructions

## To take time-lapse pictures with kinect

1. ``` $ roslaunch camera_setup start_cameras.launch ```
2. ``` $ python time_lapse.py ```

The default settings in time_lapse.py is to take pictures every 10 minutes.

## Move franka robot back and forth
1. ``` $ roslaunch panda_moveit_config demo.launch ``` for simulation or
   ``` $ roslaunch franka_control franka_control.launch robot_ip:= 172.16.2.0 load_gripper:=False/True ``` for real robot
2. ``` $ python moveRobot_backNforth.py ``` and see instructions on the terminal

