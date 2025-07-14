# asv_arov_navigation
To launch the nav2 based navigation server:
```
ros2 launch asv_arov_navigation nav_controller_launch.py use_sim:=true
```
In a second terminal window run a test file (arov_nav_test, arov_nav_test2, asv_nav_test, asv_arov_control_server):
```
ros2 run asv_arov_navigation test_file_name
```
To launch the PID based navigation service:
```
ros2 launch asv_arov_navigation nav0_launch.py use_sim:=true
```
Use rqt to call the /control_mode service to set the control mode. 0 to STOP, 1 to start the demo, and 2 to drive to home pose
