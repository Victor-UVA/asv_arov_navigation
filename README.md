# asv_arov_navigation
To launch the navigation service:
```
ros2 launch asv_arov_navigation nav0_launch.py use_sim:=true
```
Use rqt to call the /control_mode service to set the control mode. 0 to STOP, 1 to start the demo, and 2 to drive to home pose
