# asv_arov_navigation
To launch the navigation server:
```
ros2 launch asv_arov_navigation nav_controller_launch.py use_sim:=true
```
In a second terminal window run a test file (arov_nav_test, arov_nav_test2, asv_nav_test, asv_arov_control_server):
```
ros2 run asv_arov_navigation test_file_name
```
