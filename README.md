# asv_arov_navigation
To launch the navigation server:
```
ros2 launch asv_arov_navigation nav_controller_launch.py use_sim:=true
```
Wait until you see:
```
[asv.basic_navigator]: Nav2 is ready for use!
[navigation_action_server]: ASV Nav2 Active
[arov.basic_navigator]: Nav2 is ready for use!
[navigation_action_server]: AROV Nav2 Active
```
In a second terminal window run one of the test files (arov_nav_test, arov_nav_test2, asv_nav_test, asv_arov_control_server) using:
```
ros2 run asv_arov_navigation test_file_name
```
