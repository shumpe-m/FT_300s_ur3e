# FT_300s_ur3e
- This repository is equipped with robotiq's FT 300-S Force Torque Sensor and Hande on ur3e.

- ubuntu20.04, noetic, python3

- reference https://github.com/cambel/ur3.git

## Docker set up
- Docker environment building.
```
git clone https://github.com/shumpe-m/FT_300s_ur3e.git
cd FT_300s_ur3e/Docker
./build.sh
./run.sh
```

## Examples

![demo](https://raw.github.com/wiki/shumpe-m/FT_300s_ur3e/images/pick_and_place.gif)
- Each is launched in a separate terminal.'''bash run.sh''

- Start the simulator (Gazebo).
```
roslaunch ur3e_ft300s_gazebo ur_hande_ft300s.launch
```

- Start rviz.
```
roslaunch ur_hande_ft300s_moveit_config start_moveit.launch
```
- Run rviz_setup.py to limit the orbit range of rviz.
- Use moveit to control the trajectory of the UR. Run move.py.
```
rosrun ur_control_scripts rviz_setup.py 
rosrun ur_control_scripts main.py 
```


![jig](https://raw.github.com/wiki/shumpe-m/FT_300s_ur3e/images/jig.gif)



- plotting_2d.py is plotting based on information obtained from /gazebo/link_states. No startup required.
```
rosrun ur_control_scripts plotting_2d.py 
```


## Usage with real Hardware
"""
roslaunch ur_calibration calibration_correction.launch   robot_ip:=192.168.1.103 target_filename:="${HOME}/my_robot_calibration.yaml"
"""
"""
roslaunch ur_robot_driver ur3e_bringup.launch robot_ip:=192.168.1.103 kinematics_config:=/root/my_robot_calibration.yaml
"""

