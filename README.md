# FT_300s_ur3e
- This repository is equipped with robotiq's FT 300-S Force Torque Sensor and Hande on ur3e.

- ubuntu18.04, melodic, python2

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
- Each is launched in a separate terminal.'''docker exec -it ${docker_container_id} bash or /Dockre/exec.sh'''

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


## Motion trajectory
- A file "xxx.npy" is generated in the directory where plotting_2d.py was executed. Put the file in /space_data, change the loadfile name of data_plot.py and run it, and the following graph will be generated.

![trajectory](https://raw.github.com/wiki/shumpe-m/FT_300s_ur3e/images/motion.png)