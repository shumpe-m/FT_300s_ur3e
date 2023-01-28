# ur3e_tutorial

- ubuntu18.04, melodic

- reference https://github.com/cambel/ur3.git
## Set up
- Dockerによる環境構築
```
git clone https://github.com/shumpe-m/ur3e_tutorial.git
cd ur3e_tutorial/Docker
./build.sh
./run.sh
```
## Set up on desk
![demo](https://raw.github.com/wiki/shumpe-m/ur3e_tutorial/images/motion.gif)
- 4つのターミナルでそれぞれコマンドを使用。(docker exec -it ${docker_container_id} bash)

```
roslaunch ur_gazebo_motion_range ur_gripper_hande.launch
```

```
roslaunch ur_hande_moveit_config start_moveit.launch
```

```
rosrun ur_motion_range move.py 
```

```
rosrun ur_motion_range plotting_2d.py 
```


## Motion trajectory
- plotting_2d.pyを実行したディレクトリに "xxx.npy"ファイルが生成される。/space_data内にそのファイルを入れて、data_plot.pyのloadfile名を変更して実行すると次のグラフが生成される。

![demo](https://raw.github.com/wiki/shumpe-m/ur3e_tutorial/images/motion.png)