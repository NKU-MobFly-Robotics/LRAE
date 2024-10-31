# LRAE

**LRAE** is a **L**arge-**R**egion-**A**ware **E**xploration method that enables a ground robot to autonomously explore uneven terrains.

To obtain high exploration efficiency, LRAE adopts a large-region-aware exploration route optimization strategy that prioritizes exploring large regions while also considering exploring nearby small regions. To safely and completely explore uneven terrains, LRAE fully introduces traversability information to extract unknown regions and assess exploration safety levels. The safety levels are then integrated into the design of the exploration strategy to ensure safe robotic exploration. We validate LRAE in various challenging simulation scenes and real-world wild uneven terrains. The results show that our method can safely explore uneven terrains and improve exploration efficiency by up to 45.3% compared with state-of-the-art methods.

<p align="center">
 <img src="image/f.png" width = "270" height = "278" />
 <img src="image/r.png" width = "500" height = "278" />
</p>


**Video Link**: [Video on Youtube](https://youtu.be/xePDPZluLes); [Video on Bilibili](https://www.bilibili.com/video/BV1g1SVYWEfw/?spm_id_from=333.999.0.0&vd_source=0e7c59dd804a18d9a9c201eafe9ac6e5)

**Related Paper**: [Paper on IEEE](https://ieeexplore.ieee.org/document/10734213)

Q. Bi, X. Zhang, S. Zhang, R. Wang, L. Li and J. Yuan, "LRAE: Large-Region-Aware Safe and Fast Autonomous Exploration of Ground Robots for Uneven Terrains," in IEEE Robotics and Automation Letters, doi: 10.1109/LRA.2024.3486229.

(If it is useful to you, please cite our paper and ⭐️ our code.)

## Prerequisites

1. LRAE has been tested on __Ubuntu 20.04 with ROS Noetic__, please run the following commands to install required dependencies or tools:
```bash
sudo apt-get install ros-noetic-gazebo-* \
ros-noetic-gazebo-ros-control* \
ros-noetic-controller-* \
ros-noetic-ros-controllers \
ros-noetic-ros-control \
ros-noetic-tf2-* \
ros-noetic-velodyne-* \
ros-noetic-robot-state-publisher* \
ros-noetic-joint-state-controller* \
ros-noetic-velocity-controllers* 
```

2. In addition, we recommend that you download [gazebo_models](https://github.com/osrf/gazebo_models) to the directory `~/.gazebo/models`.

## Build LRAE
Then simply clone and compile our package:
```bash
cd ${YOUR_WORKSPACE_PATH}/src
git clone git@github.com:NKU-MobFly-Robotics/LRAE.git
cd ../ 
catkin_make
```

## Known issues

1. ```
   resource not found: velodyne_description
   ROS path [0]=/opt/ros/noetic/share/ros
   ROS path [1]=/${YOUR_WORKSPACE_PATH}/src
   ROS path [2]=/opt/ros/noetic/share
   ```

   ```bash
   sudo apt-get install ros-noetic-velodyne-*
   ```

2. ```
   E: Unable to locate package ros-noetic-velodyne-*
   E: Couldn't find any package by glob 'ros-noetic-velodyne-*'
   E: Couldn't find any package by regex 'ros-noetic-velodyne-*'
   ```

   ```bash
   sudo apt-get update
   ```
   
4. ```
   [INFO] [1724678398.723761, 1733.140000]: Loading controller: scout_motor_rr_controller
   [ERROR] [1724678398.729870298, 1733.145000000]: Could not load controller 'scout_motor_rr_controller' because controller type 'velocity_controllers/JointVelocityController' does not exist.
   [ERROR] [1724678398.729950407, 1733.146000000]: Use 'rosservice call controller_manager/list_controller_types' to get the available types
   [ERROR] [1724678399.731316, 1733.943000]: Failed to load scout_motor_rr_controller
   ```

   ```bash
   sudo apt-get install ros-noetic-ros-controllers ros-noetic-ros-control
   ```

## Run LRAE in simulation

After the compilation is successful, you need to open two terminals and run the following two commands.

``` bash
source devel/setup.bash && roslaunch fitplane simulation_scene1.launch
```

``` bash
source devel/setup.bash && roslaunch lrae_planner exploration_scene1.launch
```

After running the above two commands, if you can see the **red route path**, the **purple exploration path**, the **green local path**, and the robot has begun moving to explore in the RVIZ interface, it means that LRAE has started successfully.

## Run LRAE in different scenes

We have designed four testing scenes with different characteristics and sizes：

<div style="display: flex; justify-content: center; gap: 10px;">
	<figure>
  		<img src="image/s1.png" style="zoom:32%;" />
   		<figcaption>Scene 1</figcaption>
    </figure>
    <figure>
  		<img src="image/s2.png" style="zoom:32%;" />
   		<figcaption>Scene 2</figcaption>
    </figure>
</div>

If you want to run LRAE in Scene 2, you need to modify the above two commands to：

```bash
source devel/setup.bash && roslaunch fitplane simulation_scene2.launch
```

```bash
source devel/setup.bash && roslaunch lrae_planner exploration_scene2.launch
```

For Scene 3 and Scene 4, the method is the same as above.
## Acknowledgements

We sincerely appreciate the following open source projects: [FAEL](https://github.com/SYSU-RoboticsLab/FAEL), [TARE](https://github.com/caochao39/tare_planner), [PUTN](https://github.com/jianzhuozhuTHU/putn), and Ji Zhang's [local_planner](https://github.com/jizhang-cmu/ground_based_autonomy_basic/tree/noetic/src/local_planner).
