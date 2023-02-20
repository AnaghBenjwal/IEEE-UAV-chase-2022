
# IEEE UAV Chase 2022 Competition

This repository is our team's submission for the IEEE UAV Challenge 2022. The simulator uses ROS Melodic and Gazebo 9, running on Ubunutu 18.04.06 LTS, we have also succesfully tested it on ROS Noetic with Gazebo 11 on Ubuntu 20.04 LTS.

The steps are a continuation of the instructions provided in the uavcc-simulator repository: https://github.com/cam2-auto-drone/uavcc-simulator

## Trial 1 simulator

### Setting up the workspace

1. The directory ```px4vision_realsense``` containing ```model.config``` and ```px4vision_realsense.sdf``` should be placed in path ```~/(dir)/Firmware/Tools/sitl_gazebo/models```

2. The directory ```realsense_camera``` containing ```model.sdf``` and ```model.config``` should also be placed in path ```~/(dir)/Firmware/Tools/sitl_gazebo/models```

3. The launch file ```trial_1.launch``` is to be placed in path ```~/catkin_ws/src/avoidance/avoidance/avoidance/launch```

4. To set the spawn location of px4vision_realsense model, change line 43 in the launch file ```avoidance_sitl_mavros.launch``` to 
```
args="-sdf -x -8.7 -y 0 -z 0 -database $(arg model) -model $(arg vehicle)">

```
### Running the python script for trial 1

1. ```cd ~/catkin_ws/```

2. ```roslaunch avoidance trial_1.launch```

3. In a new terminal window run ```python3 trial_1.py```

## Trial 2 simulator

### Changes to be made for the trial 2 simulator

1. Follow "setting up the workspace" for trial 1 until step 2

2. Place the launch file ```trial_2.launch``` in the path ```~/catkin_ws/src/avoidance/avoidance/avoidance/launch```

4. To set the spawn location of px4vision_realsense model, change line 43 in the launch file ```avoidance_sitl_mavros.launch``` to 
```
args="-sdf -x -0 -y 0 -z 0 -database $(arg model) -model $(arg vehicle)">

```

### Running the python script for trial 2

1. ```cd ~/catkin_ws/```

2. ```roslaunch avoidance trial_2.launch```

3. In a new terminal window run ```python3 trial_2.py```

## Note

1. Please ensure that the path variables inside ```trial_1.launch``` and ```trial_2.launch``` are set according to your workspace.
