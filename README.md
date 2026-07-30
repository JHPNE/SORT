# SORT (Separation-Oriented Recognition &amp; Tagging) 

This is based for the second Kinova Arm in the corner of RoboLab with the arm hanging from the ceiling. At the time writing (30.07.2026), there is no gripper for this arm available.  


## Description

The goal of SORT is to develop a robot-human collaboration system that gives feedback for a zone base object sorting task.


### The Zone-based Sorting Task

A table is placed below the robot arm and divided into 3 zones by using April Tag markers. Additional 4 cubes with April Tags are given on the table. The cubes can then be placed onto the zones by the human.
The cubes are categorized by zones: 
- Cube 1 belongs to zone 1
- Cube 2 belongs to zone 2
- Cube 3 belongs to zone 3
- Cube 4 belongs to no zone 

We defined overall outcomes for this task:
- successful placement: cube 1, 2 or 3 is placed in the correct zone
- unsuccessful placement: cube 1, 2 or 3 is placed in the wrong zone
- uncertain placement: cube 4 is placed in any zone

NOTE: keep in mind that the robot might also not recognize the zones and is uncertain because the cubes or anything blocks the zone detection.

### Goal of the Human Robot Interaction
SORT is supervising the zones while being ready to start its feedback loop. 
Feedback is triggered, once SORT recognizes a cube being placed into a zone.  
Overall the feedback contains the following components:  
1. affective feedback:
- light feedback
- gesture feedback
- voice feedback

*(if gripper is available)*  
2. cognitive feedback: 
- correction of the cube into the correct zone by the robot arm

## Setup

### Physical Setup in RoboLab
Place a table under the arm. Height:   
Put the zones (printed DinA4 sheets) on the table.   
Place the cubes 1 to 4 next to the zones.

*if we add human tracking for movements*:  
Human holds additional april tag. 

### Software Setup in RoboLab
before starting any nodes we need to change the domain for the second arm. Apply this in each terminal used. 
Do this for all PCs running any nodes, i.e.  RoboLab PC running the arm nodes and the VM running this projects nodes
``` 
export ROS_DOMAIN_ID=2
```

#### RoboLab PC
start all nodes for the arm:

``` bash
# Terminal 1 KINOVA ARM 2

ros2 launch kinova_gen3_6dof_robotiq_2f_85_moveit_config robot.launch.py robot_ip:=10.163.18.199 use_fake_hardware:=false


# Terminal 2 KINOVA ARM 2 CAMERA

ros2 launch kinova_vision kinova_vision.launch.py launch_depth:=false device:=10.163.18.199

# Terminal 3 REALSENSE CAMERA

ros2 run realsense2_camera realsense2_camera_node
```

#### VM / PC running SORT
before starting any SORT related node, we need to build the packages first:
``` bash
# make sure you are in the correct workspace when building
# on our vm phri1 we should be in ~/ros2_ws
cd ~/ros2_ws

colcon build
# or 
colcon build --packages-select <package-name>

source install/setup.bash
```

in case you build it in the wrong folder we can remove it by using:
``` bash
rm -rf build install log
```

once everything is built we need to start our nodes:

``` bash
# Terminal 1 
ros2 run vision_module tag_detector

# Terminal 2 
ros2 run vision_module world_space

# Terminal 3
ros2 run control_module gesture_node

# Terminal 4 
ros2 run feedback_controller feedback_node

```

#### Manual Commands

``` bash
# Gesture Node

# A) Plan-Only / Dry-Run (Safe - arm does NOT move, MoveIt planning only):
ros2 run control_module gesture_node --ros-args -p gesture:=nod

# B) Execute actual physical motion on arm (nod, shake, search, home):
ros2 run control_module gesture_node --ros-args -p gesture:=home -p execute:=true
ros2 run control_module gesture_node --ros-args -p gesture:=nod -p execute:=true
ros2 run control_module gesture_node --ros-args -p gesture:=shake -p execute:=true -p velocity_scaling:=0.50


```

# Team Notes
## Helpful Commands

### Bash
```
sudo ufw disable

# mover arm to home position manually
ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'],
  points: [
    {
      positions: [-0.3442678993789787, -1.788839445251826, -0.05060219124037779, 0.033443887503647206, -1.9649835829107563, 0.00582217572740141],
      velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      time_from_start: {sec: 5, nanosec: 0}
    }
  ]
}"
```

### ROS2

```
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
```

## Connecting to the lab PC

The lab PC runs on domain 0. **Every new terminal** needs this before any `ros2` command,
otherwise you will not see the lab nodes (`/tts_node`, `/homeassistant_node`, ...) and
anything you publish silently goes nowhere:

```
export ROS_DOMAIN_ID=0
source install/setup.bash
```

Check it worked - `/tts_node` must appear in the list:
```
ros2 node list
```

To make it permanent, add it to your `~/.bashrc`: maybe leave it and only export domain id
```
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
```

## ROS2 Basics
### 1. Creating a package for ros2:
Note: package names should start with a lower case letter and only contain lower case letters, digits, underscores, and dashes.

```
ros2 pkg create --build-type ament_python --license Apache-2.0 <package-name>
```  
`build-type` sets the folder structure to the one we use  
`license` use any license to not get warnings. we can change them at a later point.

This creates a package folder with folders and files for you. The most relevant for us are:   
- `package.xml`: add info about description, maintainer and license. add package imports in `<exec_depend>`
- `<package-name>` folder: add source code for your package in here
- `setup.py`: match the `maintainer, maintainer_email, description and license fields` to your `package.xml`. Add the entry points of your package.  
    ``` 
    entry_points={
            'console_scripts': [
                    'talker = py_pubsub.publisher_member_function:main',
            ],
    },
    ```

Might also get relevant for us:
- `test` folder: maybe someone should look into how to test packages

### 2. Building a Package
```
colcon build --packages-select <package-name>
```

### 3. Run a Package
```
source install/setup.bash
ros2 run <package-name> <package-entry-point>
```

### Services
Services are done similarly but are run with a `launch` tag. We will figure out how this is done once we need it. 