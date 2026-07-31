# SORT (Separation-Oriented Recognition &amp; Tagging) 

This is based for the second Kinova Arm in the corner of RoboLab with the arm hanging from the ceiling. At the time writing (30.07.2026), there is no gripper for this arm available.  


## Description

The goal of SORT is to develop a robot-human collaboration system that gives feedback for a zone base object sorting task.


### The Zone-based Sorting Task

A table is placed below the robot arm and divided into 3 zones by using April Tag markers in each corner. Additional 4 cubes with April Tags are given on the table. The cubes can then be placed onto the zones by the human.

The cubes are categorized by zones: 
- Cube 3 belongs to Paper Zone (100 - 103 )
- Cube 4 belongs to Plastik Zone (104 - 107)
- Cube 5 belongs to Restmüll (108 - 111)
- Cube 6 belongs to no zone (Unknown)

We defined overall outcomes for this task:
- successful placement: cube 3, 4 or 5 placed in the correct zone
- unsuccessful placement: cube 3, 4 or 5 is placed in the wrong zone
- uncertain placement: cube 6 is placed alone in any zone

### Goal of the Human Robot Interaction
SORT is supervising the zones while being ready to start its feedback loop. 
Feedback is triggered, once SORT recognizes a cube being placed into a zone.  
Overall the feedback contains the following components:  

1. affective feedback:
- light feedback
- gesture feedback
- voice feedback

### Simplified Architecture

In `vision_module` AprilTagDetector takes the Input of all connected cameras over the TopicHandler and detects the AprilTags with respect to the camera, which did. The `TagDetectorNode` then publishes `/vision/tag_detections/<camera>`, which we then use in `WorlSpaceNode` and return `vision/tags`.

`TagWorld` and `Zone` are used to determine the Pose of a AprilTag in 3d Space through triangulation and Zone creates a geometry that determine where a Zone is.


## Setup SORT

### Physical Setup in RoboLab
Place a table under the arm. Height:   
Put the zones (printed DinA4 sheets) on the table.   
Place the cubes 3 to 6 into the zones.

### Software Setup in RoboLab
before starting any nodes we need to change the domain for the second arm. Apply this in each terminal used. 
Do this for all PCs running any nodes, i.e.  RoboLab PC running the arm nodes and the VM running this projects nodes
``` 
export ROS_DOMAIN_ID=0
```
However while working with the other teams we use
``` 
export ROS_DOMAIN_ID=2
```
NOTE: in Domain 2 we don't have access to home assistant and tts
#### RoboLab PC
start all nodes for the arm and the second camera:

``` bash
# Terminal 1 KINOVA ARM 2

ros2 launch kinova_gen3_6dof_robotiq_2f_85_moveit_config robot.launch.py robot_ip:=10.163.18.199 use_fake_hardware:=false

# Terminal 2 KINOVA ARM 2 CAMERA

ros2 launch kinova_vision kinova_vision.launch.py launch_depth:=false device:=10.163.18.199

# Terminal 3 REALSENSE CAMERA

ros2 run realsense2_camera realsense2_camera_node
```

#### VM / PC running SORT (See Start Sort for simplified approach)
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

## Running SORT (See Start Sort for simplified approach)

once everything is built we need to start our nodes:

### Manually Starting All Nodes

``` bash
# Terminal 1 
ros2 run vision_module tag_detector

# Terminal 2 
ros2 run vision_module world_space

# Terminal 3
ros2 run control_module gesture_node

# Terminal 4 
ros2 run feedback_controller feedback_node

# Terminal 5 move arm into home position
ros2 topic pub --once /arm/gesture std_msgs/msg/String "data: 'home'"

```

### Startup Script on our VM (phri1)

You can use this instead of starting every node by hand and building and refreshing

```
cd ~/ros2_ws
chmod +x start_sort.sh
./start_sort.sh
```

### Manual Commands

Gesture Node
``` bash
# A) Plan-Only / Dry-Run (Safe - arm does NOT move, MoveIt planning only):
ros2 run control_module gesture_node --ros-args -p gesture:=nod -p execute:=false

# B) Execute actual physical motion on arm (nod, shake, tilt, search, home):
ros2 run control_module gesture_node --ros-args -p gesture:=home
ros2 run control_module gesture_node --ros-args -p gesture:=nod
ros2 run control_module gesture_node --ros-args -p gesture:=tilt
ros2 run control_module gesture_node --ros-args -p gesture:=shake
# or
ros2 topic pub --once /arm/gesture std_msgs/msg/String "data: 'nod'"
ros2 topic pub --once /arm/gesture std_msgs/msg/String "data: 'shake'"
ros2 topic pub --once /arm/gesture std_msgs/msg/String "data: 'tilt'"
ros2 topic pub --once /arm/gesture std_msgs/msg/String "data: 'search'"
ros2 topic pub --once /arm/gesture std_msgs/msg/String "data: 'home'"
```

Vision Node:

*NOTE*: does not work when running SORT via our startup script.

Manual Topic Pubs:
``` bash 

# movement
# home position hardcode
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

# tts
ros2 topic pub --once /tts/generate std_msgs/msg/String "{data: 'Test'}"

```