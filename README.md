# SORT
Separation-Oriented Recognition &amp; Tagging 

## Helpful Commands

### Bash
```
sudo ufw disable
export ROS_DOMAIN_ID=2

sudo apt install ros-$ROS_DISTRO-pinocchio
```

### ROS2

```

# old home position
ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'],
  points: [
    {
      positions: [-0.35875, -1.61249, -0.62015, -0.03378, -0.91811, 0.00466],
      velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      time_from_start: {sec: 5, nanosec: 0}
    }
  ]
}"

# new home position
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


ros2 interface show trajectory_msgs/msg/JointTrajectory
ros2 topic info /joint_trajectory_controller/joint_trajectory --verbose

ros2 node list
ros2 topic list
ros2 service list
ros2 action list
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