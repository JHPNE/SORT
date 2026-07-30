# SORT (Separation-Oriented Recognition &amp; Tagging) 


## Description

The goal of SORT is to develop a robot-human collaboration system that gives feedback for a zone base object sorting task. This is based on the Kinova Arm 2 on in the RoboLab

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

TODO: keep in mind that the robot might also not recognize the zones and is uncertain because the cubes or anything blocks the zone detection.


## Helpful Commands

### Bash
```
sudo ufw disable
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