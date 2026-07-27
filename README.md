# SORT
Separation-Oriented Recognition &amp; Tagging 

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

### ROS2 Helpful Commands
```
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
```