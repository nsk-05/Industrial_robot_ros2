## TO build the repo
    clone the repo in directory {project_name}_ws
    cd {project_name}_ws
    rosdep install --from-paths src --ignore-src -r -y
    sudo apt install ros-humble-gazebo-dev ros-humblw-gazebo-ros ros-humble-gazebo-ros-pkgs ros-humble-navigation2 ros-humble-joint-state-publisher ros-humble-xacro ros-humble-pointcloud-to-laserscan
    colcon build
    . install/setup.bash

## To launch husky in warehouse environment
    export MAP_NAME="warehouse"
    ros2 launch bot_gazebo gazebo_husky.launch.py rviz:=false

## To launch simple robot in warehouse environment
    ros2 launch bot_gazebo gazebo.launch.py

## TO launch simple robot mapping stack
    ros2 launch bot_navigation mapping.launch.py

## To launch simple bot navigation stack
    export IS_AUTO_MAP=false
    ros2 launch bot_navigation navigation.launch.py rviz:=true

# For autonomous mapping

## start simulation
    export MAP_NAME="neo_workshop"
    ros2 launch bot_gazebo gazebo_husky.launch.py rviz:=false

## start autonomous mapping stack
    export IS_AUTO_MAP=true
    ros2 launch bot_navigation autonomous_mapping.launch.py rviz:=true

Autonomous mapping:
[autonomous_mapping.webm](https://github.com/nsk-05/Industrial_robot_ros2/assets/86995491/6f572a1e-dbeb-4cf8-98f8-3f4989b82174)


TODO 

    [X] create urdf for both diferntial and maccanum drive robot

    [X] Create husky with omniwheel control 

    [X] Add 3d lidar and frame in the husky robot

    [X] Launch the simulation with small warehouse environment

    [X] Create mapping and Navigation packages

    [X] Create mapping and Navigation for husky robot

    [X] Add autonomous mapping stack
    
    [ ] Fine tune mapping and navigation 


### Navigation planner and controller for omnidirectional robot

    Smac Planner

    DWB Planner
