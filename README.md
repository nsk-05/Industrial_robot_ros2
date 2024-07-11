## To launch husky in warehouse environment
    ros2 launch bot_gazebo gazebo_husky.launch.py

## To launch simple robot in warehouse environment
    ros2 launch bot_gazebo gazebo.launch.py

## TO launch simple robot mapping stack
    ros2 launch bot_navigation mapping.launch.py

## To launch simple bot navigation stack
    ros2 launch bot_navigation navigation.launch.py

# For autonomous mapping

## start simulation
    ros2 launch bot_gazebo gazebo_husky.launch.py rviz:=false

## start autonomous mapping stack
    ros2 launch bot_navigation autonomous_mapping.launch.py

Autonomous mapping:
[autonomous_mapping.webm](https://github.com/nsk-05/Industrial_robot_ros2/assets/86995491/6f572a1e-dbeb-4cf8-98f8-3f4989b82174)


TODO 

    [X] create urdf for both diferntial and maccanum drive robot

    [X] Create husky with omniwheel control 

    [X] Add 3d lidar and frame in the husky robot

    [X] Launch the simulation with small warehouse environment

    [ ] Modify industry environment according to the need'

    [X] Create mapping and Navigation packages

    [X] Create mapping and Navigation for husky robot

    [X] Add autonomous mapping stack
    
    [ ] Fine tune mapping and navigation 


### Navigation planner and controller for omnidirectional robot

    Smac Planner

    DWB Planner

    TEB Planner [X] Can't use TEB as it is not maintained for so long, so similar planner like mppi can be used
