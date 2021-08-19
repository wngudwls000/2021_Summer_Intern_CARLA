# 2021_Summer_Intern_CARLA_Team_A

## Folder Structure
  ```
  project_ws/
  │
  ├── config/ - configuration .ini files  
  ├── resources/ - resources such as map files  
  ├── scripts/ - shell script files for launch program  
  ├── src/ - source code  
  │   ├── app/ - applications
  │   │   ├── control/ - control package
  │   │   │   ├── control_node_1/
  │   │   │   │   ├── include/
  │   │   │   │   ├── launch/
  │   │   │   │   ├── src/
  │   │   │   │   ├── CMakeLists.txt
  │   │   │   │   └── package.xml
  │   │   │   │
  │   │   │   ├── control_node_2/
  │   │   │   └── ...
  │   │   │   
  │   │   ├── localization/ - localization package
  │   │   │   ├── localization_node_1/
  │   │   │   ├── localization_node_2/
  │   │   │   └── ...
  │   │   │   
  │   │   ├── mission/ - mission package
  │   │   │   ├── mission_node_1/
  │   │   │   ├── mission_node_2/
  │   │   │   └── ...
  │   │   │   
  │   │   ├── perception/ - perception package
  │   │   │   ├── perception_node_1/
  │   │   │   ├── perception_node_2/
  │   │   │   └── ...
  │   │   │   
  │   │   └── planning/ - planning package
  │   │       ├── planning_node_1/
  │   │       ├── planning_node_2/
  │   │       └── ...
  │   │
  │   ├── bsw/ - basic software
  │   │   ├── device/ - sensor device package
  │   │   ├── interface/ - sensor interface package
  │   │   ├── network/ - networks (can, can-fd, ...) package
  │   │   └── sim/ - simulation package
  │   │       ├── carla_ego_vehicle/
  │   │       ├── carla_manual_control/
  │   │       ├── carla_ros_bridge/
  │   │       └── scenario_runner/
  │   │
  │   ├── hmi/ - human machine interface package
  │   │   ├── launch/
  │   │   ├── resources/ - png, ... for display
  │   │   ├── src/
  │   │   ├── CMakeLists.txt
  │   │   ├── hmi.rviz
  │   │   └── package.xml
  │   │
  │   ├── rte/ - run time environment
  │   │   ├── ailab_msgs/
  │   │   │   ├── msg/ - msg files
  │   │   │   ├── CMakeLists.txt
  │   │   │   └── package.xml
  │   │   │   
  │   │   └── tf/
  │   │       ├── dae/ - dae file for display
  │   │       ├── launch/
  │   │       ├── src/
  │   │       ├── CMakeLists.txt
  │   │       └── package.xml
  │   │
  │   └── CMakeLists.txt
  │
  └── .catkin_workspace - init file about catkin workspace

  ```