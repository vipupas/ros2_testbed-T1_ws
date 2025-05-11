# 🚀 NAVIGATION STACK FOR ROS2(nav2)

In documentation summarizes the approach, configuration, encountered errors, and their solutions while setting up and debugging the Nav2 stack with Gazebo simulation and rviz visualisation using a custom robot model 'Testbed-T1.0.0' provided by ERIC-Robotics. 

## 📁 File Structure :

```
assignment_ws/
└──src
    ├── testbed_description/
    │   ├── launch/            # Launch the full base simulation
    │   ├── meshes/
    │   ├── rviz/            # RVIZ configuration files
    │   └── urdf/            # URDF files for Testbed-T1.0.0
    ├── testbed_gazebo/
    │   ├── worlds/            # Simulation world files
    │   ├── launch/            # Launch files for Gazebo
    │   └── models/            # Misc. Gazebo model files
    ├── testbed_bringup/
    │   ├── launch/            # Launch file for bringing up the robot
    │   └── maps/              # Predefined map of the test environment
    └── testbed_navigation      
    |   ├── CMakeLists.txt
    |   ├── config                # navigation configuration files
    |   │   ├── map_params.yaml    # parameter file for map loading from testbed_bringup/map
    |   │   ├── amcl_params.yaml   # parameter file for amcl 
    |   │   └── nav2_params.yaml   # parameter file for navigation stack
    |   ├── include
    |   │   └── testbed_navigation
    |   ├── launch                      # launch navigation files
    |   │   ├── localization.launch.py   # launch amcl
    |   │   ├── map_loader.launch.py     # launch map in rviz
    |   │   └── navigation.launch.py     # launch navigation stack for poses
    |   ├── package.xml
    |   └── src
    ├── help.md                # Guidelines and FAQs
    └── README.md              # Instructions for the assignment
```

## Instructions

### 1. Setting Up the Repository
1. Create your workspace:
    ```bash
    mkdir -p ~/your_ws/src  # you can add any name to your workspace 
    ```
2. Clone this repository:
   ```bash
   cd ~/your_ws/src
   git clone https://github.com/vipupas/l1-ros-internship-assignment-vipupas
   ```
2. Build the workspace:
   ```bash
   cd ~/your_ws/
   colcon build --symlink-install
   source install/setup.bash
   ```
### <u>Note :- Make sure you source the file in every terminal</u>
   ```bash
   cd ~/your_ws/
   source install/setup.bash
   ```
   


### 2. Launching the Simulation Environment
1. Start the full simulation using (In terminal 1):
   ```bash
   ros2 launch testbed_bringup testbed_full_bringup.launch.py
   ```
   This brings up the testbed environment in Gazebo and RViz.

### 3. There can be Error in Nav2 like Obstacles Detections or can be Timeout Error
   ```bash
   sudo apt install ros-humble-rmw-cyclonedds-cpp
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  # you can add this line in .bashrc file
   ```
   Solve by Robotics Back-End 
   🎥 [Nav2 Stack ](https://www.youtube.com/watch?v=idQb2pB-h2Q)

### 4. Launching the Navigation Environment
1. loading the map in RViz (In terminal 2):
   ```bash
   ros2 launch testbed_navigation map_loader.launch.py
   ```
2. AMCL Localization (In terminal 3):
   ```bash
   ros2 launch testbed_navigation localization.launch.py
   ```
3. Bring up all Navigation Plugins (In terminal 4):
   ```bash
   ros2 launch testbed_navigation navigation.launch.py
   ```

### 5. Setup in RViz

1. set fixed frame to map under Global Option in Display.

#### Add all necessary topic in display :
1. /global_costmap
2. /local_costmap
3. /map




### 6. In RViz
##### It is setting initial postion
1. Click <b>"2D Pose Estimate"</b> and click on the robot's actual location as in gazebo (<i>make sure lidar reading match to map</i>) .<br>

##### It is setting goal position</i><br>
2. Click <b>"2D Nav Goal"</b> and send a goal. <br>








## 📸 Screenshot of Mapping and Navigation

![Alt text](https://drive.google.com/uc?export=view&id=1LHk8WTRL5HdcZTzSj-gaP8OznUH6E_v4)

![My Image](https://drive.google.com/uc?export=view&id=1_dFsNoizNnaZ6JeIvqlicO8lR5sjNjck)

![My Image](https://drive.google.com/uc?export=view&id=1PbK6mIRoEZ41SSjgetR_sZwTW7eLxj-J)

![My Image](https://drive.google.com/uc?export=view&id=1hMnEhx57BWi8e9p3P-hSxxROy9YBlbKM)

![My Image](https://drive.google.com/uc?export=view&id=13FGGri-nFWW10j5mvzR9D54-ce5WAXza)


## 🎥 Navigation and Controlling Video

### teleop_twist_keyboard node:

🎥 [Controlling Bot ](https://drive.google.com/file/d/1dwnaQwllifPxBoEc0HQbY5RkaxKrNQre/view?usp=sharing)

### setting up initial pose 

🎥 [Setting initial pose](https://drive.google.com/file/d/1LgQxTDy75NkQk7VN8Q5iJtMuHYdsKMZp/view?usp=sharing)

### setting the goal pose using amcl

🎥 [Setting goal pose](https://drive.google.com/file/d/1Urr5HeDguKU2DjMAVY6TZ0ErYSXLyRcz/view?usp=sharing)

### giving pose on map for navigation

🎥 [Map and move](https://drive.google.com/file/d/11THFsZmtFio29IFrXelIimZbQpUkknf5/view?usp=sharing)

### Using costmap for navigation

🎥 [Costmap and set pose](https://drive.google.com/file/d/1p0EhRLeDU2g9-f39S8MgZAcMqHkqtWnD/view?usp=sharing)


