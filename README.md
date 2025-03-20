**#DEMO1**
![Exit from_Right](https://github.com/ishapatel97/Maze-Solving-Python/blob/main/Exit_from_right.gif)  

**#DEMO2 with gmapping**

![Maze Exploration with GMapping](https://github.com/ishapatel97/Maze-Solving-Python/blob/main/maze_explore_with_gmapping.gif)  

# Installation

After downloading this package. Compile your workspace.

Install dependencies: sudo apt-get install ros-noetic-turtlebot3-description ros-noetic-turtlebot3-gazebo

Then, add the following lines in your ~/.bashrc file so that GAZEBO will load these models correctly.
Replace "your_workspace" with the name of your workspace and make sure the full path of the "maze_simulation" package is correct.

export GAZEBO_PLUGIN_PATH=path_to_your_workspace/src/maze_simulation/lib:${GAZEBO_PLUGIN_PATH}

export GAZEBO_MODEL_PATH=path_to_your_workspace/src/maze_simulation/models:${GAZEBO_MODEL_PATH}

export GAZEBO_RESOURCE_PATH=path_to_your_workspace/src/maze_simulation/models:${GAZEBO_RESOURCE_PATH}

export TURTLEBOT3_MODEL=burger

# Usage

There are two launch files you can use to launch the two different maze world files. 

roslaunch maze_simulation maze_world_1.launch 

roslaunch maze_simulation maze_world_2.launch 

# Utilities (Map Exploration Percentage)

There's a python script called "get_map_coverage.py" in the /src folder. This ROS node calculates the explored map area as per the exact map dimensions of the maze worlds. The node subscribes to the map via the '/map' topic and publishes the explored area as percentage in the topic 'map_coverage_percentage' (Float64).

You can run this node using the below command.

rosrun maze_solver get_map_covergae.py

You can consider a map fully explored if the map_coverage_percentage reaches >= 95%

