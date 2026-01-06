# Experimental Robotics Lab - Assignment 2
This ROS 2 package contains the simulation and control nodes for an autonomous robot searching for ArUco markers in an enviroment. It also contains PDDL files to plan the mission through PDDL planning (thanks to the plansys2 pkg).Developed for the Experimental Robotics Lab class, it allows you to run missions with either a 2-wheel or 4-wheel robot in Gazebo, using computer vision to detect targets and navigate the environment.

## Authors
- Luca Bricarello: 5248168
- Barbara Ratto: 4952202
- Francesca Magno: 5294873
- Martino Arecco: 5273621
- Filippo Gorini: 4828475

## Requirements
Ensure the following requirements are installed and configured before running the simulation:
 - **ROS2 distro**: Jazzy
 - **Gazebo**
 - **Gazebo Models**: the `aruco_marker` model must be installed in your Gazebo model path. To get the model you can clone this [repository](https://github.com/LucaBricarello/aruco_box.git).
 - **cv_bridge**
 - **OpenCV (with ArUco module enabled)**: required for ArUco marker detection (`cv2.aruco`)
 - **tf-transformations**: used to convert orientation data from quaternions to Euler angles.
 - **this package:** https://github.com/CarmineD8/plansys_interface.git (used for the get_plan_and_execute node)

## How to run the code
### 1. Installation and Compilation
1. Source your ROS 2 environment.
2. Clone this repository into the `src/` folder of your ROS 2 workspace.
3. Build your workspace using colcon:
	```
	cd ~/<your_workspace>
	colcon build
	source install/setup.bash
	```
Once you have correctly built and sourced your workspace, you can procede with the following steps to launch the simulation.

### 2. Run Plansys nodes
You can launch the domain expert, the problem expert, the planner and the executor thanks to a launch file, the command is:
```
ros2 launch erl_assignment_2 distributed_actions.launch.py
```
### 3. Launch the simulator with the chosen robot
You can run the simulation with either a **2-wheel robot** or a **4-wheel (skid steer) robot**.

You can launch the simulator with the **2-wheel robot** with this launch file:
```
ros2 launch erl_assignment_2 spawn_robot.launch.py
```
Or you can launch the simulator with the **4-wheel (skid steer) robot** with this other launch file:
```
ros2 launch erl_assignment_2 spawn_robot_skid_steer.launch.py
```
### 4. Launch the mission, retrieve the plan and execute it:
Now you can launch the node *get_plan_and_execute node* of the plansys_interface pkg to get the plan and execute it:
```
ros2 run plansys_interface get_plan_and_execute
```

### 5. Visualizing the Output
The simulation uses compressed images. To visualize the processed output or the robot's camera feed, use `rqt`:
- **Processed Image**: Subscribe to the topic `/robot/processed_image`.
- **Camera Feed**: View the raw camera topic via the Image View plugin.

> **Note:** A generic CV2 window displaying the modified image will also open automatically when the image is published.

## Video of the mission
Below you can find demonstrations of the mission running on the simulator.

**mission with 2 wheel robot**:

https://github.com/user-attachments/assets/aaf72079-fcda-44d0-8fc7-84eb9105f9e9

## Package Overview
This package is made of many different components, each one responsible for a different aspect, like: mission planning, mission execution, robot modeling, simulation setup and system launch orchestration.
Below is a brief rundown of the main components and of their functionalities.

### 1. Action Nodes (`/erl_assignment_2`)
The mission actions are implemented through three dedicated ROS 2 nodes, all located in the `erl_assignment_2/` folder. They represents the actions implemented in the PDDL code in the folder `pddl/`

* **`action_move`** (`action_move.py`)
    * Implements the **move** action used in the PDDL domain file. This nodes implements the logic to reach the waypoint selected through the PDDL planning.
* **`action_rotate`** (`action_rotate.py`)
    * Implements the **rotate_and_detect** action used in the PDDL domain file. The robot rotates untill he detects an aruco marker, it checks if it has already been found, if not it pubblishes it on a topic and adds it to the list of found markers.
* **`action_analyze`** (`action_analyze.py`)
    * Implements the **analyze_marker** action used in the PDDL domain file. While the previous actions are active it listens to the topic of the detected markers, it builds a list of the found marker ids and also saves the waypoint on which the ids are found. When all the previuos actions are terminated it starts (thanks to PDDL planning). When the actions starts it orders the list, and then visits all the waypoint to find the markers, take the picture, modify it with a circle and then pubblish it on a custom topic.

### 2. Robot Models (`/urdf`)
The `urdf/` folder contains the Universal Robot Description Format (URDF) files describing the two different robot model: 
* **2-Wheel Robot:** Standard differential drive chassis.
* **4-Wheel Robot:** Skid-steer chassis. (Updated with the lidar sensor)

### 3. Simulation World (`/worlds`)
Contains the custom Gazebo world file designed for this assignment.
> **Note:** You must ensure the `aruco_marker` model is correctly installed in your Gazebo model path for the world to load without errors.

### 4. Launch Files (`/launch`)
The `launch/` folder contains Python-based ROS the launch files used to coordinate and start the entire system. These launch files are responsible for:
* Selecting and spawning the appropriate robot model (2-wheel or 4-wheel).
* Launching the Gazebo simulation environment with the custom world.
* launching all the nodes of plansys2 pkg that we need.

### 5. PDDL Files (`/pddl`)
The `pddl/` folder contains the pddl files needed for the planning of the mission. Basically there is a *domain* file that describes all the possible actions that the robot can do (**assignment2_domain.pddl**), and also a *problem* file that describes the intial state of the mission and the goal to be reached (**assignment2_problem.pddl**).

## Other details
### Obstacle avoidance
An obstacle avoidance algorithm has been develeped to move in the enviroment without colliding into the walls. This obstacle avoidance algorithm is an Artificial Potential Field that works thanks to the subscription to the scan topic (lidar sensor). Pay attention, this algorithm could stuck the robot into a local minimum (the implemented version for now does not include noise to try to avoid this problem).

