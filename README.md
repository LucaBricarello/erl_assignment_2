# Requirements

for now it is needed this package:
https://github.com/CarmineD8/plansys_interface.git

# Launch instructions

launch the following commands in order:

  - ros2 launch erl_assignment_2 distributed_actions_2.launch.py
  - ros2 launch erl_assignment_2 spawn_robot.launch.py
  - ros2 run plansys_interface get_plan_and_execute

# Development status

planning works

plan execution works (for now with get_plan_and_execute node of plansys_interface package, it is in C++, python version in our pkg for now it is not working)

all move actions work (note, using the filtered odometry of the ekf node, normal odometry diverges to fast)

all rotate and detect actions work, they pubblish the detected id on a topic so that the analyze marker action can retrieve the markers id

analyze marker action for now not working
