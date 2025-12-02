# ROS2 framework of 2026 MRT rover

ENSURE THAT YOU SOURCE ONLY THE `mrt_ws` WORKSPACE AND NO OTHER WORKSPACE IS SOURCED.

Run `colcon build` in `mrt_ws` directory before using.

TODO: Fix interfaces, venv dependencies, usage of messages in packages, launch files.

## Foxglove instructions: 

Install foxglove from the .deb x64 package. Once installed go to terminal and run:

sudo apt install ros-humble-foxglove-bridge

ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765

Inside the foxglove program connect to ws://localhost:8765

All topics should be seen there.

