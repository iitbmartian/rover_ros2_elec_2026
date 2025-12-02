# ROS2 framework of 2026 MRT rover

Do:

> python3 colcon_build.py

To colcon build in all packages (all package names must end in "controls")

## Drive Logic

package cmake drive_control_interfaces defines the DriveData message class containing:

*angle of each wheel wrt straight forward reference*

float64[] angle 

*speed of each wheel *

float64[] speed

*pwm (12 bit 0 to 4095) to each motor*

int16[] pwm 

*direction value, +1 if speed > 0, -1 if speed < 0 and =0 if speed = 0*

int8[] direction

*sys_check boolean*

bool sys_check

package py drive_control contains the drive logic, with explicit and pid control logic

## Foxglove instructions: 

Install foxglove from the .deb x64 package. Once installed go to terminal and run:

sudo apt install ros-humble-foxglove-bridge

ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765

Inside the foxglove program connect to ws://localhost:8765

All topics should be seen there.

