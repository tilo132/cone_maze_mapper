# Cat Node

This is an updated and modified version of the official turtlebot3 node. It is used to communicate with the OpenCr-Board of the Turtlebot3. The node ist used to indirectly communicate with the onboard IMU and the motor controller. 

It was modified due to some major errors and bugs in the original node. 
:info: The bugs fixed here may already be fixed in the original node.

## Installation

To install the node, clone the repository into your workspace and build it.

```bash
cd workspace/src
git clone git@gitlab.ips.biba.uni-bremen.de:ftf-navi2/cat_node.git
cd ..
colcon build --packages-up-to cat_node
```

Make sure to source your workspace after building it.

```bash
source install/setup.bash
```

## Usage

To start the node, run the following command:

```bash
ros2 run cat_node turtlebot3_ros
```

or the launch file:

```bash
ros2 launch cat_node turtlebot3_launch.py
```

For more information about the possible parameters and configuration options:

- take a look at the original [turtlebot3_node](https://github.com/ROBOTIS-GIT/turtlebot3/tree/humble-devel) repository
- take a look at the included launch file in the launch folder: [turtlebot3_launch.py](launch/turtlebot3_launch.py)

## Hint

Doesn't work? Did you specify the correct USB port while running the node? Take a look at the included launch file.