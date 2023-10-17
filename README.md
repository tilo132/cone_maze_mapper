# Cat

This is the main repository for the Cat vehicle project. It mainly is a wrapper for all necessary repositories needed to run the vehicle and also contains many handy scripts and launch files to control or interact with the vehicle.

## Prerequisites

### ROS2 (Humble)

#### Install on System
The vehicle was developed and tested for ROS2 Humble, though it may be possible to run the code on newer versions of ROS2 as well. To install ROS2 Humble, follow the instructions on the [ROS2 Wiki](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

#### Install in Docker

Or run ros2 inside a docker container:
```bash
docker run -it --net=host --name=cat ros:humble
```

To be able to use rviz2, you need to enable X11 forwarding. 

There are many ways to do this, but one of the easist is to use SSH X11 forwarding. To do this, install and setup ssh inside the docker container. Then add the public key of your host machine to the authorized keys of the docker container. Now you should be able to connect to your docker container via ssh with X11 forwarding enabled using:
```bash
ssh -X root@172.17.0.1
```
(make sure username and ip match your docker container configuration)

To make your life easier, you can add this to your ssh config:
```bash
sudo nano ~/.ssh/config
```

and add the following content:
```bash
Host cat
    HostName 172.17.0.1
    User root
    ForwardAgent yes
    ForwardX11 yes
```

Now you can connect to your docker container using:
```bash
ssh cat
```

### Dependencies

This repository heavily depends on various ROS2 packages. Most notably being:

| Package | Description | Needs manual installation | Installation via apt |
| --- | --- | --- | --- |
| [nav2](https://github.com/ros-planning/navigation2) | Navigation stack | yes | `apt install ros-humble-navigation2`|
| [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) | SLAM algorithms | yes | `apt install ros-humble-slam-toolbox`|
| [robot_localization](https://github.com/automaticaddison/robot_localization) | EKF | yes | `apt install ros-humble-robot-localization`|
| [tf2](https://github.com/ros2/geometry2) | Transformations | yes | `apt install ros-humble-geometry2`|
| [path_planning](https://github.com/Beomar97/path-planning), <br> [our custom build](https://gitlab.ips.biba.uni-bremen.de/ftf-navi2/path_planning) | FSG mode path planning algorithms | no | This project uses a modified version of that repository. It needs to be custom build. See [Installation](##Installation) |
| [m-explore-ros2](https://github.com/robo-friends/m-explore-ros2) | Logistic mode path exploration algorithm | yes | See [here](https://github.com/robo-friends/m-explore-ros2#Building) |
| [turtlebot3_node](https://github.com/ROBOTIS-GIT/turtlebot3/tree/humble-devel), <br> [our custom build](https://gitlab.ips.biba.uni-bremen.de/ftf-navi2/cat_node) | Turtlebot3 node | no | This project uses a modified version of that repository. It needs to be custom build. See [Installation](##Installation) |

## Installation

### From Source

Clone the latest version of this repository:
```bash
git clone https://github.com/tilo132/cone_maze_mapper.git
```

Make sure to source the ROS2 installation:
```bash
source /opt/ros/humble/setup.bash
```

Build the workspace:
```bash
cd ftf-v1
colcon build
```

This will build all necessary packages needed to run the vehicle. 
[!WARNING]
These packages do not include the simulation stack for cat! For that, see [cat_sim](https://gitlab.ips.biba.uni-bremen.de/ftf-navi2/cat_sim).

### Tipps

If the build process fails due to missing dependencies, install them via apt. In general the package name is `ros-humble-<package-name>`. For example when missing `slam_toolbox`:
```bash
apt install ros-humble-slam-toolbox
```

If that doesn't work your may find it by searching for the package:
```bash
apt list ros-humble-*<package-name>*
```

## Usage

This repository contains many useful scripts and launch files to control the vehicle.

### Launch Files

The launch files are located in the `launch` folder. The most important one is the `ftf.py` launch file. It launches all necessary nodes to run the vehicle. If you just want to launch a subset of the nodes:

| Launch File | Description |
| --- | --- |
| `ftf.py` | Launches all necessary nodes to run the vehicle |
| `nav2.py` | Launches the custom configured navigation stack (including the custom costmap `wall_layer`. See [cat_nav2](https://gitlab.ips.biba.uni-bremen.de/ftf-navi2/cat_nav2)) |
| `kitty.py` | Launches all nodes necessary to get `Cone` and `Poi` positions in the `camera`-frame (No SLAM / `map`- or `odom`- frame) |
| `robot_localization.py` | Launches the `robot_localization` node to estimate the `odom`-frame using a custom configured ekf-filter |
| `slam_offline.py` | Launches the custom configured `slam_toolbox` node to perform SLAM and estimate the `map`-frame.|
| `logistic.py` | Launches all nodes necessary to stabilize and transform the data from `navigation.py` using `slam_offline.py` into the `map`-frame. All parameters are optimized for the `Logistic`-mode of the vehicle |
| `bremergy.py` | Launches all nodes necessary to stabilize and transform the data from `kitty.py` using `slam_offline.py` into the `map`-frame. All parameters are optimized for the `FS`-mode of the vehicle |

### Running on multiple machines

It may be necessary due to harsh computational requirements to run the nodes on multiple machines (i.e. when running on Raspberry PIs). We support this approach. We offer launch files dedicated splitting the workload in two:

| Machine | Launch File |
| --- | --- |
| Machine 1 (Codename `Cat`) | `ftf.py` |
| Machine 2 (Codename `Kitty`) | `kitty.py` |

Make sure to have cloned and build the repository on both machines.

### Enabling Autostart

It may be useful to automatically start the launch-files / nodes on the vehicle when the machine is booted. To do this, we offer a systemd service file. To enable it, create a `cat.service` file in `/lib/systemd/system/`:
```bash
sudo nano /lib/systemd/system/cat.service
```

Paste the following content into the file.
Make sure to replace `<workspace>` with the path to the cloned instance of this repository and `<config>` with the desired workspace configuration (See [workspace.bash](###Utilities)):
```bash
[Unit]
Description=Cat Service

[Service]
Type=simple
ExecStart=/bin/bash -c "source source <workspace>/tools/workspace.bash <config> && ros2 launch <workspace>/launch/ftf.py"

[Install]
WantedBy=multi-user.target
```

and enable it using:
```bash
sudo systemctl enable cat.service
```

In the case your running the nodes on multiple machines, you need to create a `kitty.service` file on the second machine as well. Make sure to specify the correct launch file for both machines as listed in the table above.

Still having trouble? Take a look at this more in depth [tutorial](https://www.linode.com/docs/guides/start-service-at-boot/).

### Discovery Servers

It may be advantageous to use a discovery server to decrease the network load, by reducing the amount of discovery messages sent drastically. We recommend using [fast-dds](https://fast-dds.docs.eprosima.com/en/latest/fastdds/overview.html) as a discovery server solution. Start it using:
```bash
fastdds discovery -i 0
```

Make sure to notify ROS about the discovery server by setting the `ROS_DISCOVERY_SERVER` environment variable:
```bash
export ROS_DISCOVERY_SERVER=<ip-of-discovery-server>:11811
```

also, if you are using one, add it to your `super_client_configuration_file.xml`.

Take a look at our default [super_client_configuration_file.xml](https://gitlab.ips.biba.uni-bremen.de/ftf-navi2/ftf-v1/-/blob/main/tools/super_client_configuration_file.xml?ref_type=heads) as reference.

For a more in depth tutorial, see [this](https://fast-dds.docs.eprosima.com/en/latest/fastdds/ros2/discovery_server/ros2_discovery_server.html#setup-discovery-server).

When running on mutiple machines with multiple network interfaces being used on the discovery server machine, multiple discovery servers bridging the different networks may be necessary. For this to work you have to launch the discovery server with individual indices for each network interface. Also make sure to use the correct ip of the device:
```bash
fastdds discovery -i 0 -l <interface1-ip-of-device> -p 11811
fastdds discovery -i 1 -l <interface2-ip-of-device> -p 11812
```

Make sure to now also specify both servers in the order specified by the given indices in the `ROS_DISCOVERY_SERVER` environment variable:
```bash
export ROS_DISCOVERY_SERVER="[<ip-of-discovery-server1>:11811,<ip-of-discovery-server2>:11812]"
```

and adjust your `super_client_configuration_file.xml` accordingly.

### Utilities

We offer a few utilities to make your life easier:

| Utility | Description |
| --- | --- |
| [workspace.bash](https://gitlab.ips.biba.uni-bremen.de/ftf-navi2/ftf-v1/-/blob/main/tools/workspace.bash?ref_type=heads) | This script simplify setting up the ROS-environment. Parameters: <br>`-r` source ros (globally and the workspace), <br>`-s` make this shell a superclient, <br>`-k` kill the discovery server and run without it, <br>`-d <id>` set the `ROS_DOMAIN_ID` for this shell, <br>`-p <user>` grants the specified user `rwx` permissions on the entire workspace. Needs sudo. (This may be usefull when mounting the workspace from an external source into docker) |
| [rpi_temp](https://gitlab.ips.biba.uni-bremen.de/ftf-navi2/ftf-v1/-/blob/main/tools/rpi_temp?ref_type=heads) | This script prints the current temperature and whether the machine is or has been throttled due to voltage constraints. |
| [record_bag](https://gitlab.ips.biba.uni-bremen.de/ftf-navi2/ftf-v1/-/blob/main/tools/record_bag?ref_type=heads) | Shortcut for only recording a set of basic necessary topics to apply the cat-stack afterwards. |

## More Information

For more information about the vehicle, see the [project documentation](https://gitlab.ips.biba.uni-bremen.de/ftf-navi2/projektbericht_v2).
