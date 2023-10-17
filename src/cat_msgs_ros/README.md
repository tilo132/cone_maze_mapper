# cat_msgs_ros

This package contains the ROS2 (rosidl) message definitions for custom message types used in the `CAT` project.

## Message Types

| Message Type | Description |
| ------------ | ----------- |
| [Cone](msg/Cone.msg) | Message type for a single cone (`pose`, `color`, `id`). |
| [Cones](msg/Cones.msg) | Message type for an array of cones. |
| [Walls](msg/Walls.msg) | Message type for an array of walls. Consists of an array of cones and an array of pairs of indices indicating the start and end cone of a wall. |
| [Poi](msg/Poi.msg) | Message type for a single point of interest. POIs are most often QR-codes in the context of CAT (`pose`, `id`). |
| [Pois](msg/Pois.msg) | Message type for an array of POIs. (not yet implemented) |
| [Polygon](msg/Polygon.msg) | A `geometry_msgs/Polygon` with an additional `id` and `color` field. |
| [Polygons](msg/Polygons.msg) | Message type for an array of polygons. |
| [PolygonsStamped](msg/PolygonsStamped.msg) | Message type for an array of polygons with a header. |

## Installation

### From Source

Clone the latest version from this repository into your workspace and compile the package using

```bash
cd workspace/src
git clone git@gitlab.ips.biba.uni-bremen.de:ftf-navi2/cat_msgs_ros.git
cd ../
colon build --packages-up-to cat_msgs_ros
```

Source the setup file of your workspace after compilation

```bash
source workspace/install/setup.bash
```

Now you're ready to go :)

## Usage

To include one of the message types in your package, add the following to your `package.xml`:

```xml
<depend>cat_msgs_ros</depend>
```

### CPP
add to your `CMakeLists.txt`:

```cmake
find_package(cat_msgs_ros REQUIRED)
```

and to the `ament_target_dependencies`:

```cmake
cat_msgs_ros
```

### Include

i.e. for a `Cone`:

#### CPP
```cpp
#include <cat_msgs_ros/msg/cone.hpp>
```

#### Python
```python
from cat_msgs_ros.msg import Cone
```


