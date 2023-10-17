# Mapping-Node

## Overview

This package contains two nodes "locate_cones" and "wall_detector":

- locate_cones: 
This node is responsible for putting the localy detected cones and pois from the 
/cones/pointcloud and /pois topic into the global map. \
It publishes /cones/map and /poi/map.

- wall_detector: 
This node is reponsible for detecting walls between cones (/cones/map) on the global map. \
It publishes its results on the /walls topic.


## Parameters

### Locate_cones

| Parameter | Type | Description |
|-----------|------|-------------|
| max_rematch_distance | double | The distance below which cones are considered for merging |
| cleanup_time_stamp_threshold | int | The time difference between the current time and the time of the last cone detection after which the cone is considered in the cleanup step |
| cleanup_comparison_radius | double | Radius in meters in which clusters are compared to each other in the cleanup step |
| cleanup_confidence_scale | double | How much lower the confidence of a cluster has to be compared to the average confidence of clusters in the comparison radius to be cleaned up |


### Wall_detector

| Parameter | Type | Description |
|-----------|------|-------------|
| max_offset_angle | double | The maximum angle offset for a middle cone to be considered to be on a line with two outer cones |
| max_middle_offset | double | The maximum distance between the middle cone of a wall can be off from the true middle point of the outer cones |
| max_wall_length | double | The maximum length of a wall in meters |
| mode | string | Either "Bremergy" or "Logistic" changes the expected color pattern for walls |

#### Bremergy mode:
| Parameter | Type | Description |
|-----------|------|-------------|
| wall_colors | string (json arrays of strings) | List of expected wall colors can include "BLUE,GREEN,PINK,RED,YELLOW" |

#### Logistic mode:
| Parameter | Type | Description |
|-----------|------|-------------|
| wall_pairs | string (json array of array (len=2) of strings) | List of expected wall color pairs can include colors from "BLUE,GREEN,PINK,RED,YELLOW" |
| num_neighbors | int | Number of neighbors to consider when calculating the walls for logistic mode |


## Installation

### Building from Source

To build from source, clone the latest version from this repository into your workspace and compile the package using colcon build:
```
cd workspace/src
git clone git@gitlab.ips.biba.uni-bremen.de:ftf-navi2/mapping_node.git
cd ../
colcon build --packages-up-to mapping
```