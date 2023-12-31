"""Track Config module."""
from fszhaw_msgs.msg import CurrentPosition


class TrackConfig:
    """
    Configuration of a given track.

    Holds the corresponding configurations of the given track, which will be used in the Path Planner.
    """

    class Acceleration:
        """Configuration for track 'Acceleration' (acceleration.csv)."""

        NAME = 'Acceleration'
        START_CURRENT_POSITION = CurrentPosition(
            vehicle_position_x=-49.95, vehicle_position_y=0.0, yaw=0.0, vehicle_velocity=0.0)
        POSITION_DISTANCE_THRESHOLD = 20
        CONE_DISTANCE_THRESHOLD = 6
        CONES_THRESHOLD = 3
        EDGE_DISTANCE_THRESHOLD = 5
        UNKNOWN_EDGE_DISTANCE_MINIMUM = 3
        UNKNOWN_EDGE_DISTANCE_MAXIMUM = 5
        NR_OF_CONES = 45

    class Skidpad:
        """Configuration for track 'Skidpad' (skidpad.csv)."""

        NAME = 'Skidpad'
        START_CURRENT_POSITION = CurrentPosition(
            vehicle_position_x=-0.0, vehicle_position_y=-14.4, yaw=0.0, vehicle_velocity=0.0)
        POSITION_DISTANCE_THRESHOLD = 15
        CONE_DISTANCE_THRESHOLD = 6
        CONES_THRESHOLD = 3
        EDGE_DISTANCE_THRESHOLD = 5
        UNKNOWN_EDGE_DISTANCE_MINIMUM = 3
        UNKNOWN_EDGE_DISTANCE_MAXIMUM = 4
        NR_OF_CONES = 84

    class SmallTrack:
        """Configuration for track 'SmallTrack' (small_track.csv)."""

        NAME = 'Small Track'
        START_CURRENT_POSITION = CurrentPosition(
            vehicle_position_x=-13.0, vehicle_position_y=10.3, yaw=0.0, vehicle_velocity=0.0)
        POSITION_DISTANCE_THRESHOLD = 20
        CONE_DISTANCE_THRESHOLD = 9
        CONES_THRESHOLD = 3
        EDGE_DISTANCE_THRESHOLD = 7
        UNKNOWN_EDGE_DISTANCE_MINIMUM = 3
        UNKNOWN_EDGE_DISTANCE_MAXIMUM = 5
        NR_OF_CONES = 70

    class Rand:
        """Configuration for track 'Rand' (rand.csv)."""

        NAME = 'Rand'
        START_CURRENT_POSITION = CurrentPosition(
            vehicle_position_x=53.0, vehicle_position_y=11.0, yaw=0.0, vehicle_velocity=0.0)
        POSITION_DISTANCE_THRESHOLD = 20
        CONE_DISTANCE_THRESHOLD = 15
        CONES_THRESHOLD = 3
        EDGE_DISTANCE_THRESHOLD = 12
        UNKNOWN_EDGE_DISTANCE_MINIMUM = 6
        UNKNOWN_EDGE_DISTANCE_MAXIMUM = 9
        NR_OF_CONES = 211

    class Comp2021:  # set minimum_track_width to >= 4.0 for optimization
        """Configuration for track 'Comp 2021' (comp_2021.csv)."""

        NAME = 'Competition 2021'
        START_CURRENT_POSITION = CurrentPosition(
            vehicle_position_x=20.92226, vehicle_position_y=-14.0325, yaw=0.0, vehicle_velocity=0.0)
        POSITION_DISTANCE_THRESHOLD = 25
        CONE_DISTANCE_THRESHOLD = 4
        CONES_THRESHOLD = 3
        EDGE_DISTANCE_THRESHOLD = 4
        UNKNOWN_EDGE_DISTANCE_MINIMUM = 1
        UNKNOWN_EDGE_DISTANCE_MAXIMUM = 2
        NR_OF_CONES = 314

    class GardenLight:  # set minimum_track_width to >= 4.0 for optimization
        """Configuration for track 'Garden Light' (garden_light.csv)."""

        NAME = 'Garden Light'
        START_CURRENT_POSITION = CurrentPosition(
            vehicle_position_x=0.0, vehicle_position_y=-0.0, yaw=0.0, vehicle_velocity=0.0)
        POSITION_DISTANCE_THRESHOLD = 15
        CONE_DISTANCE_THRESHOLD = 6
        CONES_THRESHOLD = 3
        EDGE_DISTANCE_THRESHOLD = 5
        UNKNOWN_EDGE_DISTANCE_MINIMUM = 1
        UNKNOWN_EDGE_DISTANCE_MAXIMUM = 2
        NR_OF_CONES = 204

    class Default: #modified
        """Default configuration for tracks."""

        NAME = 'Default'
        START_CURRENT_POSITION = CurrentPosition(
            vehicle_position_x=0.0, vehicle_position_y=0.0, yaw=0.0, vehicle_velocity=0.0)
        POSITION_DISTANCE_THRESHOLD = 1
        CONE_DISTANCE_THRESHOLD = 0.5
        CONES_THRESHOLD = 3
        EDGE_DISTANCE_THRESHOLD = 1
        UNKNOWN_EDGE_DISTANCE_MINIMUM = 0.3
        UNKNOWN_EDGE_DISTANCE_MAXIMUM = 0.75
        NR_OF_CONES = 100
