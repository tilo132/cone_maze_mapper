# nav2_layers (for this CMakeLists.txt)
apt install -y ros-humble-nav2-costmap-2d ros-humble-geometry-msgs ros-humble-pluginlib
# nav2 planner (for planner_launch.py)
apt install -y ros-humble-nav2-theta-star-planner ros-humble-nav2-planner ros-humble-nav2-common ros-humble-nav2-smoother ros-humble-nav2-lifecycle-manager

# externals (for bringup_launch.py => localization_launch.py, mapper_launch.py)
apt install -y ros-humble-robot-localization ros-humble-slam-toolbox