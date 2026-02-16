Clearpath Additional Simulation Worlds
==========================================

This repository contains additional indoor and outdoor simulation environments for use with Clearpath's robot platforms and the Scout Mini robot from AgileX Robotics.

**ROS2 Humble Migration**: This repository has been migrated from ROS1 Noetic to ROS2 Humble. All launch files are now in Python format (`.launch.py`) and the build system uses `ament_cmake`.

**Scout Mini Integration**: All environments have been adapted to work with the Scout Mini robot, which is the default platform for the PIEC project.

## Building for ROS2 Humble

### Prerequisites
- ROS2 Humble installed
- Scout Mini robot packages for ROS2 (`agilex_scout`)
- Clearpath robot packages for ROS2 (e.g., `husky_gazebo`, `jackal_gazebo`, etc.) - optional for backwards compatibility
- Gazebo and `gazebo_ros` packages for ROS2

### Building with colcon
```bash
# Navigate to your ROS2 workspace
cd ~/ros2_ws/src

# Clone this repository
git clone https://github.com/muhammadamjadbit/gazebo_environment.git

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build the packages
colcon build --packages-select cpr_agriculture_gazebo cpr_race_modules cpr_obstacle_gazebo \
  cpr_accessories_gazebo cpr_empty_gazebo cpr_office_gazebo cpr_orchard_gazebo cpr_inspection_gazebo

# Source the workspace
source install/setup.bash
```

## Launching Worlds in ROS2

Launch files are now Python-based (`.launch.py` files). Use `ros2 launch` instead of `roslaunch`:

```bash
# Example: Launch agriculture world with Scout Mini (default)
ros2 launch cpr_agriculture_gazebo agriculture_world.launch.py

# Example: Launch office world with Scout Mini (default)
ros2 launch cpr_office_gazebo office_world.launch.py

# Example: Launch with custom platform (for backwards compatibility)
ros2 launch cpr_office_gazebo office_world.launch.py platform:=jackal

# Example: Launch with custom robot position
ros2 launch cpr_inspection_gazebo inspection_world.launch.py robot_x:=5.0 robot_y:=2.0 robot_z:=1.0
```


Launch Parameters
-------------------------------------------------------------

Each world has a launch file which will start the simulation environment with the appropriate robot.  These
launch files accept the following arguments:

Robot model:
- `platform`
The `platform` argument defaults to the `CPR_GAZEBO_PLATFORM` environment variable if it exists, otherwise **Scout Mini** (`scout_mini`) is chosen as the default.  Each world supports different robots.  For example Dingo and Ridgeback are not available on outdoor worlds.  Please refer to the specific world for supported robots.  Available robots for any given world will be a subset of:
- `scout_mini` (the default platform for PIEC project)
- `husky`
- `jackal`
- `warthog`
- `dingo`
- `ridgeback`
- `boxer`

Robot spawn position:
- `robot_x`
- `robot_y`
- `robot_z`
- `robot_yaw`
Most worlds default to (0, 0, 0.2, 0) as the initial (x, y, z, yaw) position, though some worlds may feature terrain or
other obstacles that necessitate a different default.  Please refer to the appropriate launch file for the default
spawn location for that world.

World spawn position:
- `world_x`
- `world_y`
- `world_z`
- `world_yaw`
All worlds default to (0, 0, 0, 0) as their spawn location.  If changing the world location, it may be necessary to
change the robot's spawn location as well, to prevent the robot from spawning in a location not above the ground plane.

Agriculture World
-------------------------------------------------------------

![Agriculture World](cpr_agriculture_gazebo/docs/agriculture_world.png "Agriculture World")

This is a flat, outdoor world with a barn, fences, and a medium-sized solar farm.

See [Agriculture World](cpr_agriculture_gazebo/docs/README.md)


Inspection World
-------------------------------------------------------------

![Inspection World](cpr_inspection_gazebo/docs/inspection_world.png "Inpsection World")

This is a hilly, outdoor world featuring a bridge, cave/mine, water, small solar farm, and water pipes.

See [Inspection World](cpr_inspection_gazebo/docs/README.md)


Obstacle World
-------------------------------------------------------------

This world features an indoor, enclosed world with non-planar ground geometry:

![Obstacle World](cpr_obstacle_gazebo/docs/obstacle-world.png "Obstacle World")

See [Obstacle World](cpr_obstacle_gazebo/docs/README.md)


Office World
-------------------------------------------------------------

This world features two maps with the same general floorplan:

![Office World](cpr_office_gazebo/docs/office_world.png "Office World")

A small office featuring hallways, meeting rooms, and furniture.

![Office World](cpr_office_gazebo/docs/construction_world.png "Construction World")

The same office, but undergoing construction.  Construction materials are piled on the floor, and several walls are
only studs.

See [Office World](cpr_office_gazebo/docs/README.md)


Orchard World
-------------------------------------------------------------

![Orchard World](cpr_orchard_gazebo/docs/whole-world.png "Orchard World")

This is a flat, outdoor world with several rows of small trees separated by dirt paths

[Orchard World](cpr_orchard_gazebo/docs/README.md)


Race Modules
-------------------------------------------------------------

This is not an actual environment, but rather a collection of concrete and dirt road segments that can be used
to build race tracks.

See [Race Modules](cpr_race_modules/docs/README.md)
