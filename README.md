# ros2-pursuit-evasion

## Dependencies
- Ubuntu 24.04
- ROS2 Jazzy
- Gazebo Harmonic

## How to Run
First, ensure the dependencies are met. Then execute:
`
colcon build
source install/setup.bash
ros2 launch pursuit_evasion pursuit_evasion.launch.py pursuit_strategy:={pure|voronoi}
`
