# ros2-pursuit-evasion
This repository contains all code and analysis files for Warren, Christian, and Omar's CS 327: Introduction to Robotics final project. We implemented a pursuit-evasion simulation in ROS2 and Gazebo, where the pursuers utilize cooperative voronoi pursuit, and the evaders use an APF evasion algorithm. Please refer to instructions below on how to run.

## Dependencies
- Ubuntu 24.04
- ROS2 Jazzy
- Gazebo Harmonic

## How to Run
First, ensure the dependencies are met. Then execute:
```
git clone https://github.com/wrkzk/ros2-pursuit-evasion
cd ros2-pursuit-evasion
colcon build
source install/setup.bash
ros2 launch pursuit_evasion pursuit_evasion.launch.py pursuit_strategy:={pure|voronoi}
```

## Sources
[1] Coulter, R. Craig. 1992. Implementation of the Pure Pursuit Path Tracking Algorithm. Technical Report CMU-RI-TR-92-01. Pittsburgh, PA: Carnegie Mellon University, Robotics Institute.

[2] Zhou, Zhengyuan, Wei Zhang, Jerry Ding, Haomiao Huang, Dušan M. Stipanović, and Claire J. Tomlin. 2016. “Cooperative Pursuit with Voronoi Partitions.” Automatica 72: 64–72. https://doi.org/10.1016/j.automatica.2016.05.007.

[3] Khatib, Oussama. 1986. “Real-Time Obstacle Avoidance for Manipulators and Mobile Robots.” The International Journal of Robotics Research 5, no. 1: 90–98. https://doi.org/10.1177/027836498600500106.

[4] Macenski, S., Foote, T., Gerkey, B., Lalancette, C., & Woodall, W. (2022). Robot Operating System 2: Design, architecture, and uses in the wild. Science Robotics, 7(66), eabm6074. https://doi.org/10.1126/scirobotics.abm6074.

[5] N. Koenig and A. Howard, "Design and use paradigms for Gazebo, an open-source multi-robot simulator," 2004 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS) (IEEE Cat. No.04CH37566), Sendai, Japan, 2004, pp. 2149-2154 vol.3, doi: 10.1109/IROS.2004.1389727.
