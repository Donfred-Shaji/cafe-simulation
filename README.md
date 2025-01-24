# cafe-simulation
## Overview

This project aims to develop an autonomous **butler robot** for a **French door café**, capable of managing food delivery to three tables. The robot is designed to handle various operational scenarios such as:

- **Single and multiple order deliveries**
- **Order confirmations at the kitchen and tables**
- **Order cancellations at different stages**
- **Optimized movement and path planning for efficiency**

The project leverages **ROS 2 Humble**, and includes simulation in **RViz** and **Gazebo**, with navigation using the **`nav2`** stack.

---

## Features

- **Autonomous Navigation:**  
  - Implemented with ROS 2 `nav2` stack for path planning and obstacle avoidance.
  - LiDAR sensor used for confirmation detection.
  
- **Lifecycle Management:**  
  - Uses `ros2_lifecycle` to control task flow efficiently.

- **Simulation Environment:**  
  - Virtual café setup in Gazebo for realistic testing.  
  - Predefined paths to simulate different order scenarios.

---

## Project Structure

```plaintext
cafe/
├── src/
│   ├── two_wheeled_robot/  # Robot description and control package
│   ├── navigation/         # Nav2 configuration and launch files
│   ├── lifecycle_manager/  # Lifecycle node management
│   ├── simulation/         # Gazebo world and RViz configurations
│   └── ...                 # Other related packages
├── README.md
└── install.sh              # Installation script for dependencies
## Installation

