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
```

## Installation
1. Clone the Repository:
```bash
git clone https://github.com/Donfred-Shaji/butler-robot.git ~/cafe
cd ~/cafe
```
2. Install Dependencies:
```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```
3. Build the Workspace:
```bash
colcon build --symlink-install
```
4. Source the Workspace:
```bash
source install/setup.bash
```
---
## Launch the Robot in RViz
Once the source setup is done run this laucnch code
1. Open a new terminal, and launch the robot
 ```bash
cd ~/cafe/
ros2 launch two_wheeled_robot two_wheeled_robot.launch.py
```
![Simulation](https://automaticaddison.com/wp-content/uploads/2021/09/3-mobile-robot-1.jpg)
---
## For showing Frames
```bash
ros2 run tf2_tools view_frames.py
```
![Simulation](https://automaticaddison.com/wp-content/uploads/2021/09/6-frames-pdf.jpg)
---
## Launch File – Cafe World
After build the package Run the following code:
```bash
ros2 launch two_wheeled_robot load_world_into_gazebo.launch.py
```
Here is the output. You should see the actors walking around the cafe.
![Simulation](https://automaticaddison.com/wp-content/uploads/2021/09/5-cafe-world.jpg)
---
## Now the final step!
Using ROS 2 Lifecycle and Nav2 tools for managing the butler robot's operations, such as:

- **Picking the order from the table**
- **Delivering the order from the kitchen**
### Following code will work as life_cycle ,This is the main code for the task
```bash
#! /usr/bin/env python3
import time # Time library
from copy import deepcopy # Modifying a deep copy does not impact the original
 
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2
 
from robot_navigator import BasicNavigator, NavigationResult # Helper module
 
# Positions for picking up items
pick_positions = {
  "front_desk": [-2.5, 1.4],
  "rear_desk": [-0.36, 20.0],
  "third_desk": [18.75, 15.3],
  "kitchen_room": [15.5, 5.4]}
 
# Positions for delivery of items
shipping_destinations = {
  "front_desk": [-2.5, 1.4],
  "rear_desk": [-0.36, 20.0],
  "third_desk": [18.75, 15.3],
  "kitchen_room": [15.5, 5.4]}
 
'''
Pick up an item in one location and deliver it to another. 
The assumption is that there is a person at the pick and delivery location
to load and unload the item from the robot.
'''
def main():
  # Recieved virtual request for picking item at front_reception_desk and bring to
  # employee at third_desk. This request would
  # contain the pick position ("front_reception_desk") and shipping destination ("main_conference_room")
  ####################
  request_item_location = 'front_desk'
  request_destination = 'third_desk'
  ####################
 
  # Start the ROS 2 Python Client Library
  rclpy.init()
 
  # Launch the ROS 2 Navigation Stack
  navigator = BasicNavigator()
 
  # Set the robot's initial pose if necessary
  # initial_pose = PoseStamped()
  # initial_pose.header.frame_id = 'map'
  # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
  # initial_pose.pose.position.x = 0.0
  # initial_pose.pose.position.y = 1.0
  # initial_pose.pose.position.z = 0.0
  # initial_pose.pose.orientation.x = 0.0
  # initial_pose.pose.orientation.y = 0.0
  # initial_pose.pose.orientation.z = 0.0
  # initial_pose.pose.orientation.w = 1.0
  # navigator.setInitialPose(initial_pose)
 
  # Wait for navigation to fully activate
  navigator.waitUntilNav2Active()
 
  # Set the pick location  
  pick_item_pose = PoseStamped()
  pick_item_pose.header.frame_id = 'map'
  pick_item_pose.header.stamp = navigator.get_clock().now().to_msg()
  pick_item_pose.pose.position.x = pick_positions[request_item_location][0]
  pick_item_pose.pose.position.y = pick_positions[request_item_location][1]
  pick_item_pose.pose.position.z = 0.0
  pick_item_pose.pose.orientation.x = 0.0
  pick_item_pose.pose.orientation.y = 0.0
  pick_item_pose.pose.orientation.z = 0.0
  pick_item_pose.pose.orientation.w = 1.0
  print('Received request for item picking at ' + request_item_location + '.')
  navigator.goToPose(pick_item_pose)
 
  # Do something during our route
  # (e.g. queue up future tasks or detect person for fine-tuned positioning)
  # Simply print information for employees on the robot's distance remaining
  i = 0
  while not navigator.isNavComplete():
    i = i + 1
    feedback = navigator.getFeedback()
    if feedback and i % 5 == 0:
      print('Distance remaining: ' + '{:.2f}'.format(
            feedback.distance_remaining) + ' meters.')
 
  result = navigator.getResult()
  if result == NavigationResult.SUCCEEDED:
    print('Got product from ' + request_item_location +
          '! Bringing product to shipping destination (' + request_destination + ')...')
    shipping_destination = PoseStamped()
    shipping_destination.header.frame_id = 'map'
    shipping_destination.header.stamp = navigator.get_clock().now().to_msg()
    shipping_destination.pose.position.x = shipping_destinations[request_destination][0]
    shipping_destination.pose.position.y = shipping_destinations[request_destination][1]
    shipping_destination.pose.position.z = 0.0
    shipping_destination.pose.orientation.x = 0.0
    shipping_destination.pose.orientation.y = 0.0
    shipping_destination.pose.orientation.z = 0.0
    shipping_destination.pose.orientation.w = 1.0
    navigator.goToPose(shipping_destination)
 
  elif result == NavigationResult.CANCELED:
    print('Task at ' + request_item_location + ' was canceled. Returning to staging point...')
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    navigator.goToPose(initial_pose)
 
  elif result == NavigationResult.FAILED:
    print('Task at ' + request_item_location + ' failed!')
    exit(-1)
 
  while not navigator.isNavComplete():
    pass
 
  exit(0)
 
if __name__ == '__main__':
  main()
```
---
## launch the robot in a Gazebo world
After build and set-up run the following code:
```bash
ros2 launch two_wheeled_robot office_world_v1.launch.py
```
![Simulation](https://automaticaddison.com/wp-content/uploads/2021/10/1-office-delivery.jpg)
---
## For pick and delivary run the code
```bash
ros2 run two_wheeled_robot pick_and_deliver.py
```
![Simulation](https://automaticaddison.com/wp-content/uploads/2021/10/3-on-the-move.jpg)

---
