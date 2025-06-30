# Stretch Robot Demo: Transporting Cups

## 1. Introduction

**Purpose:**  
The objective of this demonstration is to showcase the Stretch robot’s ability to autonomously transport objects. Specifically, the robot is tasked with moving cups between a shelf and a table, and then returning them to their original location on the shelf.

## 2. Preparation

### Setup Procedure

#### On the PC

**Terminal 1**

1. Open a terminal window.
2. Connect to the robot via SSH and launch Byobu:
    ```bash
    ssh hello-robot@192.168.102.27
    byobu
    ```
3. Within Byobu, open separate windows and execute the following commands (F2 to open new window, F3/F4 to navigate between windows):
    ```bash
    roscore
    roslaunch iai_stretch_bringup stretch_bringup.launch map_yaml:=${HELLO_FLEET_PATH}/maps/apartment-full.yaml
    rosservice call /home_the_robot
    ```
4. Wait for the robot’s confirmation beep.

**Terminal 2**

1. Open a new terminal window.
2. Split the terminal into five panes.
3. In each pane, run the following commands:
    ```bash
    rviz -d `rospack find stretch_navigation`/rviz/navigation.rviz
    roslaunch giskardpy giskardpy_stretch_iai.launch
    pycharm-professional
    rosrun robokudo main.py _ae=stretch_demo_query
    rosrun rqt_reconfigure rqt_reconfigure
    ```
4. In the **rqt_reconfigure** window, navigate to **robokudo → ImageClusterExtractor** and set **contour_min_size** to **167**.

#### Rviz

1. Use the **2D Pose Estimate** tool to set the robot’s initial location.
2. Assign a **2D Nav Goal** (or additional goals as needed) to localize the robot within the environment.

## 3. Execution

**Process Overview:**  
The demonstration is divided into two phases:
1. The Stretch robot navigates to the shelf, detects the cups, and transfers them to the table.
2. The robot then retrieves the cups from the table and places them back onto the shelf.

**Tools and Software:**  
- Giskard
- PyCRAM

## 4. Challenges

**Navigation and Localization**
- The robot sometimes loses its localization during navigation, resulting in delays and inconsistencies in its pose.
- At times, navigation is reported as successful without any movement.
  
**Pick-and-Place Operations**
- Despite accurate pose estimation from perception, the robot sometimes attempts to pick objects from incorrect positions
- When attempting to grasp objects near the robot, the robot may rotate around its gripper, causing collisions with the shelf.
- While picking up cups from the table, the robot sometimes uses incorrect vertical (z-axis) positions, attempting to pick at an unexpected height.

**Arm Joints and Motion**
- When adjusting the robot’s head to look at a specific position, the arm occasionally extends unintentionally, which is undesirable during movement.

