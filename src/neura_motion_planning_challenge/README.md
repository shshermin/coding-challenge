# Motion Planning Coding Challenge

This document provides information on how to use the MotionPlanning class for robot motion planning with MoveIt! in ROS.
Installation

To use the MotionPlanning class, ensure you have the following installed:

    1. ROS1
    2. MoveIt!
    3. Robot description and moveit config package found in the resource_debians/ directory. See README in the directory

## Installation
```
unzip neura_motion_planning_challenge.zip

# create working ROS workspace
mkdir ~/$HOME/coding_challenge/src
cd ~/$HOME/coding_challenge/src
cp -rf <downloaded unzip folder> ~/$HOME/coding_challenge/src/.
cd .. && catkin build
```

## Class Overview

 A UML class diagram has been included in the project documentation (`docs/uml_diagram.png`) for your convenience to view the overall structure of classes and their relationships. 
 I refactored existing classes for better code organization, maintanability, and acalability.

### Task 1
1. Create the MotionPlanning Class by completing each of the methods. If needed you can/should adapt the header file.
For each method, refer to the class documentation for specific arguments and return values.
2. Use the library you've developed to create an executable that does the following:

    1. Plan a motion to a reachable custom cartesian pose with a selected planner on MoveIt.
    2. Plan multiple time (e.g. 5 times).
    3. Compare the resultant trajectory of each planning attempt.
    4. Rank the trajectory from best to worst. The definition of best is free for intepretation based on some trajectory criteria (e.g. effort, length, etc.).
    5. Execute the best trajectory.

#### Task 1 - Results
**Executable Location:** `src/eura_motion_planning_challenge/Executables/`

**Video:** Video of code running can be found in `src/neura_motion_planning_challenge/docs/` directory.

**Planner Comparison & Rankings comments from terminal:**
```
[INFO] [1764235378.855514766]: Ranking (best to worst):
[INFO] [1764235378.855568481]:   1. Planner: RRT - Length: 2.071, Sum abs joints: 73.520, Time: 0.680 s
[INFO] [1764235378.855575102]:   2. Planner: LBKPIECE - Length: 2.085, Sum abs joints: 73.550, Time: 0.638 s
[INFO] [1764235378.855579742]:   3. Planner: RRTConnect - Length: 2.106, Sum abs joints: 73.695, Time: 0.626 s
[INFO] [1764235378.855584324]:   4. Planner: RRTstar - Length: 2.868, Sum abs joints: 96.707, Time: 10.613 s
[INFO] [1764235378.855589190]:   5. Planner: BKPIECE - Length: 7.719, Sum abs joints: 311.572, Time: 0.835 s
```

**Best Planner:** RRT
- **Trajectory File:** `trajectories/task_one_RRT_trajectory.yaml`

**Note on Visualization:** The visualization is slow due to WSL2 and ROS1 compatibility issues with RViz and GPU acceleration. Visualization is performed using CPU only.

### Task 2

Create an exectuble/ or extend the previous one to analyze and validate the given trajectory provided in the trajectory.json file. How can the trajectoy be improved/optimized, create matrix and utility function e.g. load trajectory, plot trajectory before and after optimized trajectory? 

#### Task 2 - Results
**Executable Location:** `src/neura_motion_planning_challenge/Executables/Task_Two` 

The output chart of plan optimization is generated at the following location: `src/neura_motion_planning_challenge/utils/trajectory_comparison/` 


## Optional challenge

Create an executable which plans a cartesian path from a point A to a point B. 
Instead of using the available planners in MoveIt, solve this problem by implementing a numerical approach.

#### Task 3 - Results
**Executable Location:** `src/neura_motion_planning_challenge/Executables/Task_Optional` 

Video available at the follwoing address:
`src/neura_motion_planning_challenge/docs/`

Note that this numerical planner does not handle singularities.

