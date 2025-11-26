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

The MotionPlanning class provides functionalities for planning robot trajectories in joint space and Cartesian space. It utilizes MoveIt! libraries for underlying calculations and handles communication with the ROS environment.

## The challenge
First, correct class definition and then rebuild workspace.

Second, create an instance of the MotionPlanning class, providing a reference to your ROS node handle:
C++

```
ros::NodeHandle nh;
MotionPlanning planner(nh);
```

The class offers several methods for planning robot trajectories:
```
    planJoint(target_joint_values, planned_trajectory): Plans a collision-free joint trajectory to reach specified joint positions.
    
    planPose(target_cart_pose, planned_trajectory): Plans a joint trajectory to reach the desired Cartesian pose (specified by geometry_msgs::PoseStamped).
    
    planCartesian(cart_waypoints): Plans a trajectory following a sequence of waypoints provided as CartMotionPlanningData structs.
```

Additional functionalities
```
    exportTrajectoryToFile(filepath, joint_trajectory, cart_pose_array);

    TrajectoryData importTrajectoryFromFile(filepath);

    visualizeJointTrajectory(joint_trajectory);
    
    visualizeCartTrajectory(cart_pose_array);

    addCollision(collision_object_array);

    removeCollision(collision_object_id);
```

The challenge exists of three different tasks, whereas the third one is optional.

### Task 1
1. Create the MotionPlanning Class by completing each of the methods. If needed you can/should adapt the header file.
For each method, refer to the class documentation for specific arguments and return values.
2. Use the library you've developed to create an executable that does the following:

    1. Plan a motion to a reachable custom cartesian pose with a selected planner on MoveIt.
    2. Plan multiple time (e.g. 5 times).
    3. Compare the resultant trajectory of each planning attempt.
    4. Rank the trajectory from best to worst. The definition of best is free for intepretation based on some trajectory criteria (e.g. effort, length, etc.).
    5. Execute the best trajectory.

### Task 2

Create an exectuble/ or extend the previous one to analyze and validate the given trajectory provided in the trajectory.json file. How can the trajectoy be improved/optimized, create matrix and utility function e.g. load trajectory, plot trajectory before and after optimized trajectory? 

## Optional challenge

Create an executable which plans a cartesian path from a point A to a point B. 
Instead of using the available planners in MoveIt, solve this problem by implementing a numerical approach.
