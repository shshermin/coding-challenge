## Install these robot description and moveit config ROS packages with

```
cd <current directory>
sudo dpkg -i *.deb
```

## Test your installation with following options

1. Test robot description installation, an RViz window with the MIRA robot should be visible.
```
roslaunch mira_description display.launch
```

2. Test moveit config installation, an RViz window with the robot and MoveIt plugin should be visible to plan and execute
```
roslaunch mira_picker_moveit_config demo.launch
```  