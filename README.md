# knowrob_poller
A node which repeatedly polls knowrob for a comment about a robot's last and pushes this out over ROS.

## Installation
The installation requires the ```knowrob``` and ```knowrob_addons``` stacks. Compiling them from source can take a bit time. Here is how to do it using ```catkin_make```, assuming you have a workspace set up at ```~/catkin_ws/src```:
```shell
rosdep update
cd ~/catkin_ws/src
wstool merge https://raw.githubusercontent.com/code-iai/knowrob_poller/master/rosinstall/rosinstall
wstool update
rosdep install --ignore-src --from-paths .
cd ~/catkin_ws
catkin_make
```

## Running
You first need to start a Knowrob with the package ```knowrob_robohow``` loaded. For testing you can do this:
```shell
roslaunch knowrob_robohow knowrob_robohow.launch
```

Then, you can start the polling node:
```shell
roslaunch knowrob_poller knowrob_poller.launch
```

For testing with a server without any facts, use the latter launch-file with the parameter ```dummmy_mode:=true```
