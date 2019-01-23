# Pause Package

This package is a template of how you could start and stop Moveit! mid-action.Moveit! does not have any pause functions. Therefore to pause an action, Moveit! needs to stop, replan, and start again.

## Dependencies

Install moveit for the PR2 with the commands:

`sudo apt-get install ros-indigo-moveit-full-pr2`

`source /opt/ros/indigo/setup.bash`

Tutorial information can be found [here](http://docs.ros.org/indigo/api/moveit_tutorials/html/index.html "Moveit! Tutorials")

### Running

In two terminals:

Terminal 1:

`roslaunch pause_pkg pause.launch`

Terminal 2:

Make service calls to move the right arm. 

NOTE: sometimes rviz will pop up but it's clear so you only see your desktop. I'm not sure why this happens but I've found just stopping and relaunching works best.

#### Service Calls

For all service calls tab-complete to get the overall structure of the request.

* `/plan`
* `/move`
* `/stop`

These are some good manipulator positions

Pose | #1 | #2 |
--- | --- | --- | 
x | 0.1 | 0.5 |
y | -0.7 | -0.2 |
z | 0.7 | 0.7 |
w | 1.0 | 1.0 |

### Potential Problems:
- There may be some dependencies missing
- If you have errors loading RVIZ it might help to add the Kinect Camera by editing the pr2.urdf.xacro in the pr2 description

## TODO:

* Implement pick&place and stop on the architecture in Gazebo. I'd look in the `table_setting_demo` package. Dave said there should be a place to interface with the architecture and Gazebo.
