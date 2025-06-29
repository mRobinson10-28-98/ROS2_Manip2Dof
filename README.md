---
Overview:
---
Hello and welcome to my ROS2 2 degree of freedom planar manipulator practice package. This package (and supporting packages) work together to render a planar 2-Dof manipulator tracking a circle. The purpose of this project is simply to demonstrate the basics of ROS2 and robotics systems concepts, and not to provide any functional use for robotics modelling or controls. The robotics concepts I am attempting to demo here include:

  1. ROS2 nodes
  2. ROS2 communcation protocols such as publishers/subscribers and service/clients
  3. Basic robotics systems architecture in which:
     a. one node publishes manipulation position goals to a topic(a circle)
     b. a second node acting as a solutions server subscribed to this goals topic runs an IK calculation to solve for joint space solutions and publishes these solutions to a second topic
     c. a third node subscribed to both the goal and solution topics and renders a 2-DoF serial chain manipulator in python using the pygame library tracing a circle

---
Installation:
---
To use, first make sure this repo is cloned in a standard ROS2 workspace in the <ros2_toplevel>/src directory. For more info on building a ROS2 workspace, see this guide: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html

Next, install the other package dependencies from my github using the vcs UNIX tool. In a terminal in the <ros2_toplevel>/src directory, run the following:

'vcs import . --input <my.repos path>'

Make sure to replace <my.repos path> with the path to the my.repos file located at the top level of this repo; if running this command from the <ros2_toplevel>/src directory, the path would be 'ROS2_Manip2Dof/my.repos'

---
To Run: 
---
To build these packages, run standard ros2 'colcon build' from top level directory of workspace.
If dependency issues are found, run 'rosdep install -i --from-path src --rosdistro humble -y'.

To run the example, first source the local workspace with 'source ./install/setup.bash', then launch the system using the launch file with the command 'ros2 launch manip2dof_practice manip2dof_practice_launch.xml'.
