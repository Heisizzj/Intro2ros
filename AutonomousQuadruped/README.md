## Getting Started


1. Clone the repository and build it
2. Download the Unity Environment: https://syncandshare.lrz.de/getlink/fiLvgiTXetubiN1i4PRjuR/
3. Unzip the Unity file and copy the files to .../devel/lib/simulation/
4. Run:
  a.) roslaunch simulation run.launch
5. There's a keyboard control mode for testing control params:
  a.) roslaunch simulation simulation.launch
  b.) roslaunch perception perception.launch
  c.) roslaunch planning planning.launch
  d.) rosrun controller_key_control key_control
  e.) roslaunch key_board keyboard.launch
 
## Dependencies

Install octomap, octomap-rviz-plugin, depth-image-proc, move-base:
`sudo apt-get install ros-noetic-octomap-rviz-plugins`
`sudo apt-get install ros-noetic-octomap-server`
`sudo apt-get install ros-noetic-depth-image-proc`
`sudo apt-get install ros-noetic-move-base`

