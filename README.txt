This is a ros package to control the AR Drone, intended to enable autonomous navigation on. This package is based primarily on the ardrone_autonomy driver and the tum_ardrone package.

Requirements: 

The system requires a AR Drone 2.0 to work on. 
The software has been tested on Ubuntu 12.04 and ROS Fuerte. 

The following packages need to be installed for AutoNav to work:

https://github.com/AutonomyLab/ardrone_autonomy

https://github.com/sniekum/ar_track_alvar

Depending on how big the workspace is, between 1-4 tags should be printed out and placed in different regions in the environment that the drone operates in. 
Logging:

The file "rosbag command" has a list of all the topics that data is logged to from the internal state estimation process running on the system. Logging is enabled by default, so it is possible to subscribe to the said list of topics and log data using rosbag. 

Installation:

Simply copy-paste the folder to a desired location, and use the rosmake command. 

The following nodes should be running before the system can be started:

ardrone_driver
ar_track_alvar (pr2_indiv_no_kinect)

The following nodes are available for running:

state_estimation: The core node that conducts the state estimation process and localizes the drone.

autopilot: Generates control commands to autonomously navigate the drone based on the user's commands.

UI: A simple interface to provide commands to the drone. 

interact: A interactive interface built using Rviz to interact with the drone. Please run only either UI or interact at any given time. 

Acknowledgements:

This package has greatly benefited from the release of :

the ardrone_autonomy package (Mani Monajjemi)
the ar_track_alvar package (Scott Niekum)
and tum_ardrone package (Jakob Engel) 
