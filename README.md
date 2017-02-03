# Trajectory Helpers

### Helper components for path planning goal generation and trajectory preprocessing for path following.

#### Goal Generator
Helper component used to set the goal pose to the planner.
Uses Gamepad as user interface. 
  * Using Start and Back buttons, the goal can be selected. 
  * Pressing the Start button the first time outputs the default goal, as specified in the config file. Only after that Start resp. Back button can be used to increment resp. decrement the goal selected.

List of all possible goals is defined inside `/trajectory_helpers::GoalGenerator.yaml`. The file contains x- and y-coordinates of possible goals. Goals are indexed in an increasing order. The initial index can be selected using the config. 

#### Trajectory Refiner
Fills in the goal heading, goal position tolerance and heading tolerance into the last Waypoint provided by the path planner. It also downsamples the trajectory, removing waypoints that are not necessary to define the polyline.  
Default tolerances are provided in the config file.


**Author: [Jan Filip](mailto:jan.filip2@gmail.com "Contact the author"),  
Contact: [Martin Azkarate](mailto:Martin.Azkarate@esa.int "Contact the maintainer"),  
Affiliation: Automation and Robotics Laboratories, ESTEC, ESA**

