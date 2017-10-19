# flight_utils
This is a repository for various flight utilities.

## Logger
Used to log various data while the quadcopter is flying.

Notes:
* Ensure that the log_directoy path is absolute! Don't use relative identifiers like ~.
* Ensure that the log_directory ends with a forward-slash '/'.
* Ensure that the log_directory exists.

Run it with: 
```roslaunch flight_utils logger.launch```


## Visualizer
Visualizes the flight path of the quadcopter in real time. 

Notes:
px4_control must be installed in the same catkin workspace, i.e. 
/catkin_ws/ must have both flight_utils and px4_control. 

To run the visualizer, simply type:
```roslaunch flight_utils visualizer.launch```
