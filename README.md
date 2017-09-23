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
