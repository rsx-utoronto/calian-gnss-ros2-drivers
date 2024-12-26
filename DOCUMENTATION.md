# ROS2 -> ROS1 Notes
(Read these notes while looking at the changes I have made in the files in the commit otherwise they wouldn't make any sense)

- The unique_id_finder file wasn't able to detect the GPS module:
    - For Ubuntu 22, the key used is "Standard". Changed that to "Dual USB" for Ubuntu 20
    - This fixed the problem though an error still occurs (which doesn't affect the code)
    - This error is because the "Dual USB" exists in both USB ports that are opened when GPS module is connected, where one of them gives device id and the other does not.
    - Had to do the same in serial_module.py file

- gps.py wasn't able to take the parameters from the launch file and config file
    - Had to fix how gps.py takes in the parameters
    - Removed the namespace (because everything is in calian_gnss)
    - Changed the structure of yaml files
    - Hardcoded the parameter key for every mode (check __init__ function in gps.py)
    - **Next Step**: Do the same for ntrip, pointperfect stuff and other .py files (like the visualizer)

- Another error:
    ![](doc_resources/events_not_callable_error.png)