# Md80 ROS Node

This node handles the communication between MAB's md80 drives in ROS environment. The node was designed to act as 
operational endpoint - to control the drives and get information from them, thus it is not capable of configuring the drives, 
for this use `mdtool` , `Candle` library or custom FDCAN application.

## Principles of operation

The node normally communicates via services for setup, and via topics for regular data transfers.
Services are: 
- /add_md80s
- /zero_md80s
- /set_mode_md80s
- /enable_md80s
- /disable_md80s

Topics subscribed by the node are:
- /md80/motion_command
- /md80/impedance_command
- /md80/velocity_pid_command
- /md80/position_pid_command

Topic published by the node is:
- /md80/joint_states

## Cloning the node
In order to run the node clone it into your local ROS workspace in the src folder. Then run build it with 'catkin' and run using the 'rosrun' command. Be sure to source your workspace with 
```
source devel/setup.sh
```
prior to running the package.

## Node setup
After starting the node, it will wait for configuration messages.

### Add drives
Firstly, the node should be informed which drives should be present on the FDCAN bus. This can be done via `/add_md80s` service.
For example:
```
rosservice call /add_md80s "drive_ids: [81, 97]"
```
Should produce the following output:
```
drives_success: [True, True]
total_number_of_drives: 2
```
informing, that both drives (ids: 81 and 97), have been successfully contacted, and were added to the node's drives list.

### Set mode
Next the desired control mode should be selected. This is accomplished with `/set_mode_md80s` service.
For example:
```
rosservice call /set_mode_md80s "{drive_ids: [81, 97], mode:["IMPEDANCE", "IMPEDANCE"]}"
```
Should produce:
```
drives_success: [True, True]
```
Informing that for both drives mode has been set correctly.

### Set Zero 
Often when starting, setting a current position to zero is desired. This can be accomplished with a call to `/zero_md80s` service.
```
rosservice call /zero_md80s "{drive_ids:[81, 97]}"
```

### Setting limits
The limits can be set using `/set_limits_md80s` service, for example 
```
rosservice call /set_limits_md80s "{drive_ids:[81, 97], velocity_limit:[10, 20], torque_limit:[0.1, 0.1]}"
```

### Enabling/Disabling drives
Using services `/enable_md80s` and `/disable_md80s` the drives and the node publishers and subscribers can be enabled/disabled.
**NOTE: After calling `/enable_md80s` service, no calls to services other than `/disable_md80s` should be done.**

After enabling, the node will publish current joint states to `/joint_states` at a frequency of 100Hz. Joint names are generated based on drivie ID, for example drive with id 546 will be called `Joint 546`.

The node will also listen for the messages on topics for controlling the drives. All of the above topics are listened to all the time, but currently applied settings are dependent on the md80 mode set before enabling.

### Controlling drives
Controlling the drives is done via the four topics listed above. For commands to be viable, all field of each message must be filled properly. For example, to set up custom gains for IMPEDANCE mode use:
```
rostopic pub /md80/impedance_command ros1_mab_md80/ImpedanceCommand "{drive_ids:[81, 97], kp:[0.25, 1.0], kd:[0.1, 0.05], max_output:[2.0, 2.0]}"
```

Setting desired position, velocity, and torque is done via `/md80/motion_command` topic. Note that for it to take effect, all fields in the message should be correctly filled. For example, to move the drives in impedance mode, it is possible to use the following command
```
rostopic pub /md80/motion_command ros1_mab_md80/MotionCommand "{drive_ids:[81,97], target_position:[3.0, -3.0], target_velocity:[0.0, 0.0], target_torque:[0, 0]}" -1
```
