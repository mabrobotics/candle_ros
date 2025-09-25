![status: unmaintained](https://img.shields.io/badge/status-unmaintained-red)

> ⚠️ **Notice**  
> This project is no longer maintained since [ROS 1](https://wiki.ros.org/Documentation) reached end of life (May 31, 2025).  
> Please use the ROS 2 version instead: [candle_ros2](https://github.com/mabrobotics/candle_ros2/tree/main).

# MD80 ROS Node

This node handles the communication between MAB's MD80 drives in ROS environment. The node was designed to act as 
operational endpoint - to control the drives and get information from them, thus it is not capable of configuring the drives, 
for this use [MDtool](https://github.com/mabrobotics/mdtool).

## Available services and topics

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

## Quick startup guide

Please find a detailed startup guide in the [MD80 x CANdle manual](https://www.mabrobotics.pl/servos)