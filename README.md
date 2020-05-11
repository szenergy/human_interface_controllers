# Human-interface software
This repository contains software related to the human-control (e.g. through wheel or keyboard) of simulated/real vehicles and robots.

# Installation
Obtain __szelectricity_common__ from the following repository: https://github.com/szenergy/szenergy-common
Or use VCS, to import prerequisite pacakge:
```bash
vcs -w1 import < ../req.repos
```

To use this software, install the following prerequisites:
- libncurses (sudo apt install libncurses-dev)

# Getting started

## Keyboard controller
To use the keyboard controller, compile the workspace. Then you can start the node, with the following command:
```bash
rosrun keyboard_controller keyboard_controller_node
```

If you see the following, you are all setup. 
```
Car-like keyboard control
-------------------------
    /\   :W
A:<    > :D
    \/   :S
```
Expect control commands on __ctrl_cmd__ topic (type: autoware_msgs/ControlCommandStamped).
