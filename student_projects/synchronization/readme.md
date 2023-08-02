# Junyi Shen with Student No. 48236327

This project simulates position control and pose synchronization of six individual agents.

Run roslaunch synchronization	gazebo.launch to load six individual agents with random initial poses and scattered position in the simulation world.
Then open another terminal and run rosservice call /controller_activate “data: true” to launch position control and pose synchronization.
