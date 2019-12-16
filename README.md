# AirSimNeurIPS
My solution to Airsim Competition at NeuRIPS using Rapid Trajectory Plannind and MPC. 

See attached pdf for implementation details. 

[![results on final track 1](https://img.youtube.com/vi/38j50BZxACM/0.jpg)](https://www.youtube.com/watch?v=38j50BZxACM)

# Disclaimer
This was written for the AirSim NeuRIPS 2019 competition as a very fast prototype. It's shared here for people interested in a state of art implementation for drone racing in Python. All I did was combining a fast planner with a MPC controller, adding a python wrapper on top. 

The system is tested on Ubuntu 18.04 with python 3.6 on a Ryzen 2700x CPU. Actual performance on your platform can strongly vary. 

# Structure of code
Within `Daniel` folder, the `planning.py` is inherited and slightly adapted from [Mark Mueller](https://github.com/markwmuller/RapidQuadrocopterTrajectories). `control.py` is an interface to the code generated **ACADO** implementation of the drone controller, inherited from [Davide Falanga](https://github.com/uzh-rpg/rpg_mpc). If I'm violating any license agreement here please let me know. Adapting the `mpc_control/codegen.sh` allows you to generate highly efficient C code and python wrapper from ACADO, differently from what is available in the repo. 

# Running the repo.
You need to install AirSim first following [instructions](https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing), remember to generate settings as instructed, and get the finals environment. 

Then you should be able to run `python daniel.py --level_name 'Final_Tier_1'`. If you are lucky, you should be able to reach the goal within 55 seconds. 

