# matsim-AMoD
A fork of MATSIM's DVRP module [0] to rebalancing of self-driving vehicles in an Autonomous Mobility on Demand (AMoD) framework. Implements the rebalancing algorithms presented in [1] and [2]. The framework was used for simulations in [3].
## Requirements
JRE (for MATSim), MATLAB (for the rebalancing algorithms in [1] and [2]), a MATLAB LP solver. The current LP solver is IBM ILOG CPLEX: a different LP solver (e.g. Gurobi or MATLAB's `linprog`) can be used by modifying the file `run_optimization.m` in `MATLAB_utils`.

## Quick start
Prepare a configuration file to specify the problem parameters. Sample configuration files are provided in `src/main/resources/configs` - the relevant sample problem inputs are also provided in this repository, but you may have to specify absolute paths to them.

Specify the location of the configuration file in `src/main/java/RunAMoDExample.java` .

Configure the location of the road map in the MATLAB optimizer by editing the file `MATLAB_utils/runOptimization.m` (line ~52). A sample road map of New York City is provided in `LoadRoadGraphNYOSM.m` . Note that the capacity of the roads in `LoadRoadGraphNYOSM` do not necessarily match the capacity of the roads in the MATSim configuration file (which are reduced to account for exogenous traffic). To ensure that MATLAB and MATSim employ the same road capacities, edit line ~89 in `LoadRoadGraphNYOSM.m`.

Ensure that 1. the road maps used by MATLAB and by MATSim are identical (see the User Guide for details on how to generate a MATSim road map from MATLAB data) and 2. the path of the optimization output in MATLAB is correctly specified in the configuration file.

Run `src/main/java/RunAMoDExample.java` to start the simulation.

The User Guide in the root folder provides a more detailed introduction to the matsim-AMoD package.

[0] M. Maciejewski, J. Bischoff, S. Hörl, K. Nagel (2017) *[Towards a testbed for dynamic vehicle routing algorithms](https://svn.vsp.tu-berlin.de/repos/public-svn/publications/vspwp/2017/17-06/)*; submitted to PAAMS-TAAPT.

[1] Zhang R, Pavone M (2016) *[Control of robotic Mobility-on-Demand systems: A queueing-theoretical perspective](http://web.stanford.edu/~pavone/IJRR_Submission/Zhang.Pavone.IJRR15.pdf)*. Int. Journal of Robotics Research 35(1-3):186--203

[2] Zhang R*, Rossi F*, Pavone M (2016) *[Routing Autonomous Vehicles in congested transportation networks: structural properties and coordination algorithms](http://www.roboticsproceedings.org/rss12/p32.html)*. In Robotics: Science and Systems. Ann Arbor, MI, July, 2016.

[3] Rossi F, Zhang R, Hindy Y, Pavone M (2017) *[Routing Autonomous Vehicles in congested transportation networks: structural properties and coordination algorithms](https://web.stanford.edu/~frossi2/pdf/Rossi.Zhang.Hindy.Pavone.AURO17.pdf)*. Submitted to Autonomous Robots.
