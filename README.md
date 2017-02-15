# matsim-AMoD
A fork of MATSIM's taxi module to rebalancing of self-driving vehicles in an Autonomous Mobility on Demand (AMoD) framework. Implements the rebalancing algorithms presented in [1] and [2]. The framework was used for simulations in [3].

The User Guide in the root folder provides a quick-start guide.

The rebalancing algorithms are implemented in MATLAB. The current LP solver is IBM ILOG CPLEX: a different LP solver (e.g. Gurobi or MATLAB's `linprog`) can be used by modifying the file `run_optimization.m` in `MATLAB_utils`.

[1] Zhang R, Pavone M (2016) *[Control of robotic Mobility-on-Demand systems: A queueing-theoretical perspective](http://web.stanford.edu/~pavone/IJRR_Submission/Zhang.Pavone.IJRR15.pdf)*. Int. Journal of Robotics Research 35(1-3):186--203

[2] Zhang R*, Rossi F*, Pavone M (2016) *[Routing Autonomous Vehicles in congested transportation networks: structural properties and coordination algorithms](http://www.roboticsproceedings.org/rss12/p32.html)*. In Robotics: Science and Systems. Ann Arbor, MI, July, 2016. 

[3] [2] Rossi F, Zhang R, Hindy Y, Pavone M (2017) *Routing Autonomous Vehicles in congested transportation networks: structural properties and coordination algorithms*. Submitted to Autonomous Robots.
