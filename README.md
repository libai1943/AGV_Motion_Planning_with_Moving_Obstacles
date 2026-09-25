# AGV Motion Planning with Moving Obstacles

MATLAB implementation of a **first-search-then-optimization trajectory planning framework** for an automated guided vehicle (AGV) navigating among moving obstacles.

This repository accompanies the paper:

> **B. Li, Y. Zhang, Y. Ouyang, Y. Liu, X. Zhong, H. Cen, and Q. Kong**,  
> “Fast Trajectory Planning for AGV in the Presence of Moving Obstacles: A Combination of 3-dim A* Search and QCQP,”  
> in *2021 33rd Chinese Control and Decision Conference (CCDC)*, pp. 7549–7554, 2021.  
> DOI: **10.1109/CCDC52312.2021.9602686**

If you use this repository, the implementation, or the associated planning framework in academic work, **please cite the paper above**.

---

## Overview

Trajectory planning in environments with moving obstacles is fundamentally a **spatiotemporal planning problem**.

A geometrically collision-free path is not sufficient: the AGV and a moving obstacle may occupy the same position at different times without conflict, or may collide even though their spatial paths appear separately reasonable.

This repository addresses the problem through a simple but effective two-stage framework:

```text
Known trajectories of moving obstacles
                |
                v
       3-D spatiotemporal A*
          (x, y, time)
                |
                v
        Coarse trajectory
                |
                v
        QCQP refinement
       via AMPL + Ipopt
                |
                v
   Smooth collision-free trajectory
```

The first stage searches a discretized **space-time grid** to quickly obtain a collision-free trajectory. The second stage treats that trajectory as an initial guess and refines it through continuous optimization.

The philosophy is straightforward:

> **Search first for robustness; optimize afterwards for trajectory quality.**

---

## Main Idea

### Stage 1: 3-D A* Search

Conventional planar A* uses nodes of the form

```text
(x, y)
```

For moving-obstacle planning, this repository augments the configuration with time:

```text
(x, y, t)
```

A node is therefore feasible only if the AGV can occupy the corresponding spatial position **at that particular time** without colliding with any moving obstacle.

Conceptually:

```text
             time
               ^
               |
          o----o----o
         /    /    /
        o----o----o
       /    /    /
      o----o----o  ---> y
     /
    x
```

The A* search operates in this discretized three-dimensional space and generates a coarse spatiotemporal trajectory from the initial position to the target.

---

### Stage 2: Continuous Trajectory Refinement

The grid-based A* solution is robust but naturally coarse.

The second stage therefore increases the temporal resolution and uses the search result as an initial guess for a continuous optimization problem.

The refined trajectory is described by:

```text
x(t), y(t)
```

together with the corresponding first- and second-order derivatives:

```text
dx(t), dy(t)
ddx(t), ddy(t)
```

The optimization penalizes motion intensity and acceleration while retaining a small regularization term toward the coarse A* trajectory.

Moving-obstacle avoidance is directly imposed at the optimization nodes.

Although the optimization is executed through AMPL and Ipopt as a nonlinear program, the mathematical structure implemented in `C1.mod` is a **quadratically constrained quadratic program (QCQP)**, including nonconvex quadratic collision-avoidance constraints.

---

## Why Combine Search and Optimization?

Directly solving the continuous optimization problem from an arbitrary initial guess can be numerically difficult because moving-obstacle collision constraints are nonconvex.

On the other hand, using only grid search introduces discretization effects and generally produces trajectories that are not sufficiently smooth.

The two stages complement each other:

| Stage | Main Role |
|---|---|
| 3-D A* | Quickly find a collision-free spatiotemporal route |
| QCQP | Smooth and refine the trajectory in continuous space |

The A* result provides the optimization stage with an initial solution that already captures the appropriate **spatiotemporal homotopy** around moving obstacles.

---

## Moving-Obstacle Model

The demonstration generates circular moving obstacles.

Each obstacle is represented by:

```text
[x_start, y_start, x_end, y_end, radius]
```

and moves linearly from its initial position to its final position during the planning horizon.

For obstacle \(j\), the implementation interpolates its position according to planning time:

```text
initial position
       |
       |  linear motion
       v
final position
```

Thus, obstacle occupancy is time dependent.

The AGV is also represented as a disk, which makes collision checking between the AGV and moving obstacles particularly efficient.

---

## Repository Workflow

The main entry is:

```matlab
RunMe
```

Its execution flow is:

```text
Initialize planning parameters
            |
            v
Randomly generate start and goal
            |
            v
Generate moving obstacles
            |
            v
SearchTrajViaAstar()
            |
            v
3-D A* coarse trajectory
            |
            v
RefinePathViaNLP()
            |
            v
AMPL / Ipopt QCQP refinement
            |
            v
DemonstrateDynamicResult()
```

For first-time users, `RunMe.m` is the best place to start.

---

# Quick Start

Clone the repository:

```bash
git clone https://github.com/libai1943/AGV_Motion_Planning_with_Moving_Obstacles.git
cd AGV_Motion_Planning_with_Moving_Obstacles
```

Open MATLAB, set this repository as the current working directory, and run:

```matlab
RunMe
```

A new scenario is randomly generated for each execution.

The default example uses:

```matlab
params_.Nobs = 5;
```

moving obstacles.

The initial and target positions are also randomly sampled inside the planning region.

---

## Main Parameters

The most important parameters can be found directly in `RunMe.m`.

### Planning Region

```matlab
params_.x_min = -20;
params_.x_max = 20;
params_.y_min = -20;
params_.y_max = 20;
```

The planning environment is therefore a square workspace.

---

### AGV Radius

```matlab
params_.radius = 2;
```

The AGV is approximated by a circle with this radius.

---

### Planning Horizon

```matlab
params_.tf_max = 40;
```

The default planning horizon is 40 time units.

---

### 3-D Search Grid

```matlab
params_.NT = 80;
params_.NX = 20;
params_.NY = 20;
```

The A* search discretizes:

```text
x direction : NX cells
y direction : NY cells
time        : NT layers
```

The corresponding increments are calculated as:

```matlab
params_.dt
params_.dx
params_.dy
```

Thus, each search node corresponds to one element of:

```text
NX × NY × NT
```

spatiotemporal space.

---

### Optimization Resolution

```matlab
params_.Nfe = params_.NT * 3;
```

The continuous optimization uses three times as many temporal nodes as the coarse A* trajectory.

With the default parameters:

```text
NT  = 80
Nfe = 240
```

This allows the second stage to refine the relatively coarse search result at a higher resolution.

---

## `GenerateDynamicObstacles.m`

This function randomly creates moving circular obstacles.

Each obstacle contains:

```matlab
[ox0, oy0, oxf, oyf, radius]
```

where:

- `ox0`, `oy0` are its initial coordinates;
- `oxf`, `oyf` are its terminal coordinates; and
- `radius` specifies its size.

The obstacle radius is randomly generated, while its initial and final positions are sampled inside the workspace.

The function also avoids placing an obstacle directly over the AGV initial or target position.

---

# `SearchTrajViaAstar.m`

This file implements the first planning stage.

It is the main search-based component of the repository.

## Spatiotemporal State

The initial configuration:

```matlab
params_.x0
params_.y0
```

is converted into the grid state:

```text
(x index, y index, time index)
```

Similarly, the target is associated with the final time layer.

The complete graph therefore explicitly contains time.

---

## Node Expansion

For each time step, the AGV can transition between nearby spatial grid cells.

With the default:

```matlab
params_.Nring = 1;
```

the planner considers spatial changes over the local neighborhood while always advancing one step in time.

This includes the possibility of remaining approximately at the same spatial location while time advances, which can be useful when a moving obstacle must pass before the AGV proceeds.

Conceptually:

```text
time k                         time k+1

 o  o  o                        o  o  o
 o  X  o        ------>         o  ?  o
 o  o  o                        o  o  o
```

---

## Collision Checking

For every candidate transition, intermediate points are sampled.

The corresponding obstacle position is calculated at each sampled time instant.

A node transition is rejected if:

```text
distance(AGV, obstacle)
```

falls below the required combined safety radius.

Therefore, the search considers not only **where** the obstacle is, but also **when** the AGV reaches the corresponding position.

---

## Search Cost

The A* implementation uses:

```text
f = g + h
```

where `g` represents accumulated transition cost and `h` guides the search toward the spatial target and final time layer.

The implementation also includes a penalty associated with changes in consecutive expansion operations, which discourages unnecessarily irregular coarse motions.

The final result is a coarse sequence:

```matlab
[x, y]
```

that is subsequently passed to the optimization stage.

---

# `RefinePathViaNLP.m`

This function connects the MATLAB search stage to the AMPL/Ipopt optimization stage.

Its main operations are:

```text
Write moving-obstacle trajectories
            |
            v
Write problem parameters
            |
            v
Write coarse A* trajectory
            |
            v
Generate optimization initial guess
            |
            v
Call AMPL + Ipopt
            |
            v
Read optimized x and y
```

The four internal preparation functions are:

```matlab
WriteObsFile()
WriteNLPInfo()
WriteCoarsePath()
WriteIG()
```

---

## Higher-Resolution Initial Guess

The A* path contains `NT` samples.

Before optimization, each coarse interval is interpolated to generate a denser trajectory containing:

```matlab
Nfe = 3 * NT
```

samples.

The initial velocity components are estimated using finite differences:

```text
dx
dy
```

and the accelerations are similarly estimated as:

```text
ddx
ddy
```

These quantities form the warm start supplied to the nonlinear solver.

---

# `C1.mod`

`C1.mod` defines the continuous trajectory-refinement problem.

The optimization variables are:

```text
x
y
dx
dy
ddx
ddy
```

at every discretization point.

The discrete dynamics are:

```text
position <- velocity
velocity <- acceleration
```

in both Cartesian directions.

---

## Objective Function

The implemented objective contains three main ideas.

First, velocity magnitude is penalized:

```text
dx² + dy²
```

Second, acceleration receives a significantly larger penalty:

```text
100 × (ddx² + ddy²)
```

which promotes a smoother trajectory.

Finally, a small regularization term keeps selected optimization nodes reasonably related to the coarse A* trajectory.

Thus, the A* path guides the local optimization without forcing the optimized trajectory to coincide exactly with the discrete search result.

---

## Collision-Avoidance Constraints

At every optimization node and for every moving obstacle:

```text
distance² >= safe radius²
```

is imposed.

More specifically, the implementation uses the combined radius of:

```text
AGV radius + obstacle radius
```

with an additional safety factor.

Because obstacle coordinates vary with time, these constraints represent the predicted obstacle occupancy throughout the complete planning horizon.

This is the key connection between the moving-obstacle prediction and the QCQP refinement.

---

# `r0.run`

This AMPL script:

1. loads `C1.mod`;
2. loads the initial guess;
3. selects Ipopt;
4. solves the optimization problem;
5. exports optimized `x` and `y`;
6. writes a solver-success flag.

The MATLAB layer subsequently loads:

```text
x.txt
y.txt
flag.txt
```

to recover the result.

---

# `DemonstrateDynamicResult.p`

This protected MATLAB function visualizes the resulting AGV trajectory together with the moving obstacles.

Because it is distributed as a `.p` file, it can be executed normally by MATLAB but its internal source code is not exposed.

Its purpose is visualization rather than the implementation of the core search or optimization algorithm.

---

## File Structure

```text
AGV_Motion_Planning_with_Moving_Obstacles/
│
├── RunMe.m
│   Main entry of the complete demonstration
│
├── GenerateDynamicObstacles.m
│   Generates moving circular obstacles
│
├── SearchTrajViaAstar.m
│   3-D spatiotemporal A* search
│
├── RefinePathViaNLP.m
│   Converts the A* solution into an optimization warm start
│   and calls AMPL/Ipopt
│
├── C1.mod
│   QCQP trajectory-refinement model
│
├── r0.run
│   AMPL execution script
│
├── ipopt.opt
│   Ipopt solver configuration
│
├── DemonstrateDynamicResult.p
│   Dynamic trajectory visualization
│
├── ipopt.exe
├── libhsl.dll
├── libipoptfort.dll
├── ampltabl.dll
│   Solver-related files from the original implementation
│
└── LICENSE
```

---

# Requirements

The demonstration was developed using:

- MATLAB
- AMPL
- Ipopt

The optimization stage is invoked directly from MATLAB using:

```matlab
!ampl r0.run
```

Therefore, the `ampl` executable must either:

- be available in the current working directory; or
- be accessible through the system `PATH`.

A **valid AMPL installation/license** should be obtained separately by the user.

The repository contains several solver-related Windows binaries from the original implementation, and the `.run` file uses the Windows command:

```text
del
```

so the archived demo is most naturally used in a **Windows + MATLAB + AMPL/Ipopt** environment.

---

## Ipopt Settings

The supplied `ipopt.opt` includes:

```text
max_iter      1000
max_cpu_time  100
tol           1e-6
mu_strategy   adaptive
print_level   0
```

The linear-solver option for MA57 is present but commented out in the current configuration.

---

# Trying Different Scenarios

The code is intentionally compact, so several experiments can be performed simply by modifying `RunMe.m`.

### Change the number of moving obstacles

```matlab
params_.Nobs = 5;
```

### Change the AGV radius

```matlab
params_.radius = 2;
```

### Change the planning horizon

```matlab
params_.tf_max = 40;
```

### Increase A* spatial resolution

```matlab
params_.NX = 20;
params_.NY = 20;
```

Larger values produce a finer spatial grid but increase search cost.

### Increase temporal resolution

```matlab
params_.NT = 80;
```

This gives the search more temporal resolution when avoiding moving obstacles, again at increased computational cost.

---

# Notes on the Demonstration

The repository is a compact research implementation intended to demonstrate the algorithmic framework in the associated paper.

Several modeling choices are deliberately simple:

- the AGV is represented as a disk;
- moving obstacles are circular;
- obstacle trajectories are known in advance;
- each obstacle follows a linear trajectory in the demo; and
- the planning horizon is fixed.

These assumptions make the implementation concise and make the central idea — **search in space-time followed by continuous optimization** — particularly easy to inspect.

For extensions to more general vehicle geometry, nonlinear dynamics, uncertain obstacle motion, or online prediction, the same overall first-search-then-optimization philosophy can be retained while replacing the corresponding models.

---

# Citation

If you use this repository, the source code, or the proposed first-search-then-optimization planning framework in your research, **please cite the following paper**:

> B. Li, Y. Zhang, Y. Ouyang, Y. Liu, X. Zhong, H. Cen, and Q. Kong,  
> “Fast Trajectory Planning for AGV in the Presence of Moving Obstacles: A Combination of 3-dim A* Search and QCQP,”  
> in *2021 33rd Chinese Control and Decision Conference (CCDC)*, pp. 7549–7554, 2021.  
> DOI: **10.1109/CCDC52312.2021.9602686**

## BibTeX

```bibtex
@inproceedings{li2021agv,
  title={Fast trajectory planning for AGV in the presence of moving obstacles: A combination of 3-dim A* search and QCQP},
  author={Li, Bai and Zhang, Youmin and Ouyang, Yakun and Liu, Yi and Zhong, Xiang and Cen, Hangjie and Kong, Qi},
  booktitle={2021 33rd Chinese Control and Decision Conference (CCDC)},
  pages={7549--7554},
  year={2021},
  organization={IEEE},
  doi={10.1109/CCDC52312.2021.9602686}
}
```

---

# License

This project is released under the **GNU General Public License v3.0 (GPL-3.0)**.

See [LICENSE](LICENSE) for details.

---

Copyright © 2021–2022 Bai Li.
