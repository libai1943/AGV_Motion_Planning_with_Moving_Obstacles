# Planning an AGV trajectory among moving obstacles

[Download ZIP](https://github.com/libai1943/AGV_Motion_Planning_with_Moving_Obstacles/archive/refs/heads/master.zip) · [Getting started](#run-one-scene) · [Guide](#follow-the-computation) · [Citation](#citation)

Two discs can pass through the same point safely if they arrive at different times. This MATLAB example searches over **position and time** to choose when the AGV should cross, then uses quadratic optimization to smooth the motion while keeping the discs separated at each optimization node. It accompanies the [CCDC 2021 paper](https://doi.org/10.1109/CCDC52312.2021.9602686).

```mermaid
flowchart LR
    A[Start, goal and moving discs] --> B[3D A* in x, y and time]
    B --> C[Coarse waypoints and interpolated guess]
    C --> D[AMPL model solved with IPOPT]
    D --> E[Position samples and solve status]
```

## Run one scene

Use MATLAB on Windows with a compatible AMPL executable and a licensed AMPL environment that supports this problem size. The checkout contains the IPOPT executable and support DLLs. Supply AMPL yourself and make `ampl` available to the MATLAB shell. The model exchanges files in the current directory, so use a writable copy with the `.mod`, `.run` and `.p` files together.

In MATLAB, change into the repository root and run the following. `RunMe` clears the workspace and closes open figures. A fixed random seed makes it easier to revisit a particular draw.

```matlab
rng(0)
RunMe
assert(ready_flag == 1, 'Check the current AMPL/IPOPT solve output')
plot(x, y, '.-')
axis equal
xlabel('x (m)')
ylabel('y (m)')
```

The driver chooses start and goal positions, generates obstacle tracks, searches for a coarse path, refines it and calls the protected dynamic display. A random scene can exhaust the search budget. If the driver stops before refinement, inspect the scene and search bounds. After a solve, check the current `ready_flag` and solver output before interpreting the trajectory. The driver calls its display even when the solver reports failure.

## Follow the computation

| File or function | Input → output | What to look for |
| --- | --- | --- |
| [RunMe.m](RunMe.m) | Parameters and random seed → a planning problem | The 40 m × 40 m workspace, disk radius of 2 m and fixed 40 s horizon are all set here. |
| [GenerateDynamicObstacles.m](GenerateDynamicObstacles.m) | Scene bounds → rows of start x/y, end x/y and radius | The planner reads the first `Nobs` tracks. Each obstacle translates linearly with time. |
| [SearchTrajViaAstar.m](SearchTrajViaAstar.m) | Start and target grid cells → coarse `x,y` samples | Each expansion advances the time index. `Nring` controls the spatial neighborhood and `max_iter` caps the exploration budget. |
| [RefinePathViaNLP.m](RefinePathViaNLP.m) | Coarse samples → optimized positions and `ready_flag` | `WriteIG` expands the search waypoints and estimates first and second derivatives for the numerical solver. |
| [C1.mod](C1.mod) | Waypoints, obstacle tracks, endpoints and workspace → a QCQP | Linear finite differences tie position, velocity and acceleration together. Squared-distance constraints keep the AGV outside each obstacle. |
| [r0.run](r0.run) and [ipopt.opt](ipopt.opt) | Model and warm start → `x.txt`, `y.txt`, `flag.txt` | A flag of 1 means the script matched “Optimal” in the solver message. |
| `DemonstrateDynamicResult.p` | Optimized positions → animated display | Compare the AGV with the obstacle at the *same time*. |

## What changes between search and refinement?

The default search uses a 20 × 20 spatial grid, 80 time layers and five obstacle tracks. Moving to a neighboring cell or waiting in place both consume one time step. The heuristic weights the remaining time-layer difference, while the transition cost also accounts for changes in grid motion.

The optimization uses **240 nodes**. Its objective weights squared speeds by 1, squared accelerations by 100 and deviations from the coarse positions by 0.0001. At each node it requires center separation of at least **1.1 times the sum of radii**. A successful local solve therefore gives a smoother position sequence under the model's sampled constraints. The source uses point-mass Cartesian dynamics with disk geometry.

To study a specific encounter, keep the scene fixed and compare each optimized point with the interpolated obstacle positions at `t = (i-1)*tf_max/(Nfe-1)`. To explore a different search resolution, change `NX` and `NY` in [RunMe.m](RunMe.m). Keep the code's `Nfe = 3*NT` relationship when varying temporal resolution because both the initial-guess writer and the AMPL model use that factor.


## Citation

If you use this repository, the source code, or the proposed first-search-then-optimization planning framework in your research, **please cite the following paper**:

> B. Li, Y. Zhang, Y. Ouyang, Y. Liu, X. Zhong, H. Cen, and Q. Kong,  
> “Fast Trajectory Planning for AGV in the Presence of Moving Obstacles: A Combination of 3-dim A* Search and QCQP,”  
> in *2021 33rd Chinese Control and Decision Conference (CCDC)*, pp. 7549–7554, 2021.  
> [DOI 10.1109/CCDC52312.2021.9602686](https://doi.org/10.1109/CCDC52312.2021.9602686)

### BibTeX

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

## License

This project is released under the **GNU General Public License v3.0 (GPL-3.0)**.

See [LICENSE](LICENSE) for details.

---

Copyright © 2021–2022 Bai Li.
