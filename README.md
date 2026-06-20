# Integrated Mobile-Manipulator Robot — Webots Simulation

A simulated collaborative robotic system combining a mobile robot base with a robotic arm, built in Webots to explore autonomous navigation and precision manipulation working together — the kind of integration used in flexible manufacturing automation.

## What it does

- Simulates a differential-drive mobile robot base (0.40 × 0.25 × 0.18 m) paired with a 3-DoF RRR manipulator arm.
- Navigation combines reactive obstacle avoidance with a global **A\* path planner** on a 0.05 m-resolution grid, using onboard distance sensors and GPS/compass modules.
- Manipulator motion uses **quintic polynomial trajectories** within joint constraints, with an enhanced A* planner using a neural-network heuristic that reduced search expansions by 35% without compromising path quality.
- Mobile base and manipulator coordinate via a simple handoff protocol: the mobile robot signals "READY" on arrival, and the manipulator executes pick-and-place with trajectory synchronization.

## Tools & Stack

- **Webots** (robot simulation)
- **MATLAB**

## Results

- Navigation completed in 15 ± 3 seconds per run with zero collisions, averaging 2.1 replans per run.
- 100% pick-and-place success even under simulated communication delays.
- 90% grasp success maintained under ±5 cm placement variability.
- 35% reduction in path-planning search expansions from the neural-network-enhanced heuristic.

## Files

- `rrr_controller/` — manipulator and navigation control code
- `Robot_.wbt` — Webots world/simulation file
- `Acw2.pdf` — full technical write-up and evaluation

## Status

Individual project. Fully simulated and tested in Webots.
