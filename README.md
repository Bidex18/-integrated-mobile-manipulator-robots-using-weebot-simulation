# Integrated-mobile-manipulator-robots-using-weebot-simulation
Flexible automation in modern manufacturing requires integrated mobile-manipulator robots 
capable of dynamic navigation and precise manipulation. This project presents a fully simulated 
collaborative robotic system in Webots, consisting of a compact differential-drive mobile base 
and a 3-DoF RRR manipulator. The mobile platform (0.40×0.25×0.18 m) utilizes three distance 
sensors and GPS/compass modules. Navigation combines reactive obstacle avoidance with a 
global A* planner operating on a 0.05 m-resolution grid. Manipulator motion uses quintic 
polynomial trajectories respecting joint constraints, enhanced by an A* planner incorporating a 
neural-network heuristic, reducing search expansions by 35% without significantly 
compromising optimality.
The coordination between robots uses a simple protocol: upon arrival, the mobile robot sends a 
"READY" signal; the manipulator executes pick-and-place with quintic trajectory 
synchronization, ensuring 100% success even under simulated communication delays.
Extensive testing demonstrates navigation completion within 15 ± 3 s without collisions, 
averaging just 2.1 replans per run and maintaining 90% grasp success under ±5 cm placement 
variability. Results affirm robustness, responsiveness, and modularity, setting a foundation for 
advanced future developments.
