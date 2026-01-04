# Path Planning Module

## Overview

This branch implements an **interactive path planning system** for a mobile robot in **ROS 2 Humble**.  
Users incrementally select waypoints, and the planner generates a smooth path that can be traversed using either **constant velocity** or **constant (trapezoidal) acceleration** motion profiles.

The system cleanly separates **geometric path generation** from **time parameterization**, enabling flexible motion execution and controller integration.

---

## Key Features

- Interactive waypoint selection via **RViz**
- Smooth cubic spline–based path generation
- Continuous position, velocity, and acceleration (C² continuity)
- Visualization in **RViz** and **Gazebo**
- Supports mapped and unmapped environments

---

## Path Planning Approach

1. **Waypoint Selection**  
   Waypoints are appended interactively (e.g., using RViz’s *Publish Point* tool).

2. **Geometric Path Generation**  
   A parametric path  
   \[
   p(s) = [x(s), y(s)]
   \]  
   is generated using cubic splines, where \( s \) approximates arc length.

---

## Demonstrations

### 1. Path Generation with Map Present

Waypoints are selected on a loaded map, and the planner generates a smooth, map-aligned path.

📹 **Video Demo:** *(add link here)*

---

### 2. Path Generation without Map

Path generation in an empty environment using only the odometry frame.

📹 **Video Demo:** *(add link here)*

---

### 3. Path Planning with Obstacle Avoidance

Path generation in the presence of obstacles, demonstrating obstacle-aware planning.

📹 **Video Demo:** *(add link here)*

---

## Visualization

- **Waypoints:** red markers  
- **Planned path:** continuous line strip  
- **Robot model and frames:** displayed via TF in RViz  

---

## Notes

- This module focuses on **path planning**, not low-level control.
- Time parameterization is designed for downstream controller usage.
- The architecture supports future extensions such as:
  - Online replanning
  - Advanced obstacle handling
  - Controller integration (Pure Pursuit, MPC, etc.)

---
