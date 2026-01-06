# Controller Module

## Overview

This module implements a **Pure Pursuit path-following controller** for a mobile robot in **ROS 2 Humble**.  
Once a smooth spline path is generated from user-selected waypoints, the controller tracks the path using a geometric lookahead-based strategy.

The robot motion is **explicitly triggered by the user** (via a terminal key press), allowing the lookahead point and path geometry to be inspected in RViz **before execution**.

---

## Controller Description

- **Path input**: `nav_msgs/Path` generated from spline interpolation  
- **Control method**: Pure Pursuit (curvature-based tracking)  
- **Lookahead point**:
  - Selected along the path using **arc-length parameterization**
  - Ensures smooth, forward-only tracking
- **Motion profile**:
  - Supports constant velocity
  - Extendable to trapezoidal (constant acceleration) profiles


---

## User Interaction

- Waypoints can be selected incrementally (e.g., via RViz)
- Robot motion starts only after pressing:
  - **`w`** → start following the path  
  - **`s`** → stop the robot  

---

## Demo

### Path Following Demonstration

🎥 *Video: Robot following a spline path using Pure Pursuit*  
*(Insert demo video or GIF here)*

