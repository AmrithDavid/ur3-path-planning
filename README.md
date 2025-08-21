# UR3 Robot Path Planning System

ROS2-based path planning module for UR3 portrait-drawing robot. Converts edge-detected images into optimised robot trajectories.

## Key Achievements

- **30-40% reduction** in drawing time through advanced path optimisation algorithms
- **<200ms** processing time for complex portraits (50+ paths)
- **±2mm** accuracy in robot workspace transformation
- Successfully deployed on physical UR3 robot for live demonstrations

## Technical Implementation

### Core Algorithms
- **2-opt Optimisation**: Iteratively swaps path segments to minimise travel distance (guarantees local optimum)
- **Simulated Annealing**: Probabilistic optimisation for complex drawings with 30+ paths
- **Douglas-Peucker**: Contour simplification while preserving shape fidelity
- **Bilinear Interpolation**: Accurate 2D to 3D coordinate transformation for non-rectangular canvases

### System Components

**`path_planner.py`**
- OpenCV contour extraction from binary edge images
- Coordinate transformation with configurable safety margins
- Handles arbitrary canvas orientations via 4-point calibration

**`path_optimiser.py`**
- Implements 2-opt and simulated annealing algorithms
- Auto-selects optimal algorithm based on complexity
- Achieves 30-40% path length reduction

**`tool_path_planner_node.py`**
- ROS2 node for real-time image processing
- Publishes JSON-formatted toolpaths to robot control system
- Integrates with team's localisation and control modules

## Technologies

**Stack**: Python, ROS2 Humble, OpenCV, NumPy  
**Robot**: Universal Robots UR3  
**Integration**: MoveIt motion planning, YAML configuration

---

*My contribution to a team project building an autonomous portrait-drawing robot. Full system includes image processing, control, and localisation modules.*
