# Advanced 3D Motion Planning and Trajectory Generation for Autonomous Quadcopters

I developed a 3D motion planning system for a quadcopter using a comprehensive engineering design process. This involved coding components for obstacle parsing, lattice construction, A* pathfinding, and polynomial trajectory generation, integrated and tested rigorously through EnvData.cpp, Lattice.cpp, trajectory.cpp, and main.cpp. The project successfully generated and visualized efficient flight paths in a 3D environment, showcasing my ability to execute detailed design and implementation steps.

The following images typify the results of the project:

![Results1](motion-planning-A/Project1/path.png)

![Results2](motion-planning-A/Project1/trajectory.png)

## Theory

### A* Pathfinding

I implemented the A* pathfinding algorithm, renowned for its efficiency in finding the shortest path between two nodes by combining Dijkstra's algorithm and a greedy best-first search. My implementation leverages the cost function:

$$
f(n) = g(n) + h(n)
$$

where:
- \( g(n) \) is the cost from the start node to node \( n \).
- \( h(n) \) is the heuristic estimate of the cost from node \( n \) to the goal.

By carefully tuning the heuristic, I ensured optimal performance and accuracy in navigating complex 3D environments, demonstrating my ability to apply advanced algorithmic concepts to practical problems.

### Seventh-Order Polynomials for Trajectory Generation

I utilized seventh-order polynomials to generate smooth, minimum snap trajectories for the quadcopter. The polynomial trajectory is represented as:

$$
\mathbf{p}(t) = a_0 + a_1 t + a_2 t^2 + a_3 t^3 + a_4 t^4 + a_5 t^5 + a_6 t^6 + a_7 t^7
$$

The goal was to minimize the snap, defined as the fourth derivative of position. I achieved this by optimizing the objective function:

$$
J = \int_{0}^{T} \left( \frac{d^4 \mathbf{p}(t)}{dt^4} \right)^2 dt
$$

This approach ensured the quadcopter's smooth and efficient movement through the environment, reflecting my proficiency in applying complex mathematical models to real-world engineering challenges. For an in-depth understanding, refer to the [Trajectory Planning Math](motion-planning-A/Project1/trajectory_generation_math.md) file in this repository. 

## Description

- **EnvData.cpp/h**: Handles environment data including obstacles.
- **Lattice.cpp/h**: Constructs a lattice for pathfinding.
- **main.cpp**: Main program that integrates obstacle parsing, lattice creation, pathfinding, and trajectory generation.
- **trajectory.cpp/h**: Generates polynomial trajectories from waypoints.
- **show_lattice.py**: Visualizes the lattice and obstacles.
- **show_path.py**: Visualizes the path found by the A* algorithm.
- **show_trajectory.py**: Visualizes the trajectory generated for the path.

### Dependencies

The project relies on several dependencies for both the C++ and Python components. Below is a list of the required libraries and their purposes.

#### C++ Dependencies

- **Eigen**: A C++ template library for linear algebra. It is used for matrix and vector operations, particularly in solving the polynomial coefficients.

To install Eigen, download it from the [official Eigen website](http://eigen.tuxfamily.org/).

#### Python Dependencies

- **Matplotlib**: A plotting library for creating static, animated, and interactive visualizations in Python.
- **Pandas**: A data manipulation and analysis library for Python. It provides data structures and functions needed to work with structured data seamlessly.

To install these Python libraries, you can use pip:
```bash
pip install matplotlib pandas
```

## Usage

1. **Compile the project**:
   Open the solution `motion-planning-controls.sln` in your preferred IDE (e.g., Visual Studio) and compile the project.

2. **Run the main program**:
   Execute `main.cpp` to parse obstacles, create a lattice, find a path using A*, and generate a trajectory.

3. **Visualize the results**:
   Use the provided Python scripts to visualize the obstacles, lattice, path, and trajectory.
   - `show_lattice.py`
   - `show_path.py`
   - `show_trajectory.py`

Each run of the main program will generate CSV files that are used by the visualization scripts. Ensure that these scripts are executed after running the main program to see the latest results.

## Notes

- The file `motion-planning-controls.sln` is used to manage the project solution in Visual Studio.
- The name "controls" in the solution file is a misnomer; the project focuses on motion planning and trajectory generation, not control.

