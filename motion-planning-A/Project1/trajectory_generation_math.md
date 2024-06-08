
## Mathematical Principles of Trajectory Generation

This project implements trajectory generation for path planning using a seventh-order polynomial interpolation scheme to approximate minimum snap trajectories. The following section outlines the core mathematical concepts and the detailed construction of the matrix system used to solve for the polynomial coefficients.

### Polynomial Representation

The trajectory is modeled as a seventh-order polynomial function of time:
$$
\mathbf{p}(t) = a_0 + a_1 t + a_2 t^2 + a_3 t^3 + a_4 t^4 + a_5 t^5 + a_6 t^6 + a_7 t^7
$$
where $$ \mathbf{p}(t) $$ is the position vector.

### Snap Minimization Approximation

Although seventh-order polynomials are not the exact solution to the minimum snap problem, they provide a good approximation by minimizing the integral of the snap squared:
$$
J = \int_{0}^{T} \left( \frac{d^4 \mathbf{p}(t)}{dt^4} \right)^2 dt
$$

### Matrix System Construction

To solve for the polynomial coefficients, we need to construct a matrix system based on the constraints of the trajectory. Here’s a detailed breakdown of how the matrix system is constructed:

#### 1. Waypoints and Segments

Given $$ N $$ waypoints, we have $$ N-1 $$ segments. Each segment is represented by a seventh-order polynomial, which has 8 coefficients. Therefore, we have a total of $$ 8(N-1) $$ unknown coefficients to solve for.

#### 2. Constraints

To determine these coefficients, we set up a system of linear equations based on the following constraints:

- **Initial and Final Constraints**:
  - Velocity, acceleration, and jerk are zero at the start and end positions.

- **Waypoint Constraints**:
  - The polynomial passes through each waypoint.

- **Continuity Constraints**:
  - Continuity of position, velocity, acceleration, jerk, snap, crackle, and pop at each interior waypoint.

#### 3. Formulating the Matrix System

Let's denote the normalized start times as $$ t_i $$ for $$ i = 0, 1, \ldots, N-1 $$. We construct the matrix $$ \mathbf{A} $$ and vector $$ \mathbf{b} $$ to encode these constraints:

1. **Initial Constraints**:

   - Velocity at $$ t_0 $$:
     $$
     7 t_0^6 a_7 + 6 t_0^5 a_6 + 5 t_0^4 a_5 + 4 t_0^3 a_4 + 3 t_0^2 a_3 + 2 t_0 a_2 + a_1 = 0
     $$

   - Acceleration at $$ t_0 $$:
     $$
     42 t_0^5 a_7 + 30 t_0^4 a_6 + 20 t_0^3 a_5 + 12 t_0^2 a_4 + 6 t_0 a_3 + 2 a_2 = 0
     $$

   - Jerk at $$ t_0 $$:
     $$
     210 t_0^4 a_7 + 120 t_0^3 a_6 + 60 t_0^2 a_5 + 24 t_0 a_4 + 6 a_3 = 0
     $$

2. **Waypoint Constraints**:

   For each waypoint $$ \mathbf{p}_i $$ at $$ t_i $$:
   $$
   \mathbf{p}_i = a_0 + a_1 t_i + a_2 t_i^2 + a_3 t_i^3 + a_4 t_i^4 + a_5 t_i^5 + a_6 t_i^6 + a_7 t_i^7
   $$

3. **Continuity Constraints**:

   For continuity at each interior waypoint $$ t_i $$ (for $$ i = 1, 2, \ldots, N-2 $$):

   - Position:
     $$
     \mathbf{p}(t_i^-) = \mathbf{p}(t_i^+)
     $$

   - Velocity:
     $$
     \frac{d\mathbf{p}}{dt}(t_i^-) = \frac{d\mathbf{p}}{dt}(t_i^+)
     $$

   - Acceleration:
     $$
     \frac{d^2\mathbf{p}}{dt^2}(t_i^-) = \frac{d^2\mathbf{p}}{dt^2}(t_i^+)
     $$

   - Jerk:
     $$
     \frac{d^3\mathbf{p}}{dt^3}(t_i^-) = \frac{d^3\mathbf{p}}{dt^3}(t_i^+)
     $$

   - Snap:
     $$
     \frac{d^4\mathbf{p}}{dt^4}(t_i^-) = \frac{d^4\mathbf{p}}{dt^4}(t_i^+)
     $$

   - Crackle:
     $$
     \frac{d^5\mathbf{p}}{dt^5}(t_i^-) = \frac{d^5\mathbf{p}}{dt^5}(t_i^+)
     $$

   - Pop:
     $$
     \frac{d^6\mathbf{p}}{dt^6}(t_i^-) = \frac{d^6\mathbf{p}}{dt^6}(t_i^+)
     $$

These constraints lead to a system of $$ 8(N-1) $$ linear equations. The matrix $$ \mathbf{A} $$ is constructed by placing the coefficients of the polynomials in each row corresponding to the constraints, and $$ \mathbf{b} $$ is the vector of known values (positions of waypoints, zeros for continuity constraints).

#### Solving the System

The system of equations $$ \mathbf{A} \mathbf{a} = \mathbf{b} $$ is solved using Eigen's `colPivHouseholderQr` solver, which provides the polynomial coefficients $$ \mathbf{a} $$.

```cpp
Eigen::VectorXd solveCoefficients(const std::vector<Eigen::Vector3f>& waypoints, const std::vector<double>& startTimes, char component) {
    // (Implementation details...)
    return A.colPivHouseholderQr().solve(b);
}
```

### Evaluating the Trajectory

Once the coefficients are determined, the `evaluateTrajectory` function evaluates the polynomial at any given time $$ t $$ to obtain the position of the trajectory.

```cpp
Eigen::Vector3f evaluateTrajectory(const Eigen::VectorXd& coeffsX, const Eigen::VectorXd& coeffsY, const Eigen::VectorXd& coeffsZ, const std::vector<double>& startTimes, double t) {
    // (Implementation details...)
    return point;
}
```

This detailed explanation illustrates how the trajectory generation algorithm constructs and solves the matrix system to achieve smooth and efficient path planning.
