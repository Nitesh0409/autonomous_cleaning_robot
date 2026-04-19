# Artificial Potential Field (APF) Theory

This document provides a formal mathematical derivation of the navigation algorithms implemented in the `robot` package for autonomous holonomic navigation.

## 1. Concept Overview
The Artificial Potential Field (APF) methodology treats a robotic agent as a point particle moving through a configuration space ($q \in \mathcal{C}$) under the influence of a potential function $U(q)$. The robot follows the negative gradient of this potential to reach its goal while avoiding obstacles.

$$ \mathbf{F}(q) = -\nabla U(q) $$

The total potential is the superposition of an attractive field and a repulsive field:

$$ U(q) = U_{att}(q) + U_{rep}(q) $$

---

## 2. Attractive Potential ($U_{att}$)
The attractive potential is modeled as a parabolic well centered at the goal position ($q_{goal}$). This ensures that the attractive force increases linearly with distance.

$$ U_{att}(q) = \frac{1}{2} \xi \rho^2(q, q_{goal}) $$

Where:
- $\xi$ is the attractive gain (**`k_att`**).
- $\rho(q, q_{goal})$ is the Euclidean distance between the robot and the goal.

The resulting attractive force $\mathbf{F}_{att}$ is:

$$ \mathbf{F}_{att}(q) = -\nabla U_{att}(q) = -\xi (q - q_{goal}) $$

In the implementation, we use a normalized version for consistent velocity scaling:
$$ \mathbf{F}_{att} = -\xi \frac{q - q_{goal}}{\|\rho\|} $$

---

## 3. Repulsive Potential ($U_{rep}$)
To ensure collision avoidance, obstacles generate a repulsive potential that increases as the robot approaches. This potential is only active within an influence range $\rho_0$ (**`d0`**).

$$ U_{rep}(q) = \begin{cases} \frac{1}{2} \eta \left( \frac{1}{\rho(q)} - \frac{1}{\rho_0} \right)^2 & \text{if } \rho(q) \leq \rho_0 \\ 0 & \text{if } \rho(q) > \rho_0 \end{cases} $$

Where:
- $\eta$ is the repulsive gain (**`k_rep`**).
- $\rho(q)$ is the shortest distance to the obstacle surface.

The repulsive force $\mathbf{F}_{rep}$ is the gradient of this potential:

$$ \mathbf{F}_{rep}(q) = \eta \left( \frac{1}{\rho(q)} - \frac{1}{\rho_0} \right) \frac{1}{\rho^2(q)} \nabla \rho(q) $$

### 3.1 Geometric Clustering Fix
In our "Advanced" implementation, $\rho(q)$ is refined by identifying discrete obstacle clusters. For a cluster with centroid $c$ and radius $r$:
$$ \rho(q) = \|q - c\| - r $$

---

## 4. Harmonic Escape (Vortex Fields)
Standard APF suffers from "Local Minima" (stable states where $\mathbf{F}_{att} + \mathbf{F}_{rep} = 0$, but the goal is not reached). To mitigate this, we implement a **Harmonic Vortex Force**.

When an obstacle is detected, a tangential force $\mathbf{F}_{curl}$ is added to "slide" the robot around the obstacle surface:

$$ \mathbf{F}_{curl} \perp \mathbf{F}_{rep} $$

This is calculated as:
$$ \mathbf{F}_{curl} = \mathbf{k}_{curl} \cdot \text{sign}(\text{yaw}_{relative}) \cdot \begin{bmatrix} 0 & 1 \\ -1 & 0 \end{bmatrix} \mathbf{F}_{rep} $$

Where $\mathbf{k}_{curl}$ is the "Vortex Gain." If the robot stagnates (detected by the **Harmonic Trap Detection** logic), this gain is dynamically increased to force an escape.

---

## 5. Holonomic Velocity Mapping
Since the robot uses a Mecanum drive, the resultant force $\mathbf{F}_{total}$ is directly mapped to linear velocities $v_x$ and $v_y$ in the robot's body frame using a standard rotation matrix transform based on the current yaw $\theta$.
