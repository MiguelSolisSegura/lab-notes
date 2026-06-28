## Motivation

Describe why you started this project and what gap or question you wanted to explore.

## Background

Provide any relevant theory or prior work. For example, inverse kinematics (IK) is the problem of finding joint angles $\theta_1, \theta_2, \ldots, \theta_n$ such that the end-effector reaches a desired position $\mathbf{p}$.

For a 2-DOF planar arm the forward kinematics are:

$$
\begin{bmatrix} x \\ y \end{bmatrix} =
\begin{bmatrix}
  l_1 \cos\theta_1 + l_2 \cos(\theta_1 + \theta_2) \\
  l_1 \sin\theta_1 + l_2 \sin(\theta_1 + \theta_2)
\end{bmatrix}
$$

## Approach

Explain your methodology. Code blocks work out of the box:

```python
import numpy as np

def fabrik_backward(joints, target, lengths):
    """One backward pass of the FABRIK algorithm."""
    joints[-1] = target
    for i in range(len(joints) - 2, -1, -1):
        d = np.linalg.norm(joints[i] - joints[i + 1])
        joints[i] = joints[i + 1] + (joints[i] - joints[i + 1]) * (lengths[i] / d)
    return joints

def fabrik_forward(joints, base, lengths):
    """One forward pass of the FABRIK algorithm."""
    joints[0] = base
    for i in range(1, len(joints)):
        d = np.linalg.norm(joints[i] - joints[i - 1])
        joints[i] = joints[i - 1] + (joints[i] - joints[i - 1]) * (lengths[i - 1] / d)
    return joints
```

The algorithm converges when the end-effector is within tolerance $\epsilon$ of the target:

$$
\|\mathbf{p}_n - \mathbf{p}^*\| < \epsilon
$$

## Results

Describe your results and key findings. Include images or GIFs if useful — just place them in `projects/assets/` and reference them like:

```
![my result](projects/assets/my-result.gif)
```

## What's Next

Outline open questions or next steps you plan to explore.
