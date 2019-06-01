# Simple Quadcopter simulation
taken from [bullet 3 examples](https://github.com/bulletphysics/bullet3)

![](screenshot.PNG)

## Prerequisites

Install `pybullet`

```bash
pip install pybullet
```

## Usage

```bash
python quadrocopter.py
```

**Controls:**
 - Press `n` and `m` to decrease (resp. increase) a "base force" equally for all rotors.
 - Use the arrow keys `UP`, `DOWN`, `LEFT` and `RIGHT` to momentarily increase the force of an individual rotor.
 
 ### Reinforcement Learning Challenge
 
 Steer the quadrocopter to the position `[8.5, 5., 5.]` without collision
 and hover for 5 seconds. Use the quadrocopter's six distance sensors in 
 *up, down, forward, backward, left, right* directions.