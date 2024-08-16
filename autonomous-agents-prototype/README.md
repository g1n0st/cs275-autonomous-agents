# UCLA CS275 Course Project: Autonomous Agents Prototype

## Installation

OS Env: Windows, Linux, Mac OS

python>=3.6

Install the latest Taichi and NumPy:
```
python3 -m pip install -U taichi numpy
```

## Usage
```
python3 main.py
```

## Code Structure

#### map.py
Define environmental data structure, grid map for visual sensing, quadtree map for path planning, and mobile map for neighbor agent querying.
### path_planning.py
Implement a customized A-star path planning algorithm for finding a valid path toward the agent's goal.
### agent.py
Define the autonomous agent model, including perception, navigational behaviors, and cognitive control.
### control.py
Define the high-level controller responsible for controlling the entire crowd in parallel.
### utils.py
Define the parameters in the scene.