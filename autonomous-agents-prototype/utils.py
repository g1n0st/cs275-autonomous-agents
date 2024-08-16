import numpy as np
import math

# visualization-related constants
default_resolusion = (1440, 720)

# environmental-map-related constants
stationary_cell_size = 0.25
mobile_cell_size = 5.0
pedestrian_bouding_box_size = 0.25

DXY = [[-1, -1], [-1, 0], [-1, 1], [0, -1], [0, 1], [1, -1], [1, 0], [1, 1]]

VALID = 0
INVALID = 1

EMPTY = 0
WALL = 1
AIR_WALL = 2
DESK = 3

INF = 10000.0

Lt = 3 # every target is expanded on a quadtree map until it touches any node at a certain level Lt

# agent-related constants
TARGET_EPS = 0.1
agent_perceptual_range = 5.0 # pedestrian's visual sensing range (currently set to 5 meters)
cos_sensing_fan_angle = 0.5 # 120-degree

# This is motivated by the fact that, at any given time, 
# people usually pay attention only to a limited number of other peopl
maximum_attention_agents = 16

# If a large angle (currently set to 90-deg) must be swept before a good direction is found,
# then the pedestrian will start to slow down,
routine_A_slow_down_factor_ratio = 0.4
routine_A_angle_density = 10.0

# act as ans exponential damping factor for routine E
routine_E_slow_down_factor_ratio = 0.6

# predefined mini-mum distance allowed between H and other pedestrians 
# (usually 2.5 times H’s bounding box size). 
pedestrian_bouding_box_scale = 2.5
minimum_distance_allowed = pedestrian_bouding_box_scale * pedestrian_bouding_box_size

# perceived “repulsiveness” to H (currently set to −0.025 for all pedestrians)
repulsiveness = -0.025

cos_similarity = 0.967 # cos(20-deg.)
headon_cos_similarity = 0.84 # cos(33-deg.)
inertia = 50.0

# The crowding factor wi determines H's willingness to "follow the crowd", 
# with a smaller value of wi giving H a greater tendency to do so (currently 1.0 ≤ wi ≤ 5.0).
crowding_factor = 2.5

increase_speed_ratio = 1.2
decrease_speed_ratio = 0.8
# cross_collision_avoidance_radian = (25.0 / 360.0) * 2 * np.pi
# headon_collision_avoidance_radian = (25.0 / 360.0) * 2 * np.pi

turning_factor_ratio = 5.0
max_headon_collision_avoidance_degree = 55.0
max_cross_collision_avoidance_degree = 55.0

HEADON_COLLISION = 0
CROSS_COLLISION = 1

routine_F_slow_down_factor_ratio = 0.1

rasterizing_eps = 1e-3
agent_walk_velocity = 1.55
visibility_segment_sampling_density = 5

blending_previous_velocity = False
blending_velocity_weight = 0.5

# util functions
def distance(a, b):
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

def norm(a):
    return math.sqrt(a[0]**2 + a[1]**2)

def sign(a):
    return -1 if a < 0 else 1

# rotate a 2D matrix counterclockwise
def rotate(a, A):
    return np.array([a[0] * np.cos(A) + a[1] * np.sin(A), 
                    -a[0] * np.sin(A) + a[1] * np.cos(A)])