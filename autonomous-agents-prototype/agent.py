import numpy as np
import random
from map import *
from path_planning import *

class Agent:
    def __init__(self, id, env_map : EnvMap, current_pos, target_pos):
        self.id = id
        self.env_map = env_map
        self.color = random.randint(0, 0xFFFFFF)
        self.current_pos = current_pos
        self.visible_pos = current_pos
        self.target_pos = target_pos
        self.navigation_path = PmultiQ(self.env_map, self.current_pos, self.target_pos)
        self.navigation_stop = 0
        self.get_farthest_visible_point()

        self.facing = np.zeros(2, dtype=np.float32) # facing direction in the last time-step
        self.velocity = np.zeros(2, dtype=np.float32) # velocity in the last time-step

        # the original direction issued by either the higher-level path planning modules or by Routine A, 
        # whichever was executed most recently prior to the execution of Routine F
        self.backup_facing = np.zeros(2, dtype=np.float32)
        self.backup_velocity = np.zeros(2, dtype=np.float32)

        self.neighbors = []

        self.E_slow_down_factor = 1.0
        self.F_slow_down_factor = 1.0
        self.D_turning_factor = 0.0

        self.history_pos_list = []
        self.history_list_max_length = 25

    def test_visible(self, pos):
        if distance(self.current_pos, pos) > agent_perceptual_range:
            return False
        dy = (pos[1] - self.current_pos[1]) 
        dx = (pos[0] - self.current_pos[0])
        if abs(dy) > abs(dx): # |K| > 1
            for d in range(int(math.floor(abs(dy) * visibility_segment_sampling_density))):
                delta = d / visibility_segment_sampling_density
                rasterized_pos = (self.current_pos[0] + (dx/dy) * delta * sign(dy), self.current_pos[1] + delta * sign(dy))
                if not self.env_map.is_empty_position(rasterized_pos):
                    return False
        else: # |K| < 1
            for d in range(int(math.floor(abs(dx) * visibility_segment_sampling_density))):
                delta = d / visibility_segment_sampling_density
                rasterized_pos = (self.current_pos[0] + delta * sign(dx), self.current_pos[1] + (dy/dx) * delta * sign(dx))
                if not self.env_map.is_empty_position(rasterized_pos):
                    return False
        
        return True

    def get_farthest_visible_point(self, reset_path=False):
        if reset_path: self.navigation_stop = 0
        if self.test_visible(self.target_pos):
            self.visible_pos = self.target_pos
            self.navigation_stop = len(self.navigation_path)+1

        for p in range(self.navigation_stop, len(self.navigation_path)+1):
            if p == len(self.navigation_path):
                pos = self.target_pos # close to the target
            else:
                pos = self.env_map.quadtree_node[self.navigation_path[p]].actual_position()
            out_range = False
            if distance(self.current_pos, pos) > agent_perceptual_range:
                out_range = True
                # bi-sect
                pos0 = self.current_pos if p == 0 else self.env_map.quadtree_node[self.navigation_path[p-1]].actual_position()
                if distance(self.current_pos, pos) < distance(pos0, pos): # agent is actually towarding a long target
                    pos0 = self.current_pos
                if distance(self.current_pos, pos0) > agent_perceptual_range:
                    break
                left = distance(self.current_pos, pos0)
                right = distance(self.current_pos, pos)
                alpha = (agent_perceptual_range - left) / (right - left)
                # cut off to the agent perceptual range
                pos = (pos0[0] * (1 - alpha) + pos[0] * alpha, pos0[1] * (1 - alpha) + pos[1] * alpha) # 

            if self.test_visible(pos):
                self.visible_pos = pos
                if out_range:
                    break
            else:
                break
            
            self.navigation_stop = p+1
    
    # for debug
    def stuck_warning(self):
        self.history_pos_list.append(self.current_pos)
        if len(self.history_pos_list) > self.history_list_max_length:
            del self.history_pos_list[0]
        
        if len(self.history_pos_list) == self.history_list_max_length and \
           np.var(np.array(self.history_pos_list)[:, 0])+np.var(np.array(self.history_pos_list)[:, 1]) < 1e-2:
            print("---STUCK-WARNING---")
            print("current_pos=", self.current_pos)
            print("target_pos=", self.target_pos)
            print("visible_pos=", self.visible_pos)
            print("navigation_stop=", self.navigation_stop)
            print("|navigation_path|=", len(self.navigation_path))
            for p in range(len(self.navigation_path)):
                print(f"pos[{p}]=", self.env_map.quadtree_node[self.navigation_path[p]].actual_position())
    
    def get_velocity_from_path_planning(self):
        if distance(self.current_pos, self.target_pos) < TARGET_EPS:
            self.target_pos = self.env_map.get_valid_random_position()
            self.navigation_path = PmultiQ(self.env_map, self.current_pos, self.target_pos)
            self.get_farthest_visible_point(reset_path=True)
        else:
            self.get_farthest_visible_point(reset_path=False)
        
        dpos = np.array(self.visible_pos) - np.array(self.current_pos)
        if norm(dpos) < TARGET_EPS:
            normalized_dos = np.zeros_like(dpos)
        else:
            normalized_dos = dpos / norm(dpos)
        velocity = normalized_dos * agent_walk_velocity
        return normalized_dos, velocity

    def _find_neighboring_agents(self, facing):
        pos = np.array(self.current_pos)
        cell_x = int(pos[0] / mobile_cell_size)
        cell_y = int(pos[1] / mobile_cell_size)
        candidates = []
        for dX in range(-1, 2):
            for dY in range(-1, 2):
                if 0 <= cell_x+dX < self.env_map.mobile_map_size[0] and \
                   0 <= cell_y+dY < self.env_map.mobile_map_size[1]:
                    for agent in self.env_map.mobile_map[cell_x+dX][cell_y+dY]:
                        agent_pos = np.array(agent.current_pos)
                        dir = agent_pos - pos
                        dir_norm = norm(dir)
                        if dir_norm < agent_perceptual_range and \
                           dir.dot(facing) > dir_norm * cos_sensing_fan_angle and \
                           agent.id != self.id:
                            if self.test_visible(agent_pos): # make sure the agent is visible!
                                candidates.append((dir_norm, agent))
        
        candidates.sort()
        self.neighbors = []
        for candidate in candidates:
            self.neighbors.append(candidate[1])
            if len(self.neighbors) == maximum_attention_agents:
                return
    
    def decompose_vector(self, vector, facing):
        return vector.dot(facing) * facing, (vector - facing * vector.dot(facing))
    
    # Routine E: Avoid dangerously close pedestrians
    # This is the fail-safe behavior routine, 
    # reserved for emergencies due to the oc- casional failure of Routines C and D, 
    # since in highly dynamic situations predictions have a nonzero probability of being incorrect. 
    # Once a pedestrian perceives another pedestrian within its front safe area (Fig. 5(E)), 
    # it will resort to a simple but effective behavior—brake as soon as possible to a full stop, 
    # then try to turn to face away from the intruder, and proceed when the way ahead clears.
    def routine_E(self, facing, velocity, dt):
        # w and d depend on H's bounding box size 
        # and d is determined by H's current speed s.
        w = minimum_distance_allowed * 0.5
        d = minimum_distance_allowed * 0.5 + norm(velocity) * dt * 1.25
        for agent in self.neighbors:
            dir = np.array(agent.current_pos) - np.array(self.current_pos)
            nor_dir, tan_dir = self.decompose_vector(dir, facing)
            if norm(nor_dir) < d and norm(tan_dir) < w:
                self.E_slow_down_factor *= routine_E_slow_down_factor_ratio
                velocity *= self.E_slow_down_factor
                return facing, velocity
        
        self.E_slow_down_factor = min(1., self.E_slow_down_factor / routine_E_slow_down_factor_ratio)
        return facing, velocity

    # Routine C: Maintain separation in a moving crowd
    # For a pedestrian H, other pedestrians are considered to be in H's temporary crowd 
    # if they are moving in a similar direction to H and are situated within a parabolic region 
    # in front of H defined by y = −(4/R)x2 + R where R is the sensing range, 
    # y is oriented in H's forward direction and x is oriented laterally (Fig. 5(C)). 
    # To maintain a comfortable distance from each individual Ci in this temporary crowd, a directed repulsive force
    def routine_C(self, facing, velocity, dt):
        R = minimum_distance_allowed
        force = np.zeros_like(velocity)
        for agent in self.neighbors:
            dir = np.array(agent.current_pos) - np.array(self.current_pos)
            nor_dir, tan_dir = self.decompose_vector(dir, facing)
            if norm(nor_dir) < R - (4 / R) * (norm(tan_dir) ** 2) and \
                facing.dot(agent.facing) > cos_similarity:
                force += repulsiveness * (dir / norm(dir)) / (norm(dir) - minimum_distance_allowed)
        a = force / inertia
        nor_a, tan_a = self.decompose_vector(a, facing)
        tan_a *= crowding_factor
        velocity += (nor_a + tan_a) * dt
        facing = velocity / norm(velocity)
        return facing, velocity

    # Routine D: Avoid oncoming pedestrians
    # To avoid pedestrians not in one’s temporary crowd, 
    # a pedestrian H estimates its own velocity v and the velocities vi of neighbors pedestrians Ci. 
    # Two types of threats are considered here. 
    # By intersecting its own linearly extrapolated trajectory T with the trajectories Ti of each of the Ci, 
    # pedestrian H identifies potential collision threats of the first type: 
    # cross-collision (Fig. 5(D1)). 
    # In the case where the trajectories of H and Ci are almost parallel and will not intersect imminently, 
    # a head-on collision (Fig. 5(D2)) may still occur if their lateral separation is too small; 
    # hence, H measures its lateral separation from oncoming pedestrians. 
    def routine_D(self, facing, velocity, dt):
        R = minimum_distance_allowed
        # The most imminent potential collision
        imm_T_agent = INF
        imm_T_self = INF
        imm_C_star = None
        imm_collision_type = -1
        for agent in self.neighbors:
            dir = np.array(agent.current_pos) - np.array(self.current_pos)
            nor_dir, tan_dir = self.decompose_vector(dir, facing)
            # skip one’s temporary crowd
            if (norm(nor_dir) < R - (4 / R) * (norm(tan_dir) ** 2) and \
               facing.dot(agent.facing) > cos_similarity):
                continue

            if facing.dot(agent.facing) > -headon_cos_similarity:
                # H will estimate who will arrive first at the anticipated intersection point p
                T_self = INF
                T_agent = INF
                Tra_M = np.array([[facing[0], 0, -1, 0],
                                [facing[1], 0, 0, -1],
                                [0, agent.facing[0], -1, 0],
                                [0, agent.facing[1], 0, -1]], dtype=np.float32)
                if abs(np.linalg.det(Tra_M)) < 1e-3: # Singular Matrix
                        continue

                for _ in range(9): # considering both boundary side of the pedesitrans
                    _pos_self = np.array(self.current_pos) + \
                                rotate(self.facing, np.pi / 2) * minimum_distance_allowed * (_//3-1)
                    _pos_agent = np.array(agent.current_pos) + \
                                rotate(agent.facing, np.pi / 2) * minimum_distance_allowed * (_%3-1)
                    Tra_b = np.array([-_pos_self[0], 
                                    -_pos_self[1], 
                                    -_pos_agent[0], 
                                    -_pos_agent[1]])
                    Tra_x = np.linalg.inv(Tra_M) @ Tra_b
                    if Tra_x[0] < 0. or Tra_x[1] < 0.:
                        continue

                    if Tra_x[0] < T_self:
                        T_self = Tra_x[0]
                        T_agent = Tra_x[1]
                
                if T_self < imm_T_self:
                    imm_T_self = T_self
                    imm_T_agent = T_agent
                    imm_C_star = agent
                    imm_collision_type = CROSS_COLLISION
            
            else:
                self_nor_vel, self_tan_vel = self.decompose_vector(self.velocity, facing)
                agent_nor_vel, agent_tan_vel = self.decompose_vector(agent.velocity, facing)
                T = norm(nor_dir) / (norm(self_nor_vel) + norm(agent_nor_vel))
                assert(T > 0.)
                gap = norm(tan_dir + T * (agent_tan_vel - self_tan_vel))
                if gap < minimum_distance_allowed and T < imm_T_self:
                    imm_T_self = T
                    imm_T_agent = T
                    imm_C_star = agent
                    imm_collision_type = HEADON_COLLISION
            
        if imm_collision_type == HEADON_COLLISION: # head-on collision
            # print("head-on collision", facing.dot(imm_C_star.facing))
            dir = np.array(imm_C_star.current_pos) - np.array(self.current_pos)
            nor_dir, tan_dir = self.decompose_vector(dir, facing)

            if sign(np.cross(nor_dir, tan_dir)) != sign(self.D_turning_factor):
                self.D_turning_factor = 0.0
            self.D_turning_factor += sign(np.cross(nor_dir, tan_dir)) * turning_factor_ratio
            self.D_turning_factor = max(min(self.D_turning_factor, max_headon_collision_avoidance_degree), \
                                        -max_headon_collision_avoidance_degree)

            velocity = rotate(velocity, np.deg2rad(self.D_turning_factor))
            facing = velocity / norm(velocity)
        elif imm_collision_type == CROSS_COLLISION:
            # print("cross collision", facing.dot(imm_C_star.facing))
            dir = np.array(imm_C_star.current_pos) - np.array(self.current_pos)
            nor_dir, tan_dir = self.decompose_vector(dir, facing)

            # If H determines that it will arrive sooner, 
            # it will increase its speed and turn slightly away from C*
            if imm_T_self < imm_T_agent:
                velocity *= increase_speed_ratio

                if sign(np.cross(nor_dir, tan_dir)) != sign(self.D_turning_factor):
                    self.D_turning_factor = 0.0
                self.D_turning_factor += sign(np.cross(nor_dir, tan_dir)) * turning_factor_ratio
                self.D_turning_factor = max(min(self.D_turning_factor, max_cross_collision_avoidance_degree), \
                                            -max_cross_collision_avoidance_degree)

                velocity = rotate(velocity, np.deg2rad(self.D_turning_factor))
                facing = velocity / norm(velocity)
            # otherwise, it will decrease its speed and turn slightly towards C∗
            else:
                velocity *= decrease_speed_ratio

                if -sign(np.cross(nor_dir, tan_dir)) != sign(self.D_turning_factor):
                    self.D_turning_factor = 0.0
                self.D_turning_factor += -sign(np.cross(nor_dir, tan_dir)) * turning_factor_ratio
                self.D_turning_factor = max(min(self.D_turning_factor, max_cross_collision_avoidance_degree), \
                                            -max_cross_collision_avoidance_degree)

                velocity = rotate(velocity, np.deg2rad(self.D_turning_factor))
                facing = velocity / norm(velocity)
        else: # no potential collision
            self.D_turning_factor -= sign(self.D_turning_factor) * turning_factor_ratio # reset turning

        return facing, velocity
    
    # Routine A: Static obstacle avoidance
    # If there is a nearby obstacle in the direction of locomotion, 
    # lateral directions to the left and right are tested 
    # until a less cluttered direction is found (Fig. 4(b)).
    # If a large angle (currently set to 90-deg) must be swept before a good direction is found,
    # then the pedestrian will start to slow down,
    # which mimics the behavior of a real person upon encountering a tough array of obstacles;
    # i.e., slow down while turning the head to look around, then proceed.
    def routine_A(self, facing, velocity, dt):
        # temporarily disabling the two routines 
        # and letting the pedestrian accurately follow the detailed path, which already avoids obstacles
        if distance(self.current_pos, self.target_pos) <= pedestrian_bouding_box_scale * stationary_cell_size:
            return facing, velocity
        angle = 0.
        while angle < 180:
            # If a large angle (currently set to 90-deg) must be swep
            w = routine_A_slow_down_factor_ratio if angle >= 90 else 1.
            if self.test_visible(np.array(self.current_pos) \
                                 + pedestrian_bouding_box_size * rotate(facing, np.deg2rad(angle)) \
                                 + pedestrian_bouding_box_scale * w * dt * rotate(velocity, np.deg2rad(angle))):
                velocity = w * rotate(velocity, np.deg2rad(angle))
                facing = rotate(facing, np.deg2rad(angle))
                break
            if self.test_visible(np.array(self.current_pos) \
                                 + pedestrian_bouding_box_size * rotate(facing, np.deg2rad(-angle)) \
                                 + pedestrian_bouding_box_scale * w * dt * rotate(velocity, np.deg2rad(-angle))):
                velocity = w * rotate(velocity, np.deg2rad(-angle))
                facing = rotate(facing, np.deg2rad(-angle))
                break
            angle += routine_A_angle_density

        self.backup_facing = facing
        self.backup_velocity = velocity
        return facing, velocity

    # Routine F: Verify new directions relative to obstacles
    # Since the reactive behavior routines are executed sequentially (see Section 4.3.1), 
    # motor control commands issued by Routines C, D or E
    # to avoid pedestrians may counteract those issued by Routines A or B to avoid obstacles, 
    # thus steering the pedestrian towards obstacles again. 
    # To avoid this, the pedestrian checks the new direction against surrounding obstacles once more. 
    # If the way is clear, it proceeds. Otherwise, the original direction issued by 
    # either the higher-level path planning modules or by Routine A, 
    # whichever was executed most recently prior to the execution of Routine F, 
    # will be used instead. However, 
    # occasionally this could lead the pedestrian toward future collisions with other pedestrians (Fig. 5(F)) 
    # and, if so, it will simply slow down to a stop, 
    # let those threatening pedestrians pass, and proceed.
    def routine_F(self, facing, velocity, dt):
        if not self.test_visible(np.array(self.current_pos) \
                                 + pedestrian_bouding_box_size * facing \
                                 + pedestrian_bouding_box_scale * velocity * dt):
            facing = self.backup_facing
            velocity = self.backup_velocity
            w = minimum_distance_allowed * 0.5
            d = minimum_distance_allowed * 0.5 + norm(velocity) * dt * 1.25
            for agent in self.neighbors:
                dir = np.array(agent.current_pos) - np.array(self.current_pos)
                nor_dir, tan_dir = self.decompose_vector(dir, facing)
                if norm(nor_dir) < d and norm(tan_dir) < w:
                    self.F_slow_down_factor *= routine_F_slow_down_factor_ratio
                    velocity *= self.F_slow_down_factor
                    return facing, velocity
        
        self.F_slow_down_factor = min(1., self.F_slow_down_factor / routine_F_slow_down_factor_ratio)
        return facing, velocity
    
    # Routine B: Static obstacle avoidance in a complex turn
    # When a pedestrian needs to make a turn that cannot be finished in one step, 
    # it will consider turns with increasing curvatures in both directions, 
    # starting with the side that permits the smaller turning angle, 
    # until a collision-free turn is found (Fig. 5(B)). 
    # If the surrounding space is too cluttered, 
    # the curve is likely to degenerate, causing the pedestrian to stop and turn on the spot. 
    # The turn test is implemented by checking sample points along a curve 
    # with interval equal to the distance of one step of the pedestrian 
    # moving with the anticipated turn speed.
    def routine_B(self, facing, velocity, dt):
        # TODO(changyu): skip this routine for prototype
        return facing, velocity

    def update_path_planning(self):
        if blending_previous_velocity:
            self._facing, self._velocity = self.get_velocity_from_path_planning()
            self.velocity = blending_velocity_weight * self.velocity + \
                            (1 - blending_velocity_weight) * self._velocity
            self.facing = self.velocity / norm(self.velocity)
        else:
            self.facing, self.velocity = self.get_velocity_from_path_planning()
        self.backup_facing = self.facing
        self.backup_velocity = self.velocity
    
    def find_neighboring_agents(self):
        self._find_neighboring_agents(self.facing)
    
    def update_routine_A(self, dt):
        if norm(self.velocity) < TARGET_EPS: return
        self._facing, self._velocity = self.routine_A(self.facing, self.velocity, dt)
    
    def update_routine_B(self, dt):
        if norm(self.velocity) < TARGET_EPS: return
        self._facing, self._velocity = self.routine_B(self.facing, self.velocity, dt)

    def update_routine_F(self, dt):
        if norm(self.velocity) < TARGET_EPS: return
        self._facing, self._velocity = self.routine_F(self.facing, self.velocity, dt)
    
    def update_routine_D(self, dt):
        if norm(self.velocity) < TARGET_EPS: return
        self._facing, self._velocity = self.routine_D(self.facing, self.velocity, dt)
    
    def update_routine_E(self, dt):
        if norm(self.velocity) < TARGET_EPS: return
        self._facing, self._velocity = self.routine_E(self.facing, self.velocity, dt)
    
    def update_routine_C(self, dt):
        if norm(self.velocity) < TARGET_EPS: return
        self._facing, self._velocity = self.routine_C(self.facing, self.velocity, dt)
    
    def merge_update(self):
        self.facing = self._facing
        self.velocity = self._velocity
                        
    def advance(self, dt):
        self.current_pos = (self.current_pos[0] + self.velocity[0] * dt, 
                            self.current_pos[1] + self.velocity[1] * dt)

