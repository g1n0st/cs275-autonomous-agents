import taichi as ti
import numpy as np
from map import *
from path_planning import *
from agent import *

def render_agents(gui, env_map, agents, show_facing=False, show_path=False, show_visible_point=False):
    def to_screen_space(pos):
        return [pos[0] / env_map.boundary_size[0], pos[1] / env_map.boundary_size[1]]

    agents_pos = []
    visible_pos = []
    agents_indices = []
    path_begin = []
    path_end = []
    path_indices = []
    palette = []
    triangle_a = []
    triangle_b = []
    triangle_c = []
    for i in range(len(agents)):
        agent = agents[i]
        triangle_a.append(to_screen_space(np.array(agent.current_pos) + agent.facing * pedestrian_bouding_box_size))
        triangle_b.append(to_screen_space(np.array(agent.current_pos) + rotate(agent.facing * pedestrian_bouding_box_size, np.deg2rad(+140.0))))
        triangle_c.append(to_screen_space(np.array(agent.current_pos) + rotate(agent.facing * pedestrian_bouding_box_size, np.deg2rad(-140.0))))
        palette.append(agent.color)
        agents_pos.append(to_screen_space(agent.current_pos))
        visible_pos.append(to_screen_space(agent.visible_pos))
        agents_indices.append(i)
        path_begin.append(to_screen_space(agent.current_pos))
        for _ in range(agent.navigation_stop-1, len(agent.navigation_path)):
            p = agent.navigation_path[_]
            pos = to_screen_space(env_map.quadtree_node[p].actual_position())
            path_begin.append(pos)
            path_end.append(pos)
            path_indices.append(i)
        path_end.append(to_screen_space(agent.target_pos))
        path_indices.append(i)
    
    radius = pedestrian_bouding_box_size / env_map.boundary_size[0] * default_resolusion[0]
    gui.circles(pos=np.array(agents_pos, dtype=np.float32), radius=radius, palette=palette, 
                              palette_indices=np.array(agents_indices, dtype=np.int32))
    if show_visible_point:
        gui.circles(pos=np.array(visible_pos, dtype=np.float32), radius=radius/2, palette=palette, 
                                palette_indices=np.array(agents_indices, dtype=np.int32))
    if show_path:
        for i in range(len(path_indices)):
            gui.line(begin=path_begin[i], end=path_end[i], radius=radius / 6, color=palette[path_indices[i]])
    
    if show_facing:
        gui.triangles(a=np.array(triangle_a), b=np.array(triangle_b), c=np.array(triangle_c), color=0xFFFFFF)

def update_path_planning(agents):
    for agent in agents: agent.update_path_planning()

def find_neighboring_agents(agents):
    for agent in agents: agent.find_neighboring_agents()

def apply_routine_A(agents, dt):
    for agent in agents: agent.update_routine_A(dt)
    for agent in agents: agent.merge_update()

def apply_routine_B(agents, dt):
    for agent in agents: agent.update_routine_B(dt)
    for agent in agents: agent.merge_update()

def apply_routine_F(agents, dt):
    for agent in agents: agent.update_routine_F(dt)
    for agent in agents: agent.merge_update()

def apply_routine_C(agents, dt):
    for agent in agents: agent.update_routine_C(dt)
    for agent in agents: agent.merge_update()

def apply_routine_D(agents, dt):
    for agent in agents: agent.update_routine_D(dt)
    for agent in agents: agent.merge_update()

def apply_routine_E(agents, dt):
    for agent in agents: agent.update_routine_E(dt)
    for agent in agents: agent.merge_update()

def all_advance(agents, dt):
    for agent in agents: agent.advance(dt)