import taichi as ti
import numpy as np
from map import *
from path_planning import *
from agent import *
from control import *

ti.init(arch=ti.cpu)
random.seed(114514)

env_map = EnvMap(objects=[Object(8.0, 13.0, 14.0, 18.0, WALL),
                          Object(25.0, 10.0, 27.0, 20.0, WALL),
                          Object(32.0, 10.0, 34.0, 20.0, WALL),
                          Object(39.0, 10.0, 41.0, 20.0, WALL),
                          Object(46.0, 10.0, 48.0, 20.0, WALL),

                          Object(5.0, 3.0, 7.0, 3.5, WALL),
                          Object(9.0, 3.0, 11.0, 3.5, WALL),
                          Object(13.0, 3.0, 15.0, 3.5, WALL),
                          Object(17.0, 3.0, 19.0, 3.5, WALL),
                          Object(21.0, 3.0, 23.0, 3.5, WALL),
                          Object(25.0, 3.0, 27.0, 3.5, WALL),
                          Object(29.0, 3.0, 31.0, 3.5, WALL),
                          Object(33.0, 3.0, 35.0, 3.5, WALL),
                          Object(37.0, 3.0, 39.0, 3.5, WALL),
                          Object(41.0, 3.0, 43.0, 3.5, WALL),
                          Object(45.0, 3.0, 47.0, 3.5, WALL),
                          Object(49.0, 3.0, 51.0, 3.5, WALL),
                          Object(53.0, 3.0, 55.0, 3.5, WALL),
                          
                          Object(5.0, 26.0, 7.0, 26.5, WALL),
                          Object(9.0, 26.0, 11.0, 26.5, WALL),
                          Object(13.0, 26.0, 15.0, 26.5, WALL),
                          Object(17.0, 26.0, 19.0, 26.5, WALL),
                          Object(21.0, 26.0, 23.0, 26.5, WALL),
                          Object(25.0, 26.0, 27.0, 26.5, WALL),
                          Object(29.0, 26.0, 31.0, 26.5, WALL),
                          Object(33.0, 26.0, 35.0, 26.5, WALL),
                          Object(37.0, 26.0, 39.0, 26.5, WALL),
                          Object(41.0, 26.0, 43.0, 26.5, WALL),
                          Object(45.0, 26.0, 47.0, 26.5, WALL),
                          Object(49.0, 26.0, 51.0, 26.5, WALL),
                          Object(53.0, 26.0, 55.0, 26.5, WALL),
                          
                          Object(0.0, 0.0, 0.5, 30.0, WALL),
                          Object(59.5, 0.0, 60.0, 30.0, WALL),
                          Object(0.0, 0.0, 60.0, 0.5, WALL),
                          Object(0.0, 29.5, 60.0, 30.0, WALL)])

_agents = []

for i in range(250):
    _agents.append(Agent(i, env_map, env_map.get_valid_random_position(), env_map.get_valid_random_position()))
        
agents = []
option = 1
if option == 0:
    agents = _agents
if option == 1:
    for i in range(250):
        # if 32.0 <= _agents[i].current_pos[0] <= 40.0 and 0 <= _agents[i].current_pos[1] <= 8.0:
        if _agents[i].id != 123:
            agents.append(_agents[i])
            # print(_agents[i].id)
elif option == 2:
    # agents.append(_agents[38])
    # agents.append(_agents[8])
    # agents.append(_agents[60])
    # agents.append(_agents[70])
    # agents.append(_agents[5])
    # agents.append(_agents[45])
    '''
    agents.append(_agents[28])
    agents.append(_agents[39])
    agents.append(_agents[48])
    '''

    # agents.append(_agents[6])
    # agents.append(_agents[23])
    # agents.append(_agents[18])

gui = ti.GUI('Autonomous Agent', default_resolusion)

frame_num = 480
substeps = 2
dt = 1 / 30

for frame in range(frame_num):
    for i in range(substeps):
        env_map.update_mobile_map(agents)
        update_path_planning(agents)
        find_neighboring_agents(agents)
        apply_routine_C(agents, dt)
        apply_routine_A(agents, dt)
        apply_routine_B(agents, dt)
        apply_routine_F(agents, dt)
        apply_routine_E(agents, dt)
        apply_routine_D(agents, dt)
        all_advance(agents, dt)

    gui.set_image(env_map._pixel)
    render_agents(gui, env_map, agents, show_facing=True, show_visible_point=False, show_path=False)
    gui.show(f'frame_{frame:05d}.png')