import numpy as np
import taichi as ti
import math
import random
from utils import *


class Object:
    def __init__(self, minx, miny, maxx, maxy, type):
        self.minx = minx
        self.miny = miny
        self.maxx = maxx
        self.maxy = maxy
        self.type = type


class QuadNode:
    def __init__(self, level, pos):
        self.level = level
        self.pos = pos
        self.conn = set() # connectvitiy with other quadtree nodes
    
    def actual_position(self):
        return ((self.pos[0] + 0.5) * (2**self.level) *  stationary_cell_size,
                (self.pos[1] + 0.5) * (2**self.level) *  stationary_cell_size)

@ti.data_oriented
class EnvMap:
    def __init__(self, boundary_size=(60.0, 30.0), objects=[]):
        self.boundary_size = boundary_size
        self.objects = objects
        self.gridmap_size = tuple(
            int(math.ceil(boundary_size[_] / stationary_cell_size)) for _ in range(2))
        self.gridmap = np.zeros(self.gridmap_size, dtype=np.int32)
        self.quadtree_level = 0
        self.quadtree_size = []
        self.quadtree_map = []
        self.quadtree_id = []
        self.quadtree_node = []
        self.quadtree_node_size = 0

        self.mobile_map_size = tuple(
            int(math.ceil(boundary_size[_] / mobile_cell_size)) for _ in range(2))
        self.mobile_map = np.zeros((self.mobile_map_size[0], self.mobile_map_size[1], 0)).tolist()

        self.init_gridmap()
        self.init_quadtree_map()

        self.init_visualizer()

    def update_mobile_map(self, agents):
        self.mobile_map = np.zeros((self.mobile_map_size[0], self.mobile_map_size[1], 0)).tolist()
        for agent in agents:
            cell_x = int(agent.current_pos[0] / mobile_cell_size)
            cell_y = int(agent.current_pos[1] / mobile_cell_size)
            self.mobile_map[cell_x][cell_y].append(agent)

    def init_gridmap(self):
        boundary_size = int(math.ceil(pedestrian_bouding_box_size / stationary_cell_size))
        for object in self.objects:
            cell_x0 = int(math.floor(
                (object.minx) / stationary_cell_size))
            cell_x1 = int(
                math.ceil((object.maxx) / stationary_cell_size))
            cell_y0 = int(math.floor(
                (object.miny) / stationary_cell_size))
            cell_y1 = int(
                math.ceil((object.maxy) / stationary_cell_size))
            for i in range(cell_x0-boundary_size, cell_x1+1+boundary_size):
                for j in range(cell_y0-boundary_size, cell_y1+1+boundary_size):
                    if cell_x0 <= i < cell_x1 and cell_y0 <= j < cell_y1:
                        self.gridmap[i, j] = object.type
                    elif 0 <= i < self.gridmap_size[0] and 0 <= j < self.gridmap_size[1] and self.gridmap[i, j] == EMPTY:
                        self.gridmap[i, j] = AIR_WALL

    def init_quadtree_map(self):
        level = 1
        self.quadtree_map.append(self.gridmap.copy())
        self.quadtree_size.append(self.gridmap_size)
        while True:
            exist_valid = False
            self.quadtree_size.append(
                tuple(self.quadtree_size[level-1][_] // 2 for _ in range(2)))
            self.quadtree_map.append(
                np.zeros(self.quadtree_size[level], dtype=np.int32))

            for i in range(self.quadtree_size[level][0]):
                for j in range(self.quadtree_size[level][1]):
                    value = 0
                    value += self.quadtree_map[level-1][i*2, j*2]
                    value += self.quadtree_map[level-1][i*2+1, j*2]
                    value += self.quadtree_map[level-1][i*2, j*2+1]
                    value += self.quadtree_map[level-1][i*2+1, j*2+1]

                    if value == 0:
                        self.quadtree_map[level][i, j] == VALID
                        self.quadtree_map[level-1][i*2, j*2] = INVALID
                        self.quadtree_map[level-1][i*2+1, j*2] = INVALID
                        self.quadtree_map[level-1][i*2, j*2+1] = INVALID
                        self.quadtree_map[level-1][i*2+1, j*2+1] = INVALID
                        exist_valid = True
                    else:
                        self.quadtree_map[level][i, j] = INVALID

            if not exist_valid:
                break
            level += 1

        self.quadtree_level = level
        print(f"quadtree_level={self.quadtree_level}")

        for l in range(self.quadtree_level):
            self.quadtree_id.append(np.zeros(self.quadtree_size[l], dtype=np.int32))
            for i in range(self.quadtree_size[l][0]):
                for j in range(self.quadtree_size[l][1]):
                    if self.quadtree_map[l][i, j] == VALID:
                        self.quadtree_id[l][i, j] = self.quadtree_node_size
                        self.quadtree_node.append(QuadNode(l, (i, j)))
                        self.quadtree_node_size += 1
            
        print(f"quadtree_node_size={self.quadtree_node_size}")
        for i in range(self.quadtree_node_size):
            q = self.quadtree_node[i]
            for dxy in DXY:
                self.search_quadtree_neighbor(i, q.level, q.pos, dxy)

    def search_quadtree_neighbor(self, nodeI, cur_level, cur_pos, dxy):
        tar_pos_x = cur_pos[0]+dxy[0]
        tar_pos_y = cur_pos[1]+dxy[1]
        if tar_pos_x >= 0 and tar_pos_x < self.quadtree_size[cur_level][0] and \
           tar_pos_y >= 0 and tar_pos_y < self.quadtree_size[cur_level][1] and \
           self.quadtree_map[cur_level][(tar_pos_x, tar_pos_y)] == VALID:
            nodeJ = self.quadtree_id[cur_level][(tar_pos_x, tar_pos_y)]
            self.quadtree_node[nodeI].conn.add(nodeJ)
            self.quadtree_node[nodeJ].conn.add(nodeI)
            return
        elif cur_level == 0:
            return
        else:
            cur_pos1 = (cur_pos[0]*2+(1 if dxy[0]==1 else 0),
                        cur_pos[1]*2+(1 if dxy[1]==1 else 0))
            if dxy[1] == 0:
                cur_pos2 = (cur_pos1[0], cur_pos1[1]+1)
                self.search_quadtree_neighbor(nodeI, cur_level-1, cur_pos1, dxy)
                self.search_quadtree_neighbor(nodeI, cur_level-1, cur_pos2, dxy)
            elif dxy[0] == 0:
                cur_pos2 = (cur_pos1[0]+1, cur_pos1[1])
                self.search_quadtree_neighbor(nodeI, cur_level-1, cur_pos1, dxy)
                self.search_quadtree_neighbor(nodeI, cur_level-1, cur_pos2, dxy)
            else:
                self.search_quadtree_neighbor(nodeI, cur_level-1, cur_pos1, dxy)
    
    def get_quadtree_node(self, pos):
        cell_x = int(pos[0] / stationary_cell_size)
        cell_y = int(pos[1] / stationary_cell_size)
        level = 0
        while level < self.quadtree_level:
            if self.quadtree_map[level][cell_x, cell_y] == VALID:
                return self.quadtree_id[level][cell_x, cell_y]
            level +=1
            cell_x //= 2
            cell_y //= 2
        assert(False)
    
    def is_empty_position(self, pos):
        cell_x = int(pos[0] / stationary_cell_size)
        cell_y = int(pos[1] / stationary_cell_size)
        if 0 <= cell_x < self.gridmap_size[0] and 0 <= cell_y < self.gridmap_size[1]:
            return self.gridmap[cell_x, cell_y] == EMPTY
        else:
            return True

    def get_valid_random_position(self):
        x, y = random.random() * self.boundary_size[0], random.random() * self.boundary_size[1]
        cell_x = int(x / stationary_cell_size)
        cell_y = int(y / stationary_cell_size)
        while self.gridmap[cell_x, cell_y] != EMPTY:
            x, y = random.random() * self.boundary_size[0], random.random() * self.boundary_size[1]
            cell_x = int(x / stationary_cell_size)
            cell_y = int(y / stationary_cell_size)
        return (x, y)

    # NOTE(changyu): visual part, for debugging
    def init_visualizer(self, test_node=-1):
        self._res = default_resolusion
        self._pixel = ti.Vector.field(3, ti.f32, shape=self._res)
        self._gridmap = ti.field(ti.i32, shape=self.gridmap_size)
        self._gridmap.from_numpy(self.gridmap)
        self._quadmap = []
        self._quadmap_id = []
        if test_node != -1:
            conn = list(self.quadtree_node[test_node].conn)
            self._conn = ti.field(ti.i32, shape=len(conn))
            self._conn.from_numpy(np.array(conn, dtype=np.int32))
        for l in range(self.quadtree_level):
            self._quadmap.append(ti.field(ti.i32, shape=self.quadtree_size[l]))
            self._quadmap_id.append(ti.field(ti.i32, shape=self.quadtree_size[l]))
            self._quadmap[l].from_numpy(self.quadtree_map[l])
            self._quadmap_id[l].from_numpy(self.quadtree_id[l])

        for l in range(self.quadtree_level):
            self.quadtree_visualizer(l, test_node)

    @ti.kernel
    def quadtree_visualizer(self, l: ti.template(), test_node : ti.template()):
        for I in ti.grouped(self._pixel):
            scale = self._res[0] // self.gridmap_size[0] * (2 ** (l))
            if all(I // scale < self.quadtree_size[l]) and \
                (self._quadmap[l][I // scale] == VALID or self._quadmap[l][I // scale] == AIR_WALL):
                flag = False
                if ti.static(test_node != -1):
                    for c in range(self._conn.shape[0]):
                        if self._conn[c] == self._quadmap_id[l][I // scale]:
                            flag = True
                if flag:
                    self._pixel[I] = ti.Vector([0.98, 0.2, 0.29])
                    if any(I % scale == 0) or any(I % scale == scale - 1):
                        self._pixel[I] = ti.Vector([0.78, 0.1, 0.19])
                elif self._quadmap_id[l][I // scale] == test_node:
                    self._pixel[I] = ti.Vector([0.98, 0.68, 0.29])
                    if any(I % scale == 0) or any(I % scale == scale - 1):
                        self._pixel[I] = ti.Vector([0.78, 0.58, 0.19])
                else:
                    self._pixel[I] = ti.Vector([0.28, 0.68, 0.99])
                    if any(I % scale == 0) or any(I % scale == scale - 1):
                        self._pixel[I] = ti.Vector([0.18, 0.58, 0.88])
