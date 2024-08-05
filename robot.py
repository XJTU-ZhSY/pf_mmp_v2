import numpy as np
from radar import Scan
from particle import Particle
from collections import deque
import math

class Robot:
    def __init__(self):
        pass

class SLAM_Robot(Robot):
    def __init__(self, pos, my_map):
        super().__init__()
        self.pos = pos
        self.my_map = my_map
        self.particles = [Particle(self.pos,1)]
        self.has_decision = []
        self.p_num = 50
        self.max_dist = 100
        self.scan = Scan()
        self.wall_num = 0
        self.unknown_num = -1
    
    def move(self):
        if not self.has_decision:
            if not self.judge():
                print("done")
                return None
        decision = self.has_decision.pop(0)
        angle = decision[1]
        dist = decision[0]
        new_particle = []
        for p in self.particles:
            new_particle += self.generate_particles(p, decision)
        self.particles = new_particle
        self.resampling()
        x = y = a = s = 0
        for p in self.particles:
            x += p.pose[0]*p.weight
            y += p.pose[1]*p.weight
            a += p.pose[2]*p.weight
            s += p.weight
        if s:
            x /= s
            y /= s
            a /= s
        self.pos = [x, y, a]
        if np.cos(angle) >= 0:
            px = int(np.ceil(x))
        if np.sin(angle) >= 0:
            py = int(np.ceil(y))
        if np.cos(angle) < 0:
            px = int(np.floor(x))
        if np.sin(angle) < 0:
            py = int(np.floor(y))    
        self.my_map[px][py] = 1
        decision = [dist+np.random.normal(0, dist/10), angle+np.random.normal(0, 0.05)]  #真实的移动有误差
        return [decision, len(decision)]

    def judge(self):
        point = None
        lx, ly = self.my_map.shape
        xs, ys = [np.ceil(self.pos[0]), np.floor(self.pos[0])], [np.ceil(self.pos[1]), np.floor(self.pos[1])]
        flag = 0
        for i in xs:
            for j in ys:
                i = int(i)
                j = int(j)
                if 0 < i < lx and 0 < j < ly and self.my_map[i][j] != self.wall_num:
                    self.my_map[i][j] = 1
                    point = (i, j)
                    flag = 1
                    break
            if flag:
                break
        if point:
            self.has_decision = self.search_way(point)
        if self.has_decision:
            return True
        return None
    
    def search_way(self, point):
        rows, cols = len(self.my_map), len(self.my_map[0])
        directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        queue = deque([(point[0], point[1], [])])  # 队列元素：当前坐标和到达此处的路径
        visited = set([point])  # 记录访问过的节点
        while queue:
            x, y, path = queue.popleft()
            # 检查当前位置是否是未知区域
            if self.my_map[x][y] == self.unknown_num:
                path = path + [(x, y)]
                path = self.path_to_angles_distances(path)
                return path
            for dx, dy in directions:
                nx, ny = x + dx, y + dy
                if 0 <= nx < rows and 0 <= ny < cols and (nx, ny) not in visited and self.my_map[nx][ny] != self.wall_num:
                    queue.append((nx, ny, path + [(x, y)]))
                    visited.add((nx, ny))
        return None  # 如果找不到路径，返回None

    def path_to_angles_distances(self, path):
        angles_distances = []
        for i in range(1, len(path)):
            x1, y1 = path[i-1]
            x2, y2 = path[i]
            dx, dy = x2 - x1, y2 - y1
            # 计算角度，使得正X方向为0度，用弧度表示
            angle = math.atan2(dy, dx)
            # 计算距离
            distance = math.sqrt(dx**2 + dy**2)
            angles_distances.append((distance, angle))
        return angles_distances
                      
    def update_map(self, wall_points):
        lx, ly = self.my_map.shape
        for particle in self.particles:
            x0, y0 = particle.pose[0], particle.pose[1]
            for signal in wall_points:
                x = x0 + signal[0] * np.cos(signal[1])
                y = y0 + signal[0] * np.sin(signal[1])
                if np.cos(signal[1]) >= 0:
                    px = int(np.ceil(x))
                if np.sin(signal[1]) >= 0:
                    py = int(np.ceil(y))
                if np.cos(signal[1]) < 0:
                    px = int(np.floor(x))
                if np.sin(signal[1]) < 0:
                    py = int(np.floor(y))
                if 0 <= px < lx and 0 <= py < ly:
                    if signal[0] < 10:
                        if self.my_map[px][py] == -1 or self.my_map[px][py] == 0: 
                            self.my_map[px][py] = self.wall_num
                    else:
                        self.my_map[px][py] -= 0.5
                        if self.my_map[px][py] < 0:
                            self.my_map[px][py] = self.wall_num
        
        px, py = self.pos[0], self.pos[1]
        for signal in wall_points:
            if signal[0] < 10:
                x = x0 + signal[0] * np.cos(signal[1])
                y = y0 + signal[0] * np.sin(signal[1])
                dx = np.cos(signal[1])
                dy = np.sin(signal[1])
                fx = fy = 0
                if dx:
                    fx = dx/abs(dx)
                if dy:
                    fy = dy/abs(dy)
                flag = 1
                while flag:
                    if px*fx < x*fx or py*fy < y*fy:
                        break
                    if dx >= 0:
                        px = int(np.ceil(x))
                    if dy >= 0:
                        py = int(np.ceil(y))
                    if dx < 0:
                        px = int(np.floor(x))
                    if dy < 0:
                        py = int(np.floor(y))
                    if self.my_map[px][py] == self.wall_num:
                        self.my_map[px][py] = 0.01
                    elif self.my_map[px][py] == self.unknown_num:
                        self.my_map[px][py] = 0.5
                    px += dx
                    py += dy
    
    def generate_particles(self, p_father, decision):
        lx, ly = self.my_map.shape
        angle = decision[1]
        distance = decision[0]+np.random.normal(0, 1/50)
        particles = []
        fpose = p_father.pose
        dx = distance * np.cos(angle)
        dy = distance * np.sin(angle)
        pose = np.array(fpose) + np.array([dx, dy, 0])
        sigma = np.array([[1.0, 0], [0, 1.0]])
        for i in range(self.p_num):
            noise = np.random.multivariate_normal([0, 0], sigma)
            px = pose[0] + noise[0]
            py = pose[1] + noise[1]
            p = [px, py, angle]
            if not (0 < p[0] < lx and 0 < p[1] < ly and self.my_map[int(p[0])][int(p[1])] != self.wall_num):    #墙的num
                continue
            diff = pose[:2] - p[:2]
            w = np.exp(-0.5 * np.dot(diff, np.dot(np.linalg.inv(sigma), diff)))
            if self.my_map[int(p[0])][int(p[1])] == -1 or self.my_map[int(p[0])][int(p[1])] == 0:
                self.my_map[int(p[0])][int(p[1])] = w
            else:
                self.my_map[int(p[0])][int(p[1])] += w 
            particle = Particle(p, w)
            particle.tradj += p_father.tradj
            particle.tradj.append(p_father.pose)
            particles.append(particle)
        return particles
    
    def resampling(self):
        self.particles.sort(key=lambda p: p.weight, reverse=True)
        selected_particles = self.particles[:self.p_num]
        total_weight = sum(p.weight for p in selected_particles)
        weights = np.array([p.weight / total_weight for p in selected_particles])
        indices = np.random.choice(len(selected_particles), size=self.p_num, p=weights)
        new_particles = [selected_particles[i] for i in indices]
        self.particles = new_particles
