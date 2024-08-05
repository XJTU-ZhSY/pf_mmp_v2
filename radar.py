import numpy as np
import math

class Scan:
    def __init__(self):
        self.max_dist = 200
    
    def scan(self, true_pose, true_map, selfpos):
        angles = np.arange(0, 360, 10)
        np.random.shuffle(angles)
        sampled_points = []     
        for angle in angles:
            radians = np.deg2rad(angle) + selfpos[2]
            signal = self.detect(true_pose, radians, true_map)  #激光雷达扫描是根据真实情况
            if signal:
                signal = signal + np.random.normal(0, signal/1000)    #激光雷达有噪声
                sampled_points.append([signal, radians])    #机器人认为自己检测到的
        return sampled_points
    
    def detect(self, true_pose, rad, true_map):
        lx = true_map.shape[0]
        ly = true_map.shape[1]
        x = true_pose[0]
        y = true_pose[1]
        pose = (x, y)
        dy = np.sin(rad)
        dx = np.cos(rad)
        x = x+dx
        y = y+dy
        while 1:
            if dx < 0:
                px = np.floor(x)
            else:
                px = np.ceil(x)
            if dy < 0:
                py = np.floor(y)
            else:
                py = np.ceil(y)
            px = int(px)
            py = int(py)
            if px >= lx or py >= ly or px < 0 or py < 0:
                return None
            if true_map[px][py] == 0:
                return self.dist(pose, (x, y))
            x = x+dx
            y = y+dy
        
    def dist(self, p1, p2):
        x1, y1 = p1
        x2, y2 = p2
        dis = math.sqrt((x1-x2)**2+(y1-y2)**2)   
        if dis < self.max_dist:
            return dis
        return None