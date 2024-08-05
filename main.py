from robot import Robot, SLAM_Robot
import matplotlib.pyplot as plt
from scipy.stats import multivariate_normal
import math
from collections import deque
import numpy as np

class God:
    def __init__(self, robot, true_map):
        self.robot = robot
        self.true_map = true_map
    
    def run(self):
        done = False
        is_scan = 1
        s_num = 0
        true_pose = self.robot.pos  #初始位置就是准确的
        plt.ion()
        fig, ax = plt.subplots()
        im = ax.imshow(self.robot.my_map, cmap='gray', origin='lower')
        current_pos_plot, = ax.plot([], [], 'ro')  # 'ro' means red color, circle marker
        while not done and s_num < 100:
            if is_scan:
                wall_points = self.robot.scan.scan(true_pose, self.true_map, self.robot.pos)
                self.robot.update_map(wall_points)
            done = self.robot.move()
            if done:
                decision = done[0]
                if done[1] <= 5:
                    is_scan = 1
                else:
                    is_scan = 0
                angle = decision[1]
                dist = decision[0]
                true_pose = [true_pose[0] + np.cos(angle)*dist, true_pose[1] + np.sin(angle)*dist, true_pose[2]+angle]
                done = False
            else:
                done = True
            current_pos_plot.set_data(self.robot.pos[1], self.robot.pos[0])  # Set current position to red
            im.set_data(self.robot.my_map)
            plt.draw()
            plt.pause(0.01)
        my_map = self.robot.my_map
        my_map[my_map>0.01] = 1
        my_map[my_map<=0.01] = -1
        im.set_data(my_map)
        plt.draw()
        plt.pause(0.01)
        plt.ioff()
        plt.show() 
        
def make_map():
    mask = np.zeros((50, 50))
    mask[5:45, 5:45] = 1
    mask[10:15, 10:15] = 0
    mask[30:45, 30:45] = 0
    mask[10:15, 40:45] = 0
    
    true_map = mask
    return true_map

def init_position(true_map):
    indices = np.argwhere(true_map == 1)
    random_index = indices[np.random.choice(indices.shape[0])]
    random_index = np.array([random_index[0], random_index[1], 0])
    return random_index

def main():
    true_map = make_map()
    lx, ly = true_map.shape
    init_pose = init_position(true_map)
    print(init_pose)
    my_map = np.full((lx, ly), -1, dtype=np.float32)
    my_map[init_pose[0]][init_pose[1]] = 1
    robot = SLAM_Robot(init_pose, my_map)
    god = God(robot, true_map)
    god.run()
    
if __name__ == "__main__":
    main()  