"""
RRT_2D
@author: huiming zhou

Modified by David Filliat
"""

import os
import sys
import math
import numpy as np
import plotting, utils
import env
import time
import pandas as pd 
from utils import *
from tqdm import tqdm
from time import sleep

# parameters

showAnimation = False

class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None


class Rrt:
    def __init__(self, environment, s_start, s_goal, step_len, goal_sample_rate, iter_max):
        self.s_start = Node(s_start)
        self.s_goal = Node(s_goal)
        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate
        self.iter_max = iter_max
        self.vertex = [self.s_start]

        self.env = environment
        if showAnimation:
            self.plotting = plotting.Plotting(self.env, s_start, s_goal)
        self.utils = utils.Utils(self.env)

        self.x_range = self.env.x_range
        self.y_range = self.env.y_range
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
        self.obs_boundary = self.env.obs_boundary

    def planning(self):
        iter_goal = None
        for i in range(self.iter_max):
            node_rand = self.generate_random_node(self.goal_sample_rate)
            node_near = self.nearest_neighbor(self.vertex, node_rand)
            node_new = self.new_state(node_near, node_rand)

            if node_new and not self.utils.is_collision(node_near, node_new):
                self.vertex.append(node_new)
                dist, _ = self.get_distance_and_angle(node_new, self.s_goal)

                if dist <= self.step_len and iter_goal == None and not self.utils.is_collision(node_new, self.s_goal):
                    node_new = self.new_state(node_new, self.s_goal)
                    node_goal = node_new
                    iter_goal = i

        if iter_goal == None:
            return None, self.iter_max
        else:
            return self.extract_path(node_goal), iter_goal

    def generate_random_node(self, goal_sample_rate):
        if np.random.random() < goal_sample_rate:
            return self.s_goal

        delta = self.utils.delta

        return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                    np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))


    @staticmethod
    def nearest_neighbor(node_list, n):
        return node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)
                                        for nd in node_list]))]

    def new_state(self, node_start, node_end):
        dist, theta = self.get_distance_and_angle(node_start, node_end)

        dist = min(self.step_len, dist)
        node_new = Node((node_start.x + dist * math.cos(theta),
                         node_start.y + dist * math.sin(theta)))
        node_new.parent = node_start

        return node_new

    def extract_path(self, node_end):
        path = [(self.s_goal.x, self.s_goal.y)]
        node_now = node_end

        while node_now.parent is not None:
            node_now = node_now.parent
            path.append((node_now.x, node_now.y))

        return path

    @staticmethod
    def get_distance_and_angle(node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)


def get_path_length(path):
    """
    Compute path length
    """
    length = 0
    for i,k in zip(path[0::], path[1::]):
        length += np.linalg.norm(np.array(i) - np.array(k)) # math.dist(i,k)
    return length

def current_milli_time():
    return round(time.time() * 1000)

def add_data(data_dic,path,nb_iter,t_end):
    path_length=[]
    
    for i in range(len(path)):
        if path[i]:
            path_length.append(get_path_length(path[i]))
            
    if len(path_length)>0:
        data_dic["Time_computation"].append(np.mean(t_end))
        data_dic["path_index"].append(int(np.min(nb_iter)))
        data_dic["path_length"].append(np.mean(np.mean(path_length)))
    else:
        data_dic["Time_computation"].append(np.mean(t_end))
        data_dic["path_index"].append(int(np.min(nb_iter)))
        data_dic["path_length"].append("No Path Found")
    return data_dic
    
 
def firstNonNanPATH(listfloats):
  for i in range(len(listfloats)):
    if listfloats[i]:
        print(i)
        
        
def main():
    x_start=(2, 2)  # Starting node
    x_goal=(49, 24)  # Goal node
    environment = env.Env2()
    t0=current_milli_time()
    rrt = Rrt(environment, x_start, x_goal, 2, 0.10, 1000)
    path, nb_iter = rrt.planning() # path and iteration
    
    # Compute and save data
    t_end=current_milli_time()-t0 # time
    
    if path:
        print("Time computation (ms) is:", t_end)
        print('Found path in ' + str(nb_iter) + ' iterations, length : ' + str(get_path_length(path)))
        if showAnimation:
            rrt.plotting.animation(rrt.vertex, path, "RRT", True)
            plotting.plt.show()
    else:   
        print("Time computation (ms) is:", t_end)
        print("No Path Found in " + str(nb_iter) + " iterations!")
        if showAnimation:
            rrt.plotting.animation(rrt.vertex, [], "RRT", True)
            plotting.plt.show()


data_dic = { "max_iter": [100,200,500,1000,1500,2000,2500,3000,3500, 4000,4500,5000,5500,6000],
	"Time_computation": [],
	"path_index": [],
	"path_length": []} 
    
def run(data_dic):
    x_start=(2, 2)  # Starting node
    x_goal=(49, 24)  # Goal node
    environment = env.Env()
    for i in tqdm(data_dic["max_iter"]):
    
        t_end_list=[]
        nb_iter_list=[]
        path_list=[]
        
        for j in tqdm(range(50)): # avoid stochastic character 
            t0=current_milli_time()
            rrt = Rrt(environment, x_start, x_goal, 2, 0.10, i)
            path, nb_iter = rrt.planning() # path and iteration
            
            # Compute and save data
            t_end=current_milli_time()-t0 # time
    
            t_end_list.append(t_end)
            nb_iter_list.append(nb_iter)
            path_list.append(path)
                
            showAnimation=False
            
            # if path:
            #     #print("Time computation (ms) is:", t_end)
            #     print('Found path in ' + str(nb_iter) + ' iterations, length : ' + str(get_path_length(path)))
            #     if showAnimation:
            #         rrt.plotting.animation(rrt.vertex, path, "RRT", True)
            #         plotting.plt.show()
            # else:   
            #     print("Time computation (ms) is:", t_end)
            #     print("No Path Found in " + str(nb_iter) + " iterations!")
            #     if showAnimation:
            #         rrt.plotting.animation(rrt.vertex, [], "RRT", True)
            #         plotting.plt.show()
            
            
        data_dic= add_data(data_dic,path_list,nb_iter_list,t_end_list)
        
    return data_dic
        

  
#   data_dic = { "step_len": [0.1,0.2,1,2,5,10,20,40,50,80,100],
# 	"Time_computation": [],
# 	"path_index": [],
# 	"path_length": []}    
     
if __name__ == '__main__':
    #main()
    data_dic=run(data_dic)
    dataFrame=pd.DataFrame.from_dict(data_dic)
    dataFrame.to_pickle("result_rrt_variation_iteMAX-100-50000.pkl")
    #data=pd.read_pickle("result_rrt.pkl")
