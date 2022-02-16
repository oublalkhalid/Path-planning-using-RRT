#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 15 11:34:12 2022

@author: oublal
"""

import pandas as pd
import matplotlib.pyplot as plt

rrt = pd.read_pickle('result_rrt_variation_step_len_0p1-100.pkl')
rrtStar=pd.read_pickle('result_rrtSTAR_step_len-0p1-100.pkl')


rrt1 = pd.read_pickle('result_rrt_variation_iteMAX-100-50000.pkl')
rrtStar1=pd.read_pickle('result_rrtSTAR_variation_iteMAX-100-50000.pkl')


import numpy as np; 
np.random.seed(0)
import seaborn as sns; 
sns.set_theme()


ax = sns.heatmap(rrt1.drop(columns=['path_length', 'max_iter','path_index']),
                 yticklabels=rrt1["max_iter"],cbar_kws={'label': 'Time_computation'})
#ax.set_yticks(rrt1["max_iter"])
plt.ylabel("max_iter")
plt.title("RRT")
plt.show()

print(rrt1.to_latex(index=False))
print(rrtStar1.to_latex(index=False))

#print((rrt.drop(columns='path_index')).to_latex(index=False)) 
#print((rrtStar.drop(columns='path_index')).to_latex(index=False)) 


data_0p1=pd.read_pickle('data/RRT_Env2_50_cycle_newFunction_0p1percent.pkl')
data_0=pd.read_pickle('data/RRT_Env2_50_cycle_newFunction_0percent.pkl')
data_10=pd.read_pickle('data/RRT_Env2_50_cycle_newFunction_10percent.pkl')

data_20=pd.read_pickle('data/RRT_Env2_50_cycle_newFunction_20percent.pkl')
data_30=pd.read_pickle('data/RRT_Env2_50_cycle_newFunction_30percent.pkl')

data_40=pd.read_pickle('data/RRT_Env2_50_cycle_newFunction_40percent.pkl')
data_50=pd.read_pickle('data/RRT_Env2_50_cycle_newFunction_50percent.pkl')

data_60=pd.read_pickle('data/RRT_Env2_50_cycle_newFunction_60percent.pkl')
data_70=pd.read_pickle('data/RRT_Env2_50_cycle_newFunction_70percent.pkl')

data_80=pd.read_pickle('data/RRT_Env2_50_cycle_newFunction_80percent.pkl')
data_90=pd.read_pickle('data/RRT_Env2_50_cycle_newFunction_90percent.pkl')

data_100=pd.read_pickle('data/RRT_Env2_50_cycle_newFunction_100percent.pkl')
