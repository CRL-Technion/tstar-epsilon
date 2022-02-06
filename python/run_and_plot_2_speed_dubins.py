#!/usr/bin/env python

import sys
from MapEnvironment import MapEnvironment
from tstar_wrapper import TstarWrapper
import toml
from IPython import embed
import numpy as np
import time

def g_func(name):
    if name == 'dubins':
        return 1;
    else:
        print("ERROR! unknown cost function! use [dubins, euclidean]")
        return -1

def h_func(name):
    if name == 'dubins':
        return 1;
    else:
        print("ERROR! unknown heuristic function! use [dubins, euclidean]")
        return -1
    
def main(planning_env, options):
    start_time = time.time()
    c_cost = g_func(options['g_func'])
    c_heuristic = h_func(options['h_func'])
    
    if -1 in [c_cost,c_heuristic]:
        sys.exit(0)
    
    # init tstar planner.
    num_of_speeds = 2
    for speed in range(num_of_speeds):
        options['speed'] = speed
        tstar = TstarWrapper(planning_env.map, planning_env.start, planning_env.goal, c_cost, c_heuristic, options)

        plan_short = tstar.run()
        if speed == 0:
            low_plan = tstar.get_mid_points(plan_short)
        else:
            high_plan = tstar.get_mid_points(plan_short)
        # plan_short = tstar.get_mid_points(plan_short)
    # Visualize the final path.
    plan_short = np.concatenate((low_plan, high_plan))
    plot_both = True
    planning_env.visualize_plan(plan_short, plot_both)
    
    #embed()


if __name__ == "__main__":
    
    options = toml.load('config.toml')
 
    # First setup the environment and the robot.
    planning_env = MapEnvironment(options['map'], options['start'], options['goal'])

    main(planning_env, options) 
