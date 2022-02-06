#!/usr/bin/env python

import sys
from MapEnvironment import MapEnvironment
from tstar_wrapper import TstarWrapper
import toml
from IPython import embed
import time

def g_func(name):
    if name == 'dubins':
        return 1;
    elif name == "euclidean":
        return 2;
    elif name == "varspeed":
        return 3;
    else:
        print("ERROR! unknown cost function! use [dubins, euclidean]")
        return -1

def h_func(name):
    if name == 'dubins':
        return 1;
    elif name == "euclidean":
        return 2;
    else:
        print("ERROR! unknown heuristic function! use [dubins, euclidean]")
        return -1

def algorithm(name):
    if name == 'tstar':
        return 1
    elif name == 'tstar-epsilon':
        return 2
    else:
        print("ERROR! unknown algorithm! use [tstar, tstar-epsilon]")
        return -1
    
def main(planning_env, options):
    start_time = time.time()
    c_cost = g_func(options['g_func'])
    c_heuristic = h_func(options['h_func'])
    c_algorithm = algorithm(options['algorithm'])


    if -1 in [c_cost, c_heuristic, c_algorithm]:
        sys.exit(0)
    
    # init tstar planner.
    # print(f"epsilon value in python = {options['epsilon']}")
    # print(f"Wind value in python = ({options['wind_x']}, {options['wind_y']})")
    tstar = TstarWrapper(planning_env.map, planning_env.start, planning_env.goal, c_cost, c_heuristic, c_algorithm, options)
    ocps_time = time.time()

    plan_short = tstar.run()
    plan_time = time.time()

    # Edit
    cells_path = plan_short
    # for item in cells_path:
    #     print(f'x = {item[0]}')

    wind_vec = [options['wind_x'], options['wind_y']]

    plan_short = tstar.get_mid_points(plan_short)
    # Visualize the final path.
    planning_env.visualize_plan(plan_short, wind_vec, cells_path)
    visualize_time = time.time()
    print("ocps build time: ", ocps_time - start_time)
    print("planing time: ", plan_time - ocps_time)
    print("plot time:", visualize_time - plan_time)
    
    #embed()


if __name__ == "__main__":
    
    options = toml.load('config.toml')
 
    # First setup the environment and the robot.
    planning_env = MapEnvironment(options['map'], options['start'], options['goal'])

    main(planning_env, options) 
