from ctypes import *
import os
import numpy as np
import platform
import varspeed_wrapper

my_dir = os.path.split(os.path.abspath(__file__))[0]

if 'Linux' in platform.platform():
    dll_path = os.path.join(os.path.split(my_dir)[0], r'c/bin/libtstar.so')
else:
    dll_path = os.path.join(os.path.split(my_dir)[0], r'c\bin\tstar.dll')

class State(Structure):
    _fields_ = [('row', c_int),
                ('col', c_int),
                ('theta', c_int),
                ('v', c_int)]

class Problem(Structure):
    _fields_ = [('uMax',c_double),
                ('vMin',c_double),
                ('vMax',c_double),
                ('speed',c_int),
                ('obstacle_clearance',c_double),
                ('h', c_int),
                ('g', c_int),
                ('epsilon',c_double),
                ('wind_x',c_double),
                ('wind_y',c_double),
                ('alg', c_int)]
    
tstar_dll = CDLL(dll_path)

# void* tstar_create(int* map, int width, int height, State start, State goal, Problem problem);
_tstar_create = getattr(tstar_dll, "tstar_create")
_tstar_create.argtypes = [c_void_p, c_int, c_int, State, State, Problem]
_tstar_create.restype = c_void_p

# int tstar_run(void* handler, double* solution, int* solution_length);
_tstar_run = getattr(tstar_dll, "tstar_run")
_tstar_run.argtypes = [c_void_p, c_void_p, POINTER(c_int)]
_tstar_run.restype = c_int

# int tstar_get_segment_n_points(void* handler, double* path);
_tstar_get_segment_n_points = getattr(tstar_dll, "tstar_get_segment_n_points")
_tstar_get_segment_n_points.argtypes = [c_void_p, POINTER(c_double)]
_tstar_get_segment_n_points.restype = c_int

# void tstar_get_segment_points(void* handler, State start, double* path, int n_points, double* points_out);
_tstar_get_segment_points = getattr(tstar_dll, "tstar_get_segment_points")
_tstar_get_segment_points.argtypes = [c_void_p, State, POINTER(c_double), c_int, c_void_p]


class TstarWrapper:
    def __init__(self, map_env, start, goal, cost, heuristic, algorithm,  options):
        self.map_env = map_env
        
        _problem = Problem()
        _problem.h = heuristic
        _problem.g = cost
        _problem.alg = algorithm
        _problem.uMax = options['uMax']
        _problem.vMax = options['vMax']
        _problem.vMin = options['vMin']
        _problem.speed = options['speed']
        _problem.obstacle_clearance = options['obstacle_clearance']
        _problem.epsilon = options['epsilon']
        _problem.wind_x = options['wind_x']
        _problem.wind_y = options['wind_y']

        # print(f"epsilon in tstar_wrapper.py = {_problem.epsilon}")
        print(f'Wind in tstar_wrapper.py = ({_problem.wind_x}, {_problem.wind_y})')

        if not os.path.isfile(options['ocps_table']) and _problem.g == 3 and _problem.alg == 1:  #build only on T* with varspeed
            self.vsw = varspeed_wrapper.VarSpeedWrapper(v_min=options['vMin'], v_max=options['vMax'], u_max=options['uMax'], wind_x=options['wind_x'], wind_y=options['wind_y'])
                
        _width = map_env.shape[1]
        _height = map_env.shape[0]
        _map = ((c_int * _width) * _height)()
        for i in range(_height):
            for j in range(_width):
                _map[i][j]= int(map_env[i][j])
        
        _start = State()
        _start.row = start[0]
        _start.col = start[1]
        _start.theta = start[2]
        if _problem.g == 3: # varspeed
            _start.v = start[3]
        elif _problem.g == 1: # dubins
            _start.v = options['speed']
            
        _goal = State()
        _goal.row = goal[0]
        _goal.col = goal[1]
        _goal.theta = goal[2]
        if _problem.g == 3: # varspeed
            _goal.v = goal[3]
        elif _problem.g == 1: # dubins
            _goal.v = options['speed']
    
        
        self._tstar = _tstar_create(cast(_map,c_void_p), _width, _height, _start, _goal, _problem)
    
    
    def run(self):
        _width = self.map_env.shape[1]
        _height = self.map_env.shape[0]
        _sol = ((c_double * 8) * (_width *_height))()
        _sol_length = c_int()
        res = _tstar_run(self._tstar, _sol, _sol_length)
        path = []
        if res:
            for i in range(_sol_length.value):
                path.append((_sol[i][0],_sol[i][1],_sol[i][2],_sol[i][3],_sol[i][4],_sol[i][5],_sol[i][6],_sol[i][7]))
        return np.array(path)        
    
    def get_mid_points(self, path):
        n_points = path.shape[0]
        parent_state = State()
        path_extended = []
        for i in range(n_points - 1):
            step = path[i]
            step_parent = path[i+1]
            parent_state.row = int(step_parent[0])
            parent_state.col = int(step_parent[1])
            parent_state.theta = int(step_parent[2])
            parent_state.v = int(step_parent[3])
            c_step = (c_double * 4)()
            
            for j in range(4):
                c_step[j] = step[j+4]
            c_segment_n_points = c_int(_tstar_get_segment_n_points(self._tstar, c_step))
            c_segment_points = ((c_double * 4) * (c_segment_n_points.value))()
            _tstar_get_segment_points(self._tstar, parent_state, c_step, c_segment_n_points, cast(c_segment_points,c_void_p))
            
            path_extended.append((step.tolist()[:4]))
            for j in reversed(range(c_segment_n_points.value)):
                path_extended.append([c_segment_points[j][0],c_segment_points[j][1],
                                      c_segment_points[j][2],c_segment_points[j][3]])
        path_extended.append((path[n_points - 1].tolist()[:4]))
        
        return np.array(path_extended) 