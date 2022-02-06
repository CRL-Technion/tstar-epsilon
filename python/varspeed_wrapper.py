from ctypes import *
import os
import numpy as np
import platform
import pickle

my_dir = os.path.split(os.path.abspath(__file__))[0]

if 'Linux' in platform.platform():
    # dll_path = os.path.join(os.path.split(my_dir)[0], r'c/bin/libVarSpeed.so')
    dll_path = os.path.join(os.path.split(my_dir)[0], r'c/bin/libVarSpeedWind.so')
else:
    dll_path = os.path.join(os.path.split(my_dir)[0], r'c\bin\libVarSpeed.dll')

    
varspeed_dll = CDLL(dll_path)

# VAR_SPEED_API void build_ocps_table(double v_min, double v_max, double u_max, double wind_x, double wind_y);
_build_ocps_table = getattr(varspeed_dll, "build_ocps_table")
_build_ocps_table.argtypes = [c_double, c_double, c_double, c_double, c_double]

class VarSpeedWrapper:
    def __init__(self, v_min, v_max, u_max, wind_x, wind_y):
        self.v_min = v_min
        self.v_max = v_max
        self.u_max = u_max
        self.wind_x = wind_x
        self.wind_y = wind_y
        
        _build_ocps_table(c_double(v_min), c_double(v_max), c_double(u_max), c_double(wind_x), c_double(wind_y))
        

if __name__ == "__main__":
    vsw = VarSpeedWrapper(v_min=0.5, v_max=1, u_max=1, wind_x=0, wind_y=0)