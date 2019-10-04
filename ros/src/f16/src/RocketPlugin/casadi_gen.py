import casadi as ca
import numpy as np
from pyecca.so3 import Mrp


def aero_prop():
    pos_ENU = ca.SX.sym('pos_ENU', 3)
    vel_ENU = ca.SX.sym('vel_ENU', 3)
    q_ENU_TRF = ca.SX.sym('q_ENU_TRF', 4)
    p = ca.SX.sym('p', 15)
    u = ca.SX.sym('u', 4)

    C_NED_ENU = np.array([
        [0, 1, 0],
        [1, 0, 0],
        [0, 0, -1]
    ])

    C_TRF_FRD = np.array([
        [0, 0, -1],
        [0, 1, 0],
        [1, 0, 0]
    ])

    so3.Mr



    F_aero = ca.vertcat(p[0], p[1], p[2])
    F_prop = ca.vertcat(x[3], x[4], x[5])

    f = ca.Function('double_this', [x, u, p], [F_aero, F_prop],
        ['x', 'u', 'p'], ['F_aero', 'F_prop'])

    gen = ca.CodeGenerator('casadi_gen.c', {'main': False, 'mex': False, 'with_header': True, 'with_mem': True})
    gen.add(f)
    gen.generate()


def gazebo_cleanup():


    f_gz = ca.Function('gazebo_cleanup', [pos_ENU, vel_ENU, q_ENU_TRF], [x, u, p],
        ['pos_ENU', 'vel_ENU', 'q_ENU_TRF'])

aero_prop()
gazebo_cleanup()
