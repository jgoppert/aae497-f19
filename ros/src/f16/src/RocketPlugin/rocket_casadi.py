import casadi as ca
import numpy as np
import sys

sys.path.insert(0, '../../../../../python/pyecca')

import matplotlib.pyplot as plt
import pyecca.lie.so3 as so3
from pyecca.util import rk4


def rocket_equations(jit=True):
    x = ca.SX.sym('x', 14)
    u = ca.SX.sym('u', 4)
    p = ca.SX.sym('p', 16)

    # State: x
    omega_b = x[0:3]  # inertial angular velocity expressed in body frame
    r_nb = x[3:7]  # modified rodrigues parameters
    v_b = x[7:10]  # inertial velocity expressed in body components
    p_n = x[10:13]  # positon in nav frame
    m_fuel = x[13]  # mass
    
    # Input: u
    m_dot = ca.if_else(m_fuel > 0, u[0], 0)
    aileron = u[1]
    elevator = u[2]
    rudder = u[3]
    
    # Parameters: p
    g = p[0]  # gravity
    Jx = p[1]  # moment of inertia
    Jy = p[2]
    Jz = p[3]
    Jxz = p[4]
    ve = p[5]
    l_fin = p[6]
    w_fin = p[7]
    CL_alpha = p[8]
    CL0 = p[9]
    CD0 = p[10]
    K = p[11]
    s_fin = p[12]
    rho = p[13]
    m_empty = p[14]
    l_motor = p[15]
    
    # Calculations
    m = m_empty + m_fuel
    J_b = ca.SX.zeros(3, 3)
    J_b[0, 0] = Jx + m_fuel*l_motor**2
    J_b[1, 1] = Jy + m_fuel*l_motor**2
    J_b[2, 2] = Jz
    J_b[0, 2] = J_b[2, 0] = Jxz

    C_nb = so3.Dcm.from_mrp(r_nb)
    g_n = ca.vertcat(0, 0, g)
    v_n = ca.mtimes(C_nb, v_b)
    
    # aerodynamics
    VT = ca.norm_2(v_b)
    q = 0.5*rho*ca.dot(v_b, v_b)
    fins = {
        'top': {
            'fwd': [1, 0, 0],
            'up': [0, 1, 0],
            'mix': aileron + rudder,
        },
        'left': {
            'fwd': [1, 0, 0],
            'up': [0, 0, -1],
            'mix': aileron + elevator,
        },
        'down': {
            'fwd': [1, 0, 0],
            'up': [0, -1, 0],
            'mix': aileron - rudder,
        },
        'right': {
            'fwd': [1, 0, 0],
            'up': [0, 0, 1],
            'mix': aileron - elevator,
        },
    }
    rel_wind_dir = v_b/VT

    # build fin lift/drag forces
    vel_tol = 1e-3
    FA_b = ca.vertcat(0, 0, 0)
    MA_b = ca.vertcat(0, 0, 0)
    for key, data in fins.items():
        fwd = data['fwd']
        up = data['up']
        mix = data['mix']
        U = ca.dot(fwd, v_b)
        W = ca.dot(up, v_b)
        side = ca.cross(fwd, up)
        alpha = ca.if_else(ca.fabs(U) > vel_tol, -ca.atan(W/U), 0)
        perp_wind_dir = ca.cross(side, rel_wind_dir)
        norm_perp = ca.norm_2(perp_wind_dir)
        perp_wind_dir = ca.if_else(ca.fabs(norm_perp) > vel_tol, 
            perp_wind_dir/norm_perp, up)
        CL = CL0 + CL_alpha*(alpha + mix)
        # model stall
        CL = ca.if_else(ca.fabs(alpha) < 0.4, CL, 0)
        CD = CD0 + K*(CL - CL0)**2
        L = CL*q*s_fin
        D = CD*q*s_fin
        FAi_b = L*perp_wind_dir - D*rel_wind_dir
        FA_b += FAi_b
        MA_b += ca.cross(-l_fin*fwd + w_fin*side, FAi_b)

    FA_b = ca.if_else(ca.fabs(VT) > vel_tol, FA_b, ca.SX.zeros(3))
    MA_b = ca.if_else(ca.fabs(VT) > vel_tol, MA_b, ca.SX.zeros(3))

    # propulsion
    FP_b = ca.vertcat(m_dot*ve, 0, 0)
    
    # force and momental total
    F_b = FA_b + FP_b + ca.mtimes(C_nb.T, m*g_n)
    M_b = MA_b

    force_moment = ca.Function(
        'force_moment', [x, u, p], [F_b, M_b], ['x', 'u', 'p'], ['F_b', 'M_b'])
    
    
    # right hand side
    rhs = ca.Function('rhs', [x, u, p], [ca.vertcat(
        ca.mtimes(ca.inv(J_b), M_b - ca.cross(omega_b, ca.mtimes(J_b, omega_b))),
        so3.Mrp.kinematics(r_nb, omega_b),
        F_b/m - ca.cross(omega_b, v_b),
        ca.mtimes(C_nb, v_b), -m_dot)], ['x', 'u', 'p'], ['rhs'], {'jit': jit})

    # prediction
    t0 = ca.SX.sym('t0')
    h = ca.SX.sym('h')
    x0 = ca.SX.sym('x', 14)
    x1 = rk4(lambda t, x: rhs(x, u, p), t0, x0, h)
    x1[3:7] = so3.Mrp.shadow_if_necessary(x1[3:7])
    predict = ca.Function('predict', [x0, u, p, t0, h], [x1], {'jit': jit})

    # initialize
    pitch_deg = ca.SX.sym('pitch_deg')
    omega0_b = ca.vertcat(0, 0, 0)
    r0_nb = so3.Mrp.from_euler(ca.vertcat(0, pitch_deg*ca.pi/180, 0))
    v0_b = ca.vertcat(0, 0, 0)
    p0_n = ca.vertcat(0, 0, 0)
    m0_fuel = 0.2
    # x: omega_b, r_nb, v_b, p_n, m_fuel
    x0 = ca.vertcat(omega0_b, r0_nb, v0_b, p0_n, m0_fuel)
    #     g, Jx, Jy, Jz, Jxz, ve, l_fin, w_fin, CL_alpha, CL0, CD0, K, s, rho, m_emptpy, l_motor
    p0 = [9.8, 1.0, 1.0, 1.0, 0.0, 350, 1.0, 0.05, 2*np.pi, 0, 0.01, 0.01, 0.05, 1.225, 0.2, 1.0]
    initialize = ca.Function('initialize', [pitch_deg], [x0, p0])

    return {
        'rhs': rhs,
        'predict': predict,
        'initialize': initialize,
        'force_moment': force_moment,
        'x': x,
        'u': u,
        'p': p
    }
    return rhs, x, u, p


def analyze_data(data):
    plt.figure(figsize=(10, 17))
    plt.subplot(321)
    plt.title('fuel')
    plt.plot(data['t'], data['x'][:, 13])
    plt.xlabel('t, sec')
    plt.ylabel('mass, kg')
    plt.grid()

    plt.subplot(322)
    plt.title('velocity')
    plt.plot(data['t'], data['x'][:, 7], label='v_X')
    plt.plot(data['t'], data['x'][:, 8], label='v_Y')
    plt.plot(data['t'], data['x'][:, 9], label='v_Z')
    plt.xlabel('t, sec')
    plt.ylabel('m/s')
    plt.grid()
    plt.legend()
    
    plt.subplot(323)
    euler = np.array(
        [np.array(ca.DM(so3.Euler.from_mrp(x))).reshape(-1) for x in data['x'][:, 3:7]])
    plt.plot(data['t'], np.rad2deg(euler[:, 0]), label='roll')
    plt.plot(data['t'], np.rad2deg(euler[:, 1]), label='pitch')
    plt.plot(data['t'], np.rad2deg(euler[:, 2]), label='yaw')
    plt.legend()
    plt.grid()
    plt.xlabel('t, sec')
    plt.ylabel('deg')
    plt.title('euler')
    
    plt.subplot(324)
    plt.title('angular velocity')
    plt.plot(data['t'], data['x'][:, 0], label='x')
    plt.plot(data['t'], data['x'][:, 1], label='y')
    plt.plot(data['t'], data['x'][:, 2], label='z')
    plt.xlabel('t, sec')
    plt.ylabel('rad/s')
    plt.grid()
    plt.legend()
    
    plt.subplot(325)
    plt.title('trajectory [side]')
    plt.plot(data['x'][:, 10], -data['x'][:, 12])
    plt.xlabel('North, m')
    plt.ylabel('Altitude, m')
    plt.axis('equal')
    plt.grid()
    
    plt.subplot(326)
    plt.title('trajectory [top]')
    plt.plot(data['x'][:, 11], data['x'][:, 10])
    plt.xlabel('East, m')
    plt.ylabel('North, m')
    plt.axis('equal')
    plt.grid()


def simulate(rocket, x0, u0, p0, dt=0.01, t0=0, tf=5):
    """
    An integrator using a fixed step runge-kutta approach.
    """
    x = x0
    data = {
        't': [],
        'x': []
    }
    for t in np.arange(t0, tf, dt):
        data['x'].append(np.array(x).reshape(-1))
        data['t'].append(t)
        x = rocket['predict'](x, u0, p0, t, dt)
   
    for k in data.keys():
        data[k] = np.array(data[k])
    return data


def gazebo_equations():
    # gazebo variables that you want to pass in
    omega_FLT = ca.SX.sym('omega_FLT', 3)
    pos_ENU = ca.SX.sym('pos_ENU', 3)
    vel_FLT = ca.SX.sym('vel_FLT', 3)
    q_ENU_FLT = ca.SX.sym('q_ENU_FLT', 4)
    m_fuel = ca.SX.sym('m_fuel')
    x_gz = ca.vertcat(omega_FLT, q_ENU_FLT, vel_FLT, pos_ENU, m_fuel)
    
    p = ca.SX.sym('p', 15)
    u = ca.SX.sym('u', 4)

    C_NED_ENU = np.array([
        #E  N   U
        [0, 1,  0], # N
        [1, 0,  0], # E
        [0, 0, -1]  # D
    ])

    C_FLT_FRB = np.array([
        #F   R  B
        [1,  0, 0],  # F
        [0, -1, 0],  # L
        [0,  0, -1]   # T
    ])
    
    C_ENU_FLT = so3.Dcm.from_quat(q_ENU_FLT)
    r_NED_FRB = so3.Mrp.from_dcm(ca.mtimes([C_NED_ENU, C_ENU_FLT, C_FLT_FRB]))
    
    omega_FRB = ca.mtimes(C_FLT_FRB.T, omega_FLT)
    pos_NED = ca.mtimes(C_NED_ENU, pos_ENU)
    vel_FRB = ca.mtimes(C_FLT_FRB.T, vel_FLT)
    
    x = ca.vertcat(omega_FRB, r_NED_FRB, vel_FRB, pos_NED, m_fuel)
    state_from_gz = ca.Function('gazebo_state', [x_gz], [x], ['x_gz'], ['x'])
    return {
        'state_from_gz': state_from_gz,
        'C_NED_ENU': C_NED_ENU,
        'C_FLT_FRB': C_FLT_FRB
    }


def code_generation():
    x_gz = ca.SX.sym('x_gz', 14)
    p = ca.SX.sym('p', 16)
    u = ca.SX.sym('u', 4)
    gz_eqs = gazebo_equations()
    x = gz_eqs['state_from_gz'](x_gz)
    eqs = rocket_equations()
    C_FLT_FRB = gz_eqs['C_FLT_FRB']
    F_FRB, M_FRB = eqs['force_moment'](x, u, p)
    F_FLT = ca.mtimes(C_FLT_FRB, F_FRB)
    M_FLT = ca.mtimes(C_FLT_FRB, M_FRB)
    f_force_moment = ca.Function('rocket_force_moment',
        [x_gz, u, p], [F_FLT, M_FLT], ['x_gz', 'u', 'p'], ['F_FLT', 'M_FLT'])
    gen = ca.CodeGenerator(
        'casadi_gen.c',
        {'main': False, 'mex': False, 'with_header': True, 'with_mem': True})
    gen.add(f_force_moment)
    gen.generate()
    

def run():
    rocket = rocket_equations()
    x0, p0 = rocket['initialize'](70)
    # m_dot, aileron, elevator, rudder
    u0 = [0.1, 0.01, 0.0, 0.0]
    data = simulate(rocket, x0, u0, p0, tf=10)
    analyze_data(data)
    plt.savefig('rocket.png')
    plt.show()

    code_generation()

run()
