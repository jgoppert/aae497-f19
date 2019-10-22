import casadi as ca
import sys

sys.path.insert(0, '/home/zp/aae497/aae497-f19/python/pyecca')
from pyecca.lie import so3
from pyecca.util import rk4

def rockt_eom(jit=True):
    x = ca.SX.sym('x', 14)
    u = ca.SX.sym('u', 4)
    p = ca.SX.sym('p', 15)

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
    CL_alpha = p[7]
    CL0 = p[8]
    CD0 = p[9]
    K = p[10]
    s_fin = p[11]
    rho = p[12]
    m_empty = p[13]
    l_motor = p[14]

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
    q = 0.5*rho*VT**2
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

    vel_tol = 1e-3
    FA_b = ca.SX.zeros(3)
    MA_b = ca.SX.zeros(3) 
    for key, data in fins.items():
        fwd = data['fwd']
        up = data['up']
        mix = data['mix']
        U = ca.dot(fwd, v_b)
        W = ca.dot(up, v_b)
        alpha = ca.if_else(
            ca.logic_and(ca.fabs(W) > vel_tol, ca.fabs(U) > vel_tol),
            -ca.atan(W/U), 0)
        rel_wind_dir = ca.if_else(ca.fabs(VT) > vel_tol, v_b/VT, -ca.DM(fwd))
        perp_wind_dir = ca.cross(ca.cross(fwd, up), rel_wind_dir)
        perp_wind_dir = perp_wind_dir/ca.norm_2(perp_wind_dir)
        CL = CL0 + CL_alpha*(alpha + mix)
        CD = CD0 + K*(CL - CL0)**2
        L = CL*q*s_fin
        D = CD*q*s_fin
        FA_b += L*perp_wind_dir - D*rel_wind_dir
        MA_b += ca.cross(ca.vertcat(-l_fin, 0, 0), FA_b)

    # propulsion
    FP_b = ca.vertcat(m_dot*ve, 0, 0)
    MP_b = ca.vertcat(0, 0, 0)
    
    # force and momental total
    F_b = FA_b + FP_b + ca.mtimes(C_nb.T, g_n)
    M_b = MA_b + MP_b

    # Casadi Functions
    rocket_aero_forces = ca.Function(
        'rocket_aero_forces',[x,u,p],[FA_b],['x','u','p'],['FA_b'])
    rocket_aero_moments = ca.Function(
        'rocket_aero_moments',[x,u,p],[MA_b],['x','u','p'],['MA_b'])
    rocket_prop_forces = ca.Function(
        'rocket_prop_forces',[x,u,p],[FP_b],['x','u','p'],['FP_b'])
    rocket_prop_moment = ca.Function(
        'rocket_prop_moments',[x,u,p],[MP_b],['x','u','p'],['MP_b'])
    
    gen = ca.CodeGenerator('casadi_gen_rocket.c', {'main': False, 'mex': False, 'with_header': True, 'with_mem': True})
    gen.add(rocket_aero_forces)
    gen.add(rocket_aero_moments)
    gen.add(rocket_prop_forces)
    gen.add(rocket_prop_moment)
    gen.generate()

rockt_eom()