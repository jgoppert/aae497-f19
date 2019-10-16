
import rocket_casadi
import numpy as np
import casadi as ca
import matplotlib.pyplot as plt

eqs = rocket_casadi.rocket_equations()
x0, p0 = eqs['initialize'](75)

dt = 0.1
m_dot = 0.1
t_vect = np.arange(0, 4, dt)
n = len(t_vect)
elv_vect = ca.SX.sym('elv', n)
x = x0
for i in range(n):
    t = t_vect[i]
    elv = elv_vect[i]
    u = ca.vertcat(m_dot, 0, elv, 0)
    x = eqs['predict'](x, u, p0, t, dt)
x_final = x
dx_final = eqs['rhs'](x_final, u, p0)

s0 = 0*np.ones(n)
x = ca.SX.sym('x')
nlp = {'x': elv_vect,
        'f': ca.dot(elv_vect, elv_vect),
        'g': ca.vertcat(dx_final[12])}

args = {
    'print_time': 1,
    'monitor': ['nlp_f', 'nlp_g'],
    'ipopt': {
        'sb': 'yes',
        'print_level': 5,
        }
    }
S = ca.nlpsol('S', 'ipopt', nlp, args)
n_g=1
res = S(x0=s0, lbg=np.zeros(n_g), ubg=np.zeros(n_g), lbx=-0.2*np.ones(n), ubx=0.2*np.ones(n))
#stats = S.stats()
#stats['return_status']
#res

def simulate(rocket, x0, elv_list, p0, dt=0.005, t0=0, tf=5):
    """
    An integrator using a fixed step runge-kutta approach.
    """
    x = x0
    data = {
        't': [],
        'x': [],
        'u': []
    }
    u = np.array([0.1, 0, 0, 0])
    for i, t in enumerate(np.arange(t0, tf, dt)):
        elv = elv_list[i]
        data['t'].append(t)
        data['x'].append(np.array(x).reshape(-1))
        data['u'].append(np.array(u).reshape(-1))
        u[1] =  elv
        x = rocket['predict'](x, u, p0, t, dt)
   
    for k in data.keys():
        data[k] = np.array(data[k])
    return data

data = simulate(eqs, x0, res['x'], p0, dt, 0, 4)
plt.plot(data['t'], -data['x'][:,12])
plt.grid()


plt.figure()
plt.plot(res['x'])
plt.show()