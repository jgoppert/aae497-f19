import casadi as ca

x = ca.SX.sym('x', 14)
u = ca.SX.sym('u', 4)
p = ca.SX.sym('p', 15)

F_aero = ca.vertcat(p[0], p[1], p[2])
F_prop = ca.vertcat(x[3], x[4], x[5])

f = ca.Function('double_this', [x, u, p], [F_aero, F_prop],
    ['x', 'u', 'p'], ['F_aero', 'F_prop'])

gen = ca.CodeGenerator('casadi_gen.c', {'main': False, 'mex': False, 'with_header': True, 'with_mem': True})
gen.add(f)
gen.generate()
