import casadi as ca

x = ca.SX.sym('x')
y = 2*x
f = ca.Function('double_this', [x], [y], ['x'], ['y'])

gen = ca.CodeGenerator('casadi_gen.c', {'main': False, 'mex': False, 'with_header': True, 'with_mem': True})
gen.add(f)
gen.generate()
