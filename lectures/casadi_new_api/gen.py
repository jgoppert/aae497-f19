#!/usr/bin/env python

import casadi as ca

x = ca.SX.sym('x')
y = 2*x

f = ca.Function('f', [x], [y], ['x'], ['y'])
f.generate('func.c', {'with_header': True, 'with_mem': True})
