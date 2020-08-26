import casadi as ca
import sys
sys.path.insert(0, '../../../python/pyecca')
import pyecca.lie.so3 as so3

r_ref = so3.Mrp.from_euler(ca.vertcat(0, 0, 0))

euler = ca.SX.sym('euler', 3)
r = so3.Mrp.from_euler(euler)
k_fin = 1.0

omega = so3.Mrp.log(so3.Mrp.product(so3.Mrp.inv(r), r_ref))
fin_pos = k_fin*omega

f_fin = ca.Function('fins', [euler], [fin_pos], ['euler'], ['fin_pos'])
f_fin.generate("fin.c", {"main": False, "with_header": True, "with_mem": True})
