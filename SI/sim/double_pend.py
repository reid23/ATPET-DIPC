#%%
import sympy as sym
from sympy import symbols, zeros, pprint
from sympy.solvers.ode import dsolve, systems
# from sympy.physics.mechanics import Body, PinJoint, PrismaticJoint, JointsMethod, inertia, dynamicsymbols
from sympy.physics.mechanics import *
from scipy.integrate import solve_ivp, solve_bvp
import numpy as np
import control as ct
sym.init_printing()
sym.physics.mechanics.mechanics_printing()
#%%
q = dynamicsymbols('q:3')
dq = dynamicsymbols('\dot{q}_0 \dot{q}_1 \dot{q}_2')
f, u = dynamicsymbols('f u')

m = symbols('m:2')
l = symbols('l:2')
a = symbols('a:2')
I = symbols('I:2')
c = symbols('c:2')

#* known:
#* l[0]
#* m[1]
# %%

track = Body('N')
cart = Body('C')
# cart.masscenter.set_acc(cart.frame, u*track.x)
I0 = inertia(cart.frame, 0, 0, I[0])
I1 = inertia(cart.frame, 0, 0, I[1])
top_pend = Body('P_0', mass=m[0], central_inertia=I0)
end_pend = Body('P_1', mass=m[1], central_inertia=I1)

slider = PrismaticJoint('s', track, cart, coordinates=q[2], speeds=dq[2])

rev1 = PinJoint('r_1', cart, top_pend, coordinates=q[0], speeds=dq[0],
                child_axis=top_pend.z, child_joint_pos=a[0]*top_pend.y,
                parent_axis=cart.z)

rev2 = PinJoint('r_2', top_pend, end_pend, coordinates=q[1], speeds=dq[1],
                child_axis=end_pend.z, parent_joint_pos=-(l[0]-a[0])*top_pend.y,
                parent_axis=top_pend.z, child_joint_pos=a[1]*end_pend.y)


joints = (rev1, rev2, slider)
bodies = (track, cart, top_pend, end_pend)

cart.apply_force(f*cart.x) # motor


# %%

# gravity
g = 9.800 # in san francisco this is the true value

cart.apply_force(-track.y*cart.mass*g)
top_pend.apply_force(-track.y*top_pend.mass*g)
end_pend.apply_force(-track.y*end_pend.mass*g)

# get equations of motion
method = JointsMethod(track, slider, rev1, rev2)
method.form_eoms()
ydot = method.rhs()
# %%

eq = sym.Eq(u, ydot[3])
#%%
res = sym.solve(eq, f)
# %%
