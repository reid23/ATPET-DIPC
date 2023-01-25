#%%
from sympy import symbols
from sympy.physics.mechanics import *
mechanics_printing(pretty_print=True)
q = dynamicsymbols('q:3')
u = dynamicsymbols('u:3')
f = dynamicsymbols('f')
m = symbols('m:3')
l = symbols('l:2')
g, t = symbols('g t')

track = Body('track')
cart = Body('cart', mass=m[0])
arm1 = Body('arm1', mass=m[1])
arm2 = Body('arm2', mass=m[2])
joints = []
joints.append(PrismaticJoint(
    'x', track, cart, q[0], u[0],))
joints.append(PinJoint(
    'theta1', cart, arm1, q[1], u[1],
    parent_joint_pos = 0*cart.x, child_joint_pos=l[0]*arm1.x,
    parent_axis=cart.z, child_axis=arm1.z))
joints.append(PinJoint(
    'theta2', arm1, arm2, q[2], u[2],
    parent_joint_pos = 0*arm1.x, child_joint_pos=l[1]*arm1.x,
    parent_axis=arm1.z, child_axis=arm2.z))

cart.apply_force(f*track.x)
arm1.apply_force(g*m[1]*track.y)
arm2.apply_force(g*m[2]*track.y)

# L = Lagrangian(track, cart, arm1, arm2)
# LM = LagrangesMethod(L, q)
# LM.form_lagranges_equations()
#%%

jm = JointsMethod(track, *joints)
jm.form_eoms(LagrangesMethod)
# joint.kdes
# #%%
# child.masscenter.pos_from(parent.masscenter)
# #%%
# child.masscenter.vel(parent.frame)
# # %%

# %%
