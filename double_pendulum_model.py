#%%
from sympy import zeros, symbols
from sympy.solvers.ode import dsolve, systems
from sympy.physics.mechanics import Body, PinJoint, PrismaticJoint, JointsMethod, inertia
from sympy.physics.mechanics import dynamicsymbols
import sympy as sp
from sympy.physics.mechanics import *
from scipy.integrate import solve_ivp, solve_bvp
import numpy as np
import matplotlib.pyplot as plt
import control as ct
import matplotlib.animation as animation

#             ----> x
# ___________III______
#             \   |
#              \ / θ_1
#               \
#                l
#                 \
#                  mb (dist from top is lcom*l)
#                   \
#                    /\
#                   /  \
#                  /    \
#                 /(-θ_2)\
#                l
#               /
#             mc

#%%
# q1, q2, q3, u1, u2, u3 = dynamicsymbols('q1, q2, q3, u1, u2, u3')
y = dynamicsymbols('y:6')
# y1, y4 is cart pos, vel
# y2, y5 is cart's pivot joint pos, vel
# y3, y6 is mid-pendulum joint
l, lcom, c, g, t= symbols('l, l_{com}, c, g, t')
# l is first pend length
# lcom is pos of COM from cart pivot joint of first pend (fraction of L)
# c is friction of cart
# g is gravity (positive)
# t is time

ma, mb, mc, IBzz= symbols('m_a, m_b, m_c, I_{Bzz}')
# ma is mass of cart
# mb is mass of first pendulum
# mc is mass of end pendulum
# IBzz is the moment of inertia of the first pendulum
# moment of inertia of end pendulum is assumed to just come from mass at a radius

track = Body('N')
cart = Body('C', mass=ma)
IB = inertia(cart.frame, 0, 0, IBzz)
top_pend = Body('P_1', mass=mb, central_inertia=IB)
end_pend = Body('P_2', mass=mc)
bodies = (track, cart, top_pend, end_pend)

slider = PrismaticJoint('slider', track, cart, coordinates=y[0], speeds=y[3])
rev1 = PinJoint('r1', cart, top_pend, coordinates=y[1], speeds=y[4],
                child_axis=top_pend.z, child_joint_pos=l*lcom*top_pend.y,
                parent_axis=cart.z)

rev2 = PinJoint('r2', top_pend, end_pend, coordinates=y[2], speeds=y[5],
                child_axis=end_pend.z, parent_joint_pos=-l*(1-lcom)*top_pend.y,
                parent_axis=top_pend.z, child_joint_pos=l*end_pend.y)

joints = (slider, rev1, rev2)
F = dynamicsymbols('F')
cart.apply_force(F*cart.x) # motor
cart.apply_force(-c*y[3]*cart.x, reaction_body=track) # friction

# gravity
cart.apply_force(-track.y*cart.mass*g)
top_pend.apply_force(-track.y*top_pend.mass*g)
end_pend.apply_force(-track.y*end_pend.mass*g)

# get equations of motion
method = JointsMethod(track, slider, rev1, rev2)
method.form_eoms() #idk why but if i remove this it breaks
ydot = method.rhs() # get f where \dot{x} = f(x)

#%%
# prints out the equations of motion
x,   th1,   th2   = sp.symbols('x, \\theta_1, \\theta_2')
dx,  dth1,  dth2  = sp.symbols('\dot{x}, \dot{\\theta_1}, \dot{\\theta_2}')

#* print out LaTeX for equations of motion
# print("""
# \\begin{bmatrix}
# \dot x \\
# \ddot x \\
# \dot \\theta_1 \\
# \ddot \\theta_1 \\
# \dot \\theta_2 \\
# \ddot \\theta_2
# \end{bmatrix} = 
# """ + sp.latex(ydot.subs([
#     (y[0].diff(t), dx),
#     (y[1].diff(t), dth1),
#     (y[2].diff(t), dth2),
    
#     (y[0], x),
#     (y[1], th1),
#     (y[2], th2),
#     (y[3], dx),
#     (y[4], dth1),
#     (y[5], dth2),
# ])))
#%%
# A, B = method.method.to_linearizer().linearize(A_and_B=True)
# A_func = sp.lambdify([y, l, lcom, c, g, ma, mb, mc, IBzz], A)

#            l,    lcom,   c,    g,   ma,    mb,    mc, IBzz
constants = [0.25,  0.5, 0.1, 9.81, 0.25, 0.125, 0.125, 0.00065]

op_point=dict(zip(
    y+[F,l,lcom,c,g,ma,mb,mc,IBzz], 
    [0,sp.pi,0,0,0,0,0]+constants))
    
A, B = method.method.to_linearizer().linearize(A_and_B=True, op_point=op_point)
#%%
A, B = np.array(A).astype(np.float64), np.array(B).astype(np.float64)
print(np.linalg.matrix_rank(ct.ctrb(A, B))) #this should be 6 since we have 6 degrees of freedom

eigs = np.array([
    [-2.50], 
    [-2.51],
    [-2.52],
    [-2.53],
    [-2.54],
    [-2.55],
])

#construct proportional controller through pole placement
# aka place eigenvalues of A+B@(-K)
K = ct.place(A, B, eigs)
Q, R = np.zeros((6,6)), np.zeros((1,1))
np.fill_diagonal(Q, [100, 10, 5, 1, 100, 50])
np.fill_diagonal(R, [10])
K2, _, _ = ct.lqr(A, B, Q, R)

#* eigenvalues (should be equal to eigs)
print(np.linalg.eig(A-B@K2))


#%%

# J = ydot.jacobian([*y, F])
# j_func = sp.lambdify([y, F, l, lcom, c, g, ma, mb, mc, IBzz], J)
func = sp.lambdify([y, F, l, lcom, c, g, ma, mb, mc, IBzz], ydot)

# def get_linearized_func(x0):
#     return lambda x: func(x0, 0, *constants) + j_func(x0, 0, *constants)@(x-x0)
#%%
target = np.array([0, np.pi, 0, 0, 0, 0])
k = K[0]
def func_for_scipy(t, x):
    return func(x, (-K2@(x-target))[0], *constants).flatten()

soln = solve_ivp(func_for_scipy, (0, 5), [0, (7/8)*np.pi, 0, 0, 0, 0], t_eval = np.arange(0, 5, 1/60))
plt.plot(soln.t, soln.y[0], label='$x$ (m)')
plt.plot(soln.t, soln.y[3], label='$\dot{x}$ (m/s)')
plt.plot(soln.t, soln.y[1], label='$\\theta_1$ (rad)')
plt.plot(soln.t, soln.y[4], label='$\dot\\theta_1$ (rad/s)')
plt.plot(soln.t, soln.y[2], label='$\\theta_2$ (rad)')
plt.plot(soln.t, soln.y[5], label='$\dot\\theta_2$ (rad/s)')
plt.plot(soln.t, [-K2@(i-target) for i in soln.y.T], label='$u$ (N)')
plt.xlabel('Time (s)')
plt.legend()
# %%
from get_xy import get_xy
fig, ax = plt.subplots()
mat, = ax.plot(*get_xy(1, 3, 0.1, 1), marker='o')
ax.plot(-1, 2, linestyle = 'dashed')
# lines = ax.plot()
num_frames = len(soln.y[0])
def animate(i):
    mat.set_data(get_xy(*soln.y[0:3, i%(num_frames+30)-30 if i%(num_frames+30) > 29 else 0], constants[0]))
    return mat
ax.axis([-1,1,-0.25,0.75])
ani = animation.FuncAnimation(fig, animate, interval=50)
plt.show()
# %%
# def func_for_path_finding(x, y, p):

# solve_bvp()