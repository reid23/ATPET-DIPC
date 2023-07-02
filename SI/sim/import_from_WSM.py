#%%
import casadi as ca
from casadi import DaeBuilder, integrator, MX
import numpy as np
from matplotlib import pyplot as plt
from scipy.integrate import solve_ivp

dae = DaeBuilder('f', 'DIPC8')
#%%
params = [0.001, 0.01, 0.3, 0.15, 0.1,
          0.001, 0.01, 0.3, 0.15, 0.1]
for param in zip(['top_friction', 'top_inertia', 'top_L', 'top_lc', 'top_mass', 
              'end_friction', 'end_inertia', 'end_L', 'end_lc', 'end_mass'],
              params):
    dae.register_p(param[0])
    # dae.set(*param)
#%%
# f = dae.create('banana')
# soln = solve_ivp(lambda t, y: np.array(f(x=y, u=0.1, p=params)['ode']).flatten(), t_span=(0, 10), y0=[0,0,0,0,0,0])
# plt.plot(soln.t, soln.y.T)
# f2 = dae.create('f', ['t', 'x', 'z', 'p', 'u'], ['ode'], {'aux': ['y0', 'y1', 'y2', 'y3', 'y4', 'y5']})
# f3 = dae.create('f2', ['t', 'x', 'z', 'p', 'u'], ['ode', 'alg', 'quad'])
# %%
# test = dae.create('test')
grid=np.arange(0, 10, 0.01)
intfunc = integrator('intfunc', 'rk', dae.create('for_intfunc'), 0, grid)
# %%
res = intfunc(x0=[0, 0, ca.pi/2, 0, 0, 0], u=[1]*250 + [-1]*500 + [1]*250, p=params)
plt.plot(grid, res['xf'].T, label=['$x$', '$\dot x$', '$\\theta_1$', '$\dot \\theta_1$', '$\\theta_2$', '$\dot \\theta_2$'])
plt.legend()
plt.show()
# %%

# #* please forgive me for the following class.
# #* I couldn't think of a better way to do this.
# #* **pain**
# class part:
#     def __init__(self, **things):
#         self.print_str = str(things)
#         self.__dict__.update(self._convert(things))
#     def _convert(self, things):
#         for key, val in things.items():
#             match val:
#                 case int(size):
#                     things[key] = mx(key, size) if size>0 else mx(key)
#                 case (int(rows), int(cols)):
#                     things[key] = mx(key, rows, cols)
#                 case (int(size), subfields):
#                     things[key] = mx(key, size) if size>0 else mx(key)
#                     things[key].__dict__.update(self._convert(subfields))
#         return things
#     def __repr__(self):
#         return self.print_str

# def mx(name, *args, **kwargs):
#     thing = MX.sym(name, *args, **kwargs)
#     thing.__dict__.update({'start': MX.sym(f'{name}.start')})
#     return thing
# #%%
# u = MX.sym('u')
# prismatic1 = part(a=0)
# accelerate1 = part(s=(0, dict(start=0)), 
#                    v=(0, dict(start=0)))
# cart = part(r_0=4,
#             a_0=4,
#             v_0=4,
#             body=(0, dict(z_a=4, 
#                           z_0_start=4, 
#                           R_start=(4, 4),
#                           w_0_start=4)),
#             )


# prismatic1.a = u
# accelerate1.s = accelerate1.s.start
# cart.r_0[1] = accelerate1.s
# end_pend.shape1.r[3] = 0.0
# end_pend.r_0[3] = end_pend.shape1.r[3]
# damper1.phi_rel = damper1.phi_rel.start
# revolute1.R_rel.T[2,2] = cos(damper1.phi_rel)
# end_pend.shape1.r[2] = 0.3 * revolute1.R_rel.T[2,2]
# end_pend.r_0[2] = end_pend.shape1.r[2]
# -revolute1.R_rel.T[1,2] = -sin(damper1.phi_rel)
# top_pend.shape1.r[1] = accelerate1.s + 0.1
# end_pend.shape1.r[1] = (-0.3) * revolute1.R_rel.T[1,2] + top_pend.shape1.r[1]
# end_pend.r_0[1] = end_pend.shape1.r[1]
# top_pend.r_0[1] = top_pend.shape1.r[1]
# cart.a_0[1] = u
# accelerate1.v = accelerate1.v.start
# cart.v_0[1] = accelerate1.v
# end_pend.a_0[3] = 0.0
# end_pend.v_0[3] = 0.0
# top_pend.a_0[3] = 0.0
# top_pend.v_0[3] = 0.0
# top_pend.r_0[3] = 0.0
# top_pend.a_0[2] = 0.0
# top_pend.v_0[2] = 0.0
# top_pend.r_0[2] = 0.0
# top_pend.a_0[1] = u
# top_pend.v_0[1] = accelerate1.v
# cart.a_0[3] = 0.0
# cart.v_0[3] = 0.0
# cart.r_0[3] = 0.0
# cart.a_0[2] = 0.0
# cart.v_0[2] = 0.0
# cart.r_0[2] = 0.0
# cart.body.z_a[1].start = cart.body.z_0_start[3] * cart.body.R_start.T[1,3] + cart.body.z_0_start[1] * cart.body.R_start.T[1,1] + cart.body.z_0_start[2] * cart.body.R_start.T[1,2]
# cart.body.z_a[1] = 0.0
# cart.body.z_a[2].start = cart.body.z_0_start[3] * cart.body.R_start.T[2,3] + cart.body.z_0_start[1] * cart.body.R_start.T[2,1] + cart.body.z_0_start[2] * cart.body.R_start.T[2,2]
# cart.body.z_a[2] = 0.0
# cart.body.z_a[3].start = cart.body.z_0_start[3] * cart.body.R_start.T[3,3] + cart.body.z_0_start[1] * cart.body.R_start.T[3,1] + cart.body.z_0_start[2] * cart.body.R_start.T[3,2]
# cart.body.z_a[3] = 0.0
# top_pend.body.z_a[1].start = top_pend.body.z_0_start[3] * top_pend.body.R_start.T[1,3] + top_pend.body.z_0_start[1] * top_pend.body.R_start.T[1,1] + top_pend.body.z_0_start[2] * top_pend.body.R_start.T[1,2]
# top_pend.body.z_a[1] = 0.0
# top_pend.body.z_a[2].start = top_pend.body.z_0_start[3] * top_pend.body.R_start.T[2,3] + top_pend.body.z_0_start[1] * top_pend.body.R_start.T[2,1] + top_pend.body.z_0_start[2] * top_pend.body.R_start.T[2,2]
# top_pend.body.z_a[2] = 0.0
# damper1.w_rel = damper1.w_rel.start
# top_pend.body.w_a[3].start = top_pend.body.w_0_start[3] * top_pend.body.R_start.T[3,3] + top_pend.body.w_0_start[1] * top_pend.body.R_start.T[3,1] + top_pend.body.w_0_start[2] * top_pend.body.R_start.T[3,2]
# top_pend.body.w_a[3] = damper1.w_rel
# top_pend.body.w_a[2].start = top_pend.body.w_0_start[3] * top_pend.body.R_start.T[2,3] + top_pend.body.w_0_start[1] * top_pend.body.R_start.T[2,1] + top_pend.body.w_0_start[2] * top_pend.body.R_start.T[2,2]
# top_pend.body.w_a[2] = 0.0
# top_pend.body.w_a[1].start = top_pend.body.w_0_start[3] * top_pend.body.R_start.T[1,3] + top_pend.body.w_0_start[1] * top_pend.body.R_start.T[1,1] + top_pend.body.w_0_start[2] * top_pend.body.R_start.T[1,2]
# top_pend.body.w_a[1] = 0.0
# prismatic1.s = accelerate1.s
# cart.body.w_a[3].start = cart.body.w_0_start[3] * cart.body.R_start.T[3,3] + cart.body.w_0_start[1] * cart.body.R_start.T[3,1] + cart.body.w_0_start[2] * cart.body.R_start.T[3,2]
# cart.body.w_a[3] = 0.0
# cart.body.w_a[2].start = cart.body.w_0_start[3] * cart.body.R_start.T[2,3] + cart.body.w_0_start[1] * cart.body.R_start.T[2,1] + cart.body.w_0_start[2] * cart.body.R_start.T[2,2]
# cart.body.w_a[2] = 0.0
# cart.body.w_a[1].start = cart.body.w_0_start[3] * cart.body.R_start.T[1,3] + cart.body.w_0_start[1] * cart.body.R_start.T[1,1] + cart.body.w_0_start[2] * cart.body.R_start.T[1,2]
# cart.body.w_a[1] = 0.0
# cart.body.r_0[1] = accelerate1.s
# cart.body.r_0[2] = 0.0
# cart.body.r_0[3] = 0.0
# prismatic1.v = accelerate1.v
# end_pend.body.w_a[1].start = end_pend.body.w_0_start[3] * end_pend.body.R_start.T[1,3] + end_pend.body.w_0_start[1] * end_pend.body.R_start.T[1,1] + end_pend.body.w_0_start[2] * end_pend.body.R_start.T[1,2]
# end_pend.body.w_a[1] = 0.0
# end_pend.body.w_a[2].start = end_pend.body.w_0_start[3] * end_pend.body.R_start.T[2,3] + end_pend.body.w_0_start[1] * end_pend.body.R_start.T[2,1] + end_pend.body.w_0_start[2] * end_pend.body.R_start.T[2,2]
# end_pend.body.w_a[2] = 0.0
# damper2.w_rel = damper2.w_rel.start
# end_pend.body.w_a[3].start = end_pend.body.w_0_start[3] * end_pend.body.R_start.T[3,3] + end_pend.body.w_0_start[1] * end_pend.body.R_start.T[3,1] + end_pend.body.w_0_start[2] * end_pend.body.R_start.T[3,2]
# end_pend.body.w_a[3] = damper1.w_rel + damper2.w_rel
# end_pend.body.z_a[1].start = end_pend.body.z_0_start[3] * end_pend.body.R_start.T[1,3] + end_pend.body.z_0_start[1] * end_pend.body.R_start.T[1,1] + end_pend.body.z_0_start[2] * end_pend.body.R_start.T[1,2]
# end_pend.body.z_a[1] = 0.0
# end_pend.body.z_a[2].start = end_pend.body.z_0_start[3] * end_pend.body.R_start.T[2,3] + end_pend.body.z_0_start[1] * end_pend.body.R_start.T[2,1] + end_pend.body.z_0_start[2] * end_pend.body.R_start.T[2,2]
# end_pend.body.z_a[2] = 0.0
# damper2.phi_rel = damper2.phi_rel.start
# -revolute2.R_rel.T[1,2] = -sin(damper2.phi_rel)
# revolute2.R_rel.T[2,2] = cos(damper2.phi_rel)
# end_pend.shape1.rxvisobj[1] = (-revolute1.R_rel.T[2,2]) * revolute2.R_rel.T[1,2] - revolute1.R_rel.T[1,2] * revolute2.R_rel.T[2,2]
# end_pend.shape1.rxvisobj[2] = revolute1.R_rel.T[2,2] * revolute2.R_rel.T[2,2] - revolute1.R_rel.T[1,2] * revolute2.R_rel.T[1,2]
# end_pend.body.z_a[3].start = end_pend.body.z_0_start[3] * end_pend.body.R_start.T[3,3] + end_pend.body.z_0_start[1] * end_pend.body.R_start.T[3,1] + end_pend.body.z_0_start[2] * end_pend.body.R_start.T[3,2]
# -revolute2.tau = damper2.d * damper2.w_rel
# -revolute1.tau = damper1.d * damper1.w_rel
# -top_pend.frame_b.t[3] = revolute2.tau
# end_pend.shape1.ryvisobj[1] = revolute1.R_rel.T[2,2] * revolute2.R_rel.T[2,2] - revolute1.R_rel.T[1,2] * revolute2.R_rel.T[1,2]
# end_pend.shape1.ryvisobj[2] = revolute1.R_rel.T[1,2] * revolute2.R_rel.T[2,2] + revolute1.R_rel.T[2,2] * revolute2.R_rel.T[1,2]


# revolute2.tau = end_pend.body.I_33 * end_pend.body.z_a[3] - end_pend.body.w_a[2] * (end_pend.body.I_31 * end_pend.body.w_a[3] + end_pend.body.I_11 * end_pend.body.w_a[1] + end_pend.body.I_21 * end_pend.body.w_a[2]) + end_pend.body.I_31 * end_pend.body.z_a[1] + end_pend.body.I_32 * end_pend.body.z_a[2] - end_pend.body.r_CM[2] * end_pend.body.frame_a.f[1] + end_pend.body.w_a[1] * (end_pend.body.I_32 * end_pend.body.w_a[3] + end_pend.body.I_21 * end_pend.body.w_a[1] + end_pend.body.I_22 * end_pend.body.w_a[2]) + end_pend.body.r_CM[1] * end_pend.body.frame_a.f[2]
# -top_pend.frame_b.f[1] = end_pend.body.frame_a.f[1] * revolute2.R_rel.T[2,2] - end_pend.body.frame_a.f[2] * revolute2.R_rel.T[1,2]
# top_pend.body.frame_a.f[2] = top_pend.body.m * (world.g * revolute1.R_rel.T[2,2] - u * revolute1.R_rel.T[1,2] - top_pend.body.r_CM[2] * damper1.w_rel ^ 2 + top_pend.body.r_CM[1] * damper1.a_rel)
# top_pend.body.frame_a.f[1] = top_pend.body.m * (u * revolute1.R_rel.T[2,2] - top_pend.body.r_CM[1] * damper1.w_rel ^ 2 - top_pend.body.r_CM[2] * damper1.a_rel + world.g * revolute1.R_rel.T[1,2])
# top_pend.body.frame_a.t[3] = top_pend.body.I_33 * damper1.a_rel - top_pend.body.r_CM[2] * top_pend.body.frame_a.f[1] + top_pend.body.r_CM[1] * top_pend.body.frame_a.f[2]
# top_pend.body.frame_a.t[3] - revolute1.tau + top_pend.frameTranslation.frame_a.t[3] = 0.0
# top_pend.frameTranslation.frame_a.t[3] + (-0.3) * top_pend.frame_b.f[1] + top_pend.frame_b.t[3] = 0.0
# der(der(revolute1.R_rel.T[2,1])) = damper1.w_rel ^ 2 * sin(damper1.phi_rel) - damper1.a_rel * cos(damper1.phi_rel)
# end_pend.a_0[1] = u + 0.3 * der(der(revolute1.R_rel.T[2,1]))
# der(der(revolute1.R_rel.T[2,2])) = (-damper1.w_rel ^ 2) * cos(damper1.phi_rel) - damper1.a_rel * sin(damper1.phi_rel)
# end_pend.a_0[2] = 0.3 * der(der(revolute1.R_rel.T[2,2]))
# end_pend.body.frame_a.f[2] = end_pend.body.m * (end_pend.a_0[1] * end_pend.shape1.rxvisobj[1] - end_pend.body.w_a[1] * (end_pend.body.r_CM[2] * end_pend.body.w_a[1] - end_pend.body.r_CM[1] * end_pend.body.w_a[2]) + (end_pend.a_0[2] + world.g) * end_pend.shape1.rxvisobj[2] - end_pend.body.r_CM[3] * end_pend.body.z_a[1] + end_pend.body.r_CM[1] * end_pend.body.z_a[3] + end_pend.body.w_a[3] * (end_pend.body.r_CM[3] * end_pend.body.w_a[2] - end_pend.body.r_CM[2] * end_pend.body.w_a[3]))
# end_pend.body.frame_a.f[1] = end_pend.body.m * (end_pend.a_0[1] * end_pend.shape1.ryvisobj[1] - end_pend.body.w_a[3] * (end_pend.body.r_CM[1] * end_pend.body.w_a[3] - end_pend.body.r_CM[3] * end_pend.body.w_a[1]) + (end_pend.a_0[2] + world.g) * end_pend.shape1.ryvisobj[2] - end_pend.body.r_CM[2] * end_pend.body.z_a[3] + end_pend.body.r_CM[3] * end_pend.body.z_a[2] + end_pend.body.w_a[2] * (end_pend.body.r_CM[2] * end_pend.body.w_a[1] - end_pend.body.r_CM[1] * end_pend.body.w_a[2]))
# # %%
