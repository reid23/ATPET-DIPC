#%%
import numpy as np
import casadi as ca
from casadi import MX, DM
from casadi import vertcat, horzcat
from casadi import sin, cos
from casadi import integrator, nlpsol
import matplotlib.pyplot as plt

#%%
class PathFollowMPC:
    def __init__(self, N, tstep, l, c, px, pu, Q, R):
        self.N = N
        self.tstep = tstep
        self.thoriz = N*tstep
        self.params = (l, c)
        self.p_num_x = px
        self.p_num_u = pu
        self.Q = DM(np.diag(Q))
        self.R = DM(np.diag(R))
        self.solver_opts = {
            'ipopt.linear_solver': 'ma27',
            
        }
        # self.integrator = self._get_integrator(np.linspace(self.tstep, self.thoriz, self.N, dtype=float).tolist(), self.params)
        self.integrator = self._get_integrator(self.tstep, self.params)
        ## INITIALIZE VARIABLES
        self.u = MX.sym('u', self.N)
        self.p = MX.sym('p', 4, self.N+1) # first is x0
        self.x = self.integrator(x0=self.p[:, 0], p=self.u)['xf']

        ## INITIALIZE COST
        y = MX.sym('y', 4)
        self.quad_form_cost = ca.Function('quad', [y], [y.T@self.Q@y]).map(self.N)

        self.cost = ca.sum2(self.quad_form_cost(self.p[:, 1:]-self.x))
        self.cost += ca.sumsqr(self.cost*np.sqrt(self.R))

        self.g = horzcat(
            self.x[0, :], 
            9*self.x[2, :]**4 + 50*self.x[2, :]**2 * (self.u**2 + 0.26*self.u**4).T
        )

        self._construct_nlp()
        
    def _get_integrator(self, t_step, p):
        l, c = p[0], p[1]
        f = MX.sym('f')
        y = MX.sym('y', 4)
        ydot = horzcat(
            y[2],
            y[3],
            f,
            -c*y[3] + (-9.8*sin(y[1])-f*cos(y[1]))/l
        )
        ode = {'x':y, 'p':f, 'ode': ydot}
        self.int_one_step = integrator('F_i', 'rk', ode, 0, t_step)
        return self.int_one_step.mapaccum('int', self.N, ['x0'], ['xf'])
    def _construct_nlp(self):
        nlp = {
            'x': self.u,    # control inputs
            'f': self.cost, # cost function
            'p': self.p,    # path to follow
            'g': self.g,
        }
        self.solver = nlpsol('solver', 'ipopt', nlp, self.solver_opts)
    def solve(self, x0):
        self.soln = self.solver(
            x0 = self.p_num_u,
            p = horzcat(DM(x0), DM(self.p_num_x).T),
            lbg = [-0.8]*self.N + [0.]*self.N,
            ubg = [0.8]*self.N + [100.]*self.N,
        )

    def _update_path(self, px, pu):
        self.p_num_x = DM(px)
        self.p_num_u = DM(pu)
#%%

n = 10
p = PathFollowMPC(
    N=n, 
    tstep=0.1, 
    l=0.220451, 
    c=0.139185, 
    px=DM([[0, np.pi, 0, 0]]*n), 
    pu=DM([0.]*n), 
    Q=[1e1, 1e3, 1e0, 1e0], 
    R=[1e0])
x0 = DM([0.1, np.pi, 0., 0.])
#%%
p.solve(x0)

#%%
u_res = p.soln['x']
# ff_prog = [2.575035906172058, -3.8158905593996133, -2.0671993855312296, 4.139213882689204, 2.2243153434826377, 1.4948059154622, -4.421279471702257, -2.082844104057677, -1.819244414580912, 4.242038184131376, 2.3720656176437367, -3.938846230847921, 0.]
# u_res = DM(ff_prog+[0.]*7)
# res = p.integrator(x0=x0, u = u_res)['xf']
# res = np.array(res)
# p._update_path(res.T, ff_prog + [0.]*7)
# p.solve(x0)
# u_res = p.soln['x']
# # res = DM(ff_prog)

def plot(u, x0, x=None):
    if x is None:
        res = p.integrator(x0=x0, p = u)['xf']
        res = np.array(res)
    else:
        res = np.array(x)
    fig, axs = plt.subplots(2, 2)
    titles = ['cart pos', 'pend angle', 'cart vel', 'pend vel']
    for idx, ax in enumerate(axs.flatten()):
        ax.plot(np.linspace(0, p.thoriz, res.shape[1]), res[idx])
        ax.set_title(titles[idx])
    axs[0][0].plot(np.linspace(0, p.thoriz, res.shape[1]), u, color='black')
    plt.show()

plot(u_res, x0)
#%%
exit()
# %%
class PathFollowMPCOpti:
    def __init__(self, N, tstep, l, c, px, pu, Q, R):
        self.opti = ca.Opti()
        self.N = N
        self.tstep = tstep
        self.thoriz = self.N*self.tstep
        self.params = l, c
        self.p_num_x = px
        self.p_num_u = pu
        self.Q = DM(np.diag(Q))
        self.R = DM(np.diag(R))
        y = MX.sym('y', 4)
        self.popts = {
            'print_time': 0
        }
        self.sopts = {
            # 'print_level': 0,
            # 'sb': 'yes',
            'linear_solver': 'MA57',
        }
        self.tgrid = np.linspace(self.tstep, self.thoriz, self.N, dtype=float).tolist()
        self.integrator = self._get_integrator(self.tstep, self.params)
        self.quad_form_cost = ca.Function('quad', [y], [y.T@self.Q@y])

        self.x0 = self.opti.parameter(4)
        self.opti.set_value(self.x0, DM.zeros(4))
        self.p = self.opti.parameter(4, self.N)
        self.opti.set_value(self.p, DM([[0., np.pi, 0., 0.]]*self.N).T)
        self.u = self.opti.variable(self.N)
        self.x = self.integrator(x0=self.x0, u=self.u)['xf']

        self.cost = self.quad_form_cost.mapsum([self.p-self.x], 'unroll')
        self.opti.minimize(self.cost[0])
        self.opti.subject_to(ca.Opti_bounded(-0.8, self.x[0, :], 0.8))
        self.opti.subject_to(9*self.x[2, :]**4 + 50*self.x[2, :]**2 * (self.u**2 + 0.26*self.u**4).T < 100)

        self.opti.solver('ipopt', self.popts, self.sopts)
    def _get_integrator(self, t_step, p):
        l, c = p[0], p[1]
        f = MX.sym('f')
        y = MX.sym('y', 4)
        ydot = horzcat(
            y[2],
            y[3],
            f,
            -c*y[3] + (-9.8*sin(y[1])-f*cos(y[1]))/l
        )
        ode = {'x':y, 'p':f, 'ode': ydot}
        self.int_one_step = integrator('F_i', 'rk', ode, 0, t_step)
        return self.int_one_step.mapaccum('int', self.N, ['x0'], ['xf'])

    def update(self, plan_x, plan_u, x0):
        self.p_num_x = DM(plan_x)
        self.p_num_u = DM(plan_u)
        self.opti.set_value(self.p, self.p_num_x)
        self.opti.set_value(self.x0, DM(x0))
        self.opti.set_initial(self.u, self.p_num_u)
    def solve(self):
        self.soln = self.opti.solve()


# %%
p = PathFollowMPCOpti(20, 0.1, 0.220451, 0.139185, DM([[0, 0, 0, 0]]*10), DM([0]*10), [10, 1000, 10, 1], [10])
#%%
p.update(DM([[0, 0, 0, 0]]*p.N).T, DM([0.]*p.N), [0., 2, 0., 0.])
# %%
p.solve()
res = np.concatenate([np.array([[0.], [0.1], [0.], [0.]]), p.soln.value(p.x)], axis=1)
fig, axs = plt.subplots(2, 2)
titles = ['cart pos', 'pend angle', 'cart vel', 'pend vel']
for idx, ax in enumerate(axs.flatten()):
    ax.plot(np.linspace(0, p.thoriz, p.N+1), res[idx])
    ax.set_title(titles[idx])
plt.show()
# %%
