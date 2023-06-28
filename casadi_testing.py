#%%
from casadi import sin, cos, pi, integrator, vertcat, SX, MX, DM, fabs, exp, bilin
import numpy as np
import casadi
from time import perf_counter
from MPC_testing import get_mpc
import sys
from scipy.integrate import solve_ivp
from time import sleep
#%%
# l, ma, mb, I, c = [0.23116035, 0.00625   , 0.05      , 0.        , 0.10631411]
p = MX.sym('p', 6)
l, ma, mb, I, c = p[0], p[1], p[2], p[3], p[4]
f = p[5]#MX.sym('f')
y = MX.sym('y', 4)

ydot = vertcat(
    y[2],
    y[3],
    f,
    -c*y[3] + (-9.8*l*mb*sin(y[1]) - l*mb*(l*mb*y[3]**2*sin(y[1]) + (-I*l*mb*y[3]**2*sin(y[1]) + I*ma*f + I*mb*f - l**3*mb**2*y[3]**2*sin(y[1]) + l**2*ma*mb*f - l**2*mb**2*f*cos(y[1])**2 + l**2*mb**2*f - 4.9*l**2*mb**2*sin(2.0*y[1]))/(I + l**2*mb))*cos(y[1])/(ma + mb))/(I - l**2*mb**2*cos(y[1])**2/(ma + mb) + l**2*mb)
)
p = MX.sym('p', 3)
l, c, f = p[0], p[1], p[2]
ydot = vertcat(
    y[2],
    y[3],
    f,
    -c*y[3] + (-9.8*sin(y[1])-f*cos(y[1]))/l
)
ode = {'x':y, 'p': p, 'ode': ydot}


# dae = DaeBuilder()

# l = dae.add_p('l')
# ma = dae.add_p('ma')
# mb = dae.add_p('mb')
# I = dae.add_p('I')
# c = dae.add_p('c')

# f = dae.add_u('f')

# x = dae.add_x('x')
# th = dae.add_x('th')
# dx = dae.add_x('dx')
# dth = dae.add_x('dth')

# dae.add_ode('x', dx)
# dae.add_ode('th', dth)
# dae.add_ode('dx', f)
# dae.add_ode('dth', -c*dth + (-9.8*l*mb*sin(th) - l*mb*(l*mb*dth**2*sin(th) + (-I*l*mb*dth**2*sin(th) + I*ma*f + I*mb*f - l**3*mb**2*dth**2*sin(th) + l**2*ma*mb*f - l**2*mb**2*f*cos(th)**2 + l**2*mb**2*f - 4.9*l**2*mb**2*sin(2.0*th))/(I + l**2*mb))*cos(th)/(ma + mb))/(I - l**2*mb**2*cos(th)**2/(ma + mb) + l**2*mb))

# dae.set_unit('x','m')
# dae.set_unit('x','m/s')
# dae.set_unit('th','rad')
# dae.set_unit('dth','rad/s')
# dae.set_unit('f','m/s^2')


def get_integrator(t_step, p):
    l, ma, mb, I, c = p[0], p[1], p[2], p[3], p[4]
    f = SX.sym('f')
    y = SX.sym('y', 4)
    ydot = vertcat(
        y[2],
        y[3],
        f,
        -c*y[3] + (-9.8*l*mb*sin(y[1]) - l*mb*(l*mb*y[3]**2*sin(y[1]) + (-I*l*mb*y[3]**2*sin(y[1]) + I*ma*f + I*mb*f - l**3*mb**2*y[3]**2*sin(y[1]) + l**2*ma*mb*f - l**2*mb**2*f*cos(y[1])**2 + l**2*mb**2*f - 4.9*l**2*mb**2*sin(2.0*y[1]))/(I + l**2*mb))*cos(y[1])/(ma + mb))/(I - l**2*mb**2*cos(y[1])**2/(ma + mb) + l**2*mb)
    )
    ode = {'x':y, 'p':f, 'ode': ydot}
    if isinstance(t_step, float): 
        return integrator('F_i', 'rk', ode, dict(t0=0, tf=t_step))
    return integrator('F_i', 'rk', ode, dict(grid=t_step))

def get_integrator_simple(t_step, p):
    l, c = p[0], p[1]
    f = SX.sym('f')
    y = SX.sym('y', 4)
    ydot = vertcat(
        y[2],
        y[3],
        f,
        -c*y[3] + (-9.8*sin(y[1])-f*cos(y[1]))/l
    )
    ode = {'x':y, 'p':f, 'ode': ydot}
    if isinstance(t_step, float): 
        return integrator('F_i', 'rk', ode, dict(t0=0, tf=t_step))
    return integrator('F_i', 'rk', ode, dict(grid=t_step))

def eval_ode_casadi(t_step, t_f, x0=np.array([0,pi/2,0,0]), p=np.array([0.23116035, 0.00625   , 0.05      , 0.        , 0.10631411, 0])):
    int_func = integrator('F_i', 'cvodes', ode, dict(t0=0, tf=t_step))
    # x0 = np.array([0,np.pi/2,0,0])
    p=np.array([0.23116035, 0.00625   , 0.05      , 0.        , 0.10631411, 0])
    res = []
    for i in range(int(t_f/t_step)):
        x0 = int_func(x0=x0, p=p)['xf']
        res.append(x0)
    return np.array(res)
#%%
def model(t, y, l, ma, mb, I, c, f):
    return np.array([
        y[2],
        y[3],
        f,
        -c*y[3] + (-9.8*l*mb*np.sin(y[1]) - l*mb*(l*mb*y[3]**2*np.sin(y[1]) + (-I*l*mb*y[3]**2*np.sin(y[1]) + I*ma*f + I*mb*f - l**3*mb**2*y[3]**2*np.sin(y[1]) + l**2*ma*mb*f - l**2*mb**2*f*np.cos(y[1])**2 + l**2*mb**2*f - 4.9*l**2*mb**2*np.sin(2.0*y[1]))/(I + l**2*mb))*np.cos(y[1])/(ma + mb))/(I - l**2*mb**2*np.cos(y[1])**2/(ma + mb) + l**2*mb)
    ])
def eval_ode_scipy(t_step, t_f, x0=np.array([0,np.pi/2,0,0]), p=np.array([0.23116035, 0.00625   , 0.05      , 0.        , 0.10631411, 0])):
    return solve_ivp(lambda t, y: model(t, y, *p), y0=x0, t_span=(0, t_f), t_eval=np.arange(0, t_f, t_step)).y

class controller:
    def __init__(self, tstep=0.2, thoriz=1, l=0.220451, c=0.139185, x0=[0,0,0,0]):
        self.tstep = tstep
        self.thoriz = thoriz
        self.nstep = int(thoriz/tstep)
        self.p = DM([l, c])
        f = SX.sym('f')
        y = SX.sym('y', 4)
        ydot = vertcat(
            y[2],
            y[3],
            f,
            -c*y[3] + (-9.8*sin(y[1])-f*cos(y[1]))/l
        )
        ode = {'x':y, 'p':f, 'ode': ydot}
        self.intfunc = integrator('F_i', 'rk', ode, dict(t0=0, tf=self.tstep))
        self.solver_opts = {
            'ipopt.print_level': 0,
            'ipopt.sb': 'yes',
            'print_time': 0,
            'ipopt.linear_solver': 'MA27',
        }
        self.x0 = DM(x0)
        self.soln = DM([0.0]*self.nstep)
        self.cost_mat = MX(DM([[9, 25], [25, 0.26]]))
    def make_step(self, x0=None, steps=1):
        start=perf_counter()
        u = [MX.sym('u' + str(j)) for j in range(self.nstep)]
        cost_acc = 0
        g = []
        x = x0 if x0 is not None else self.x0
        for j in range(self.nstep):
            res = self.intfunc(x0=x, p=u[j])
            x = res['xf']
            cost_acc += 100*cos(x[1]) + x[3]**2 + (5*x[0])**2
            # g += [x[0], 10*exp(-1.5*fabs(x[3]))-fabs(u[j])-0.8]
            # g += [x[0], -2.5*fabs(x[2])-fabs(u[j])]
            # thing = vertcat(x[2]**2, u[j]**2)
            # g += [x[0], bilin(self.cost_mat, thing, thing)]
            g += [x[0], 9*x[2]**4 + 50*x[2]**2 * u[j]**2 + 0.26*u[j]**4]
            # g += [x[0], x[2], x[3]]



        nlp = {'x':vertcat(*u), 'f':cost_acc, 'g':vertcat(*g)}
        solver = casadi.nlpsol('solver', 'ipopt', nlp, self.solver_opts)
        # print(perf_counter()-start)
        self.soln = solver(
            x0=casadi.vertcat(self.soln[steps:], DM([self.soln[-1]]*steps)),
            # x0=self.soln,
            # lbx=-5,
            # ubx=5,
            # lbg=[-0.7, -0.75, -8]*self.nstep,
            # ubg=[0.7, 0.75, 8]*self.nstep,
            lbg=[-0.7, 0]*self.nstep,
            ubg=[0.7, 100]*self.nstep
        )['x']
        # print(self.soln['g'])
        # self.soln = self.soln['x']
        
        return self.soln[0]
    
    def get_full_path(self):
        return np.array(self.soln).flatten()


#%%
if __name__ == '__main__':
    if False:
        mpc = controller(tstep=0.1, thoriz=5)
        mpc.make_step(x0=DM([0,0.01,0,0]))
        print(list(mpc.get_full_path()))
        exit()
    mpc = controller(tstep=0.2, thoriz=1)
    x0 = DM([0,0.01,0,0])
    mpc.make_step(x0)
    sleep(2)
    record = []
    for i in range(100):
        start = perf_counter()
        u = mpc.make_step(x0)
        dt = perf_counter()-start
        print(str(np.round(np.array(x0).flatten(), 4)).ljust(55), np.format_float_positional(u, 5, trim='k').ljust(9), round(dt, 4))
        x0 = mpc.intfunc(x0=x0, p=u)['xf']
        record.append(float(u))
    print(record)

#%%