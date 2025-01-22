#%%
from numpy._typing._generic_alias import NDArray
from eoms import get_eoms
from dynamics import to_casadi
import all_settings as settings
import casadi as ca
import numpy as np
from time import perf_counter
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import control as ct
from typing import Callable, Optional
from matplotlib.figure import Figure
from matplotlib.axes._axes import Axes
from matplotlib.animation import FuncAnimation
#%%

class Simulator:
    def __init__(self, model_params: settings.ModelParameters, dt: float, delay: float, integration_method: settings.IntegrationMethod, noise: Callable[[], np.ndarray[(6,), float]], A_MAX: float):
        """initialize a Simulator object.

        Args:
            model_params (settings.ModelParameters): parameters for the dynamics model - lengths, masses, moments of inertia, etc.
            dt (float): simulator base time step. Choose to be approximately the sensor/actuator update frequency (much less than controller runtime)
            delay (float): extra time to add onto controller runtime, mimicking some latency in the control stack
            integration_method (settings.IntegrationMethod): which integration method to use. `rk` is typically much faster and is fine since we don't need sensitivities.
            noise (Callable[[], np.ndarray]): function that produces a 6-vector of noise each time it is called with no arguments. This is added to the state before it is passed into the controller.
            A_MAX (float): maximum acceleration acheivable by the motor. Controller outputs will be clipped by this value.
        """
        self.dt = dt
        self.delay = delay
        self.noise = noise
        self._eoms = to_casadi(get_eoms())
        if ca.__version__ == '3.5.5':
            self._eoms_silly = {
                'x': self._eoms['x'],
                'p': self._eoms['u'],
                'ode': ca.substitute(self.eoms['ode'], self.eoms['p'], list(model_params.values())),
            }
            self.intfunc_bad = ca.integrator('integrator', integration_method.value, self._eoms_silly, {'t0': 0.0, 'tf': dt})
            self.intfunc = lambda x0, u, p: self.intfunc_bad(x0=x0, p=u)
        else:
            self.intfunc = ca.integrator('integrator', integration_method.value, self._eoms, 0.0, dt)
        self.labels = ['$x$', '$\\dot x$', '$\\theta_1$', '$\\dot \\theta_1$', '$\\theta_2$', '$\\dot \\theta_2$']
        self.model_params = model_params
        self.p = list(self.model_params.values())
        self.A_MAX = A_MAX
    def run(self, observer, controller: Callable[[float, np.ndarray[(6,), float]], float], x0: np.ndarray, tf: float):
        """run the simulation with a given controller. Each time the controller is run, we time how long it takes, and run the simulation for that amount of time before applying the input.

        Args:
            controller (Callable[[t, x], u]): function that gives control input from current time and system state (float, np.ndarray of shape (6,))
            x0 (np.ndarray | ca.DM): initial system state. Shape (6,).
            tf (float): final time (how long, in seconds, to run the simulation for)
        """
        self.x = [ca.DM(x0)]
        self.u = [[0.0]]
        self.xhat = []
        self.solve_times = []
        print()
        while len(self.u)<=int(tf/self.dt):
            u_prev = self.u[-1]
            t = len(self.u)*self.dt
            x = np.array(self.x[-1] + self.noise()).flatten()
            start_time = perf_counter()
            u = np.clip(np.array(controller(t, x)).flatten(), -self.A_MAX, self.A_MAX)
            end_time = perf_counter()
            for i in range(np.ceil(((end_time-start_time) + self.delay)/self.dt).astype(int)):
                self.x.append(self.intfunc(x0=self.x[-1], u=u_prev, p=self.p)['xf'])
                self.xhat.append(observer(np.array(self.x[-1] + self.noise()).reshape((6, 1)), u_prev, self.dt))
                if i!=0: self.u.append(u_prev)
            self.u.append(u)
            self.solve_times.append(end_time-start_time)
            print(f'\rSimulating: t={t:.3f}, solve took {end_time-start_time:.4f}s', end='')
        print('\nSimulation finished!')
            
    @classmethod
    def rot(self, th: float) -> np.ndarray[(2, 2), float]:
        """make 2d rotation matrix that rotates by `th`

        Args:
            th (float): angle to rotate by

        Returns:
            ndarray[(2, 2), float64]: 2x2 rotation matrix
        """
        return np.array([
            [np.cos(th), -np.sin(th)],
            [np.sin(th),  np.cos(th)]
        ])
    def get_xy(self, x: float, th1: float, th2: float, l1: float, l2: float) -> np.ndarray[(2, 3), float]:
        """get xy positions of cart, mid joint, and end of pendulum, from system configuration and parameters.

        Returns:
            ndarray[(2, 3), float]: array of x/y coordinates
        """
        cart = np.array([[x], [0]])
        pend_1 = cart + Simulator.rot(th1)@np.array([[0],[-l1]])
        pend_2 = pend_1 + Simulator.rot(th1+th2)@np.array([[0], [-l2]])
        return np.concatenate([cart, pend_1, pend_2], axis=1)   
    def plot_anim(self, fig: Optional[Figure] = None, ax: Optional[Axes]=None, filename: Optional[str] = None) -> tuple[FuncAnimation, Figure, Axes]:
        """create animation of the simulation's results

        Args:
            fig (Optional[Figure], optional): figure to use; if not given, creates a new figure. Defaults to None.
            ax (Optional[Axes], optional): axes to use; if not given, creates new axes. Defaults to None.

        Returns:
            tuple[FuncAnimation, Figure, Axes]: Animaiton object, figure, and axes containing the animation.
        """
        x = np.array(ca.horzcat(*self.x))
        u = np.array(ca.horzcat(*self.u))

        num_frames = len(u[0])
        if fig is None or ax is None:
            fig, ax = plt.subplots()
        mat, = ax.plot(*self.get_xy(*x[::2, 0], self.p[0], self.p[1]), marker='o')
        times = np.arange(0, num_frames*self.dt, self.dt)
        assert len(times) == num_frames

        ax.axhline(0, linestyle='dashed')

        frame_mult = ((1/30)/self.dt)
        def animate(i: int):
            mat.set_data(self.get_xy(*x[::2, int(i*frame_mult)%num_frames], self.p[0], self.p[1]))
            return mat
        ax.axis([-1.0,1.0,-0.75,0.75])
        anim = animation.FuncAnimation(fig, animate, interval=1000/30, frames=int(num_frames*self.dt*30))
        if filename is not None:
            anim.save(filename, fps=30)
        return anim, fig, ax
    def plot_graph(self, fig: Optional[Figure] = None, axs: Optional[list[Axes]] = None) -> tuple[Figure, Axes]:
        """plot all states vs. time in one graph.

        Args:
            fig (Optional[Figure], optional): Figure to use; if not given, creates a new figure. Defaults to None.
            ax (Optional[Axes], optional): Axes to use; if not given, creates new axes. Defaults to None.

        Returns:
            tuple[Figure, Axes]: Figure and Axes containing plot.
        """
        x = np.array(ca.horzcat(*self.x))
        u = np.array(ca.horzcat(*self.u))

        num_frames = len(u[0])
        if fig is None or axs is None:
            fig, axs = plt.subplots(4, sharex='all')
        times = np.array(list(range(num_frames)))*self.dt
        for idx, dim in enumerate(x):
            axs[int(idx/2)].plot(times, dim, label=self.labels[idx])
            axs[int(idx/2)].legend()
        axs[3].plot(times, u[0], label="u")
        axs[3].legend()
        axs[3].set_xlabel('Time (s)')
        axs[3].set_ylabel('Control (m/s^2)')
        axs[2].set_ylabel('rad, rad/s')
        axs[1].set_ylabel('rad, rad/s')
        axs[0].set_ylabel('m, m/s')

        fig.set_figheight(12)
        fig.set_figwidth(8)
        return fig, axs
    
    @property
    def eoms(self):
        return self._eoms
    
    def linearContinuousMatrices(self, x: np.ndarray[(6,), float], u: np.ndarray[(1,), float]) -> tuple[np.ndarray[(6,6), float], np.ndarray[(6, 1), float]]:

        A = np.array(ca.Function('A', [self._eoms['x'], self._eoms['u'], self._eoms['p']], [ca.jacobian(self._eoms['ode'], self._eoms['x'])])(x, u, self.p))
        B = np.array(ca.Function('B', [self._eoms['x'], self._eoms['u'], self._eoms['p']], [ca.jacobian(self._eoms['ode'], self._eoms['u'])])(x, u, self.p))
        return A, B


#%%
if __name__ == '__main__':
    sim = Simulator(**settings.SimulatorSettingsLQR)
    op_pt = np.array([0.0, 0.0, np.pi, 0.0, 0.0, 0.0]), np.array([0.0])
    A, B = sim.linearContinuousMatrices(*op_pt)

    Q = np.diag([100, 10, 5, 1, 100, 50])
    R = np.diag([10])
    K, S, E = ct.lqr(A, B, Q, R)
    

    sim.run(lambda t, x: -K@(x-op_pt[0]), [1.0, 0, np.pi*0.97, 0, 0, 0], 5.0)
    
    # fig, axs = plt.subplots(2)
    # anim, _, _ = sim.plot_anim(fig=fig, ax=axs[0])
    # sim.plot_graph(fig=fig, ax=axs[1])
    
    sim.plot_graph()
    anim, _, _ = sim.plot_anim()

    plt.show()

# %%