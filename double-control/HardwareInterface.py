from numpy._typing._generic_alias import NDArray
from eoms import get_eoms
from dynamics import to_casadi
import all_settings as settings
import casadi as ca
import numpy as np
from time import perf_counter, sleep
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import control as ct
from typing import Callable, Optional
from matplotlib.figure import Figure
from matplotlib.axes._axes import Axes
from matplotlib.animation import FuncAnimation
import struct
import serial
from controller import mpcController, mpcControllerForces
from swingupsim import SwingUpController


class HardwareInterface:
    def __init__(self, model_params: settings.ModelParameters, serial: serial.Serial):
        """initialize a Simulator object.

        Args:
            model_params (settings.ModelParameters): parameters for the dynamics model - lengths, masses, moments of inertia, etc.
            dt (float): simulator base time step. Choose to be approximately the sensor/actuator update frequency (much less than controller runtime)
            delay (float): extra time to add onto controller runtime, mimicking some latency in the control stack
            integration_method (settings.IntegrationMethod): which integration method to use. `rk` is typically much faster and is fine since we don't need sensitivities.
            noise (Callable[[], np.ndarray]): function that produces a 6-vector of noise each time it is called with no arguments. This is added to the state before it is passed into the controller.
            A_MAX (float): maximum acceleration acheivable by the motor. Controller outputs will be clipped by this value.
        """
        self.ser = serial
        self.p = list(model_params.values())
        self._eoms = to_casadi(get_eoms())
    def zero_encoders(self):
        while True:
            res = self._write_read(0.0)
            print(res)
            if np.abs(res[4])<0.02 and np.abs(res[6])<0.02:
                break
        self.ser.write(bytes([10]))
        self.ser.flush()
        res = np.array(struct.unpack("<Lffffff", self.ser.read(7*4)))
        print(f'state at time of zero: {res[1:]}')
        for i in range(5):
            res = self._write_read(0.0)
            print(f'next state: {res[1:]}')
            sleep(0.01)

    def run(self, controller: Callable[[float, np.ndarray[(6,), float]], float]):
        """run the simulation with a given controller. Each time the controller is run, we time how long it takes, and run the simulation for that amount of time before applying the input.

        Args:
            controller (Callable[[t, x], u]): function that gives control input from current time and system state (float, np.ndarray of shape (6,))
            x0 (np.ndarray | ca.DM): initial system state. Shape (6,).
            tf (float): final time (how long, in seconds, to run the simulation for)
        """

        ret = self._write_read(0.0)
        t0 = ret[0]
        prevt = t0
        self.history = []
        self.acc_hist = []
        try:
            while True:
                print(f'\rtime taken: {ret[0]-prevt:.3f}s', end='')
                prevt = ret[0]
                ret = self._write_read(acc:=controller(ret[0]-t0, ret[1:]))
                self.history.append(ret)
                self.acc_hist.append(acc)
        except:
            return
        finally:
            self.ser.write([5])
            self.ser.flush()
    def _write_read(self, acc: float):
        """set cart acceleration to `acc` meters per second^2 and get the system state.

        Args:
            acc (float): acceleration, in m/s^2

        Returns:
            tuple: (time (us), x, xdot, theta1, theta1 dot, theta2, theta2 dot)
        """
        # print(acc)
        self.ser.write(bytes([0]) + struct.pack(">f", acc*1000))
        self.ser.flush()
        res = np.array(struct.unpack("<Lffffff", self.ser.read(7*4)))
        res[0] /= 1e6
        res[1] /= 1000.0
        res[2] /= 1000.0
        # sleep(0.001)
        # print(res)
        return res


    @property
    def eoms(self):
        return self._eoms
    
    def linearContinuousMatrices(self, x: np.ndarray[(6,), float], u: np.ndarray[(1,), float]) -> tuple[np.ndarray[(6,6), float], np.ndarray[(6, 1), float]]:

        A = np.array(ca.Function('A', [self._eoms['x'], self._eoms['u'], self._eoms['p']], [ca.jacobian(self._eoms['ode'], self._eoms['x'])])(x, u, self.p))
        B = np.array(ca.Function('B', [self._eoms['x'], self._eoms['u'], self._eoms['p']], [ca.jacobian(self._eoms['ode'], self._eoms['u'])])(x, u, self.p))
        return A, B

if __name__ == '__main__':
    op_pt = np.array([0.0, 0.0, np.pi, 0.0, 0*np.pi, 0.0]), np.array([0.0])
    Q = np.diag([100, 10, 5, 1, 100, 50])
    R = np.diag([10])
    with serial.Serial('/dev/serial/by-id/usb-Teensyduino_USB_Serial_15749420-if00', baudrate=115200) as ser:
        hardware = HardwareInterface(settings.FittedModelParameters, ser)
        # input()
        A, B = hardware.linearContinuousMatrices(*op_pt)
        K, _, _ = ct.lqr(A, B, Q, R)
        print(K)
        mpc_controller = mpcControllerForces(hardware.eoms, A, B, **settings.MPCSettings, recompile=False)
        # hardware.zero_encoders()
        # mpc_controller.compile_and_reload()
        mpc = lambda t, x: mpc_controller.solve(x, np.array([[0.0, 0.0, np.pi, 0.0, 0*np.pi, 0.0, 0.0]]*settings.MPCSettings.N))
        mpc(0, op_pt[0])
        mpc(0, np.zeros(6))
        swingup = SwingUpController(settings.FittedModelParameters, hardware.eoms, 10, 0.001, 10)
        thing_to_do = [
            ('trajectory_cl.json', [0.0, 0.0, np.pi, 0.0, 0.0, 0.0, 0.0]),     # go to up-up
            ('trajectory2.json', [0.0, 0.0, 0.0, 0.0, np.pi, 0.0, 0.0]),    # go to down-up
            ('trajectory3.json', [0.0, 0.0, np.pi, 0.0, np.pi, 0.0, 0.0]),  # go to up-down
        ]
        controller = swingup.make_mpc_controller(mpc_controller, *thing_to_do[0])
        def fix_angle(x):
            x[2] = x[2] - 2*np.pi * np.floor((x[2]+np.pi)/(2*np.pi))
            x[4] = x[2] - 2*np.pi * np.floor((x[4]+np.pi)/(2*np.pi))
            return x
        # print(mpc(0, np.zeros(6)))
        input()
        # hardware.run(mpc)
        t = 0.75
        x_err = 0.075
        th1_offset = x_err*(t)/7
        th2_offset = x_err*(1-t)/50
        th1_offset = 0.0014
        th2_offset = 0.0011
        offset = np.array([0., 0., th1_offset, 0., th2_offset, 0.])
        def cope_factor(x):
            x[2] += th1_offset*(np.cos(x[2]+np.pi)+1)/2
            x[4] += th2_offset*(np.cos(x[4]+np.pi)+1)/2
            return x
        hardware.run(lambda t, x: mpc(t, cope_factor(x)))
        # hardware.run(controller)
    hist = np.vstack(hardware.history)
    fig, ax = plt.subplots(4)
    for i in range(6):
        ax[int(i/2)].plot(hist[:, 0] - hist[0, 0], hist[:, 1+i], label=['pos', 'vel'][i%2])
        ax[int(i/2)].plot(np.array(list(range(swingup.x_traj.shape[1])))*swingup.dt, swingup.x_traj[i, :], linestyle='dashed', label=['pos plan', 'vel plan'][i%2])
    ax[3].plot(hist[:, 0] - hist[0, 0], u_hist:=np.array(hardware.acc_hist))
    ax[3].plot(np.array(list(range(swingup.x_traj.shape[1]-1)))*swingup.dt, swingup.u_traj.flatten(), linestyle='dashed')
    ax[0].legend()
    ax[1].legend()
    ax[2].legend()
    plt.show(block=False)
    breakpoint()

        

