from simulator import Simulator
from controller import mpcController, mpcControllerForces
import all_settings as settings
import numpy as np
import matplotlib.pyplot as plt
import control as ct
from swingupsim import SwingUpController


# sim = Simulator(settings.SimulatorSettings.model_params, 0.001, 0.0, settings.IntegrationMethod.RungeKutta, lambda: np.random.normal(0.0, ), 5.0)
sim = Simulator(**settings.SimulatorSettings.withopt(model_params=settings.FittedModelParametersOld))
op_pt = np.array([0.0, 0.0, np.pi, 0.0, 0.0, 0.0]), np.array([0.0])
A, B = sim.linearContinuousMatrices(*op_pt)

Q = np.diag([100, 10, 5, 1, 100, 50])
R = np.diag([10])

N = settings.MPCSettings.N
K, _, _ = ct.lqr(A, B, Q, R)
lqr = lambda t, x: -K@(x-op_pt[0])
mpc_controller = mpcController(sim.eoms, A, B, **settings.MPCSettings.withopt(p=settings.FittedModelParametersOld))
#* this line slightly increases speed by generating and compiling C code for the solver
#* but it takes a couple extra seconds to do so only use if you really need it
# mpc_controller.compile_and_reload()
mpc_controller.load_solver()
mpc = lambda t, x: mpc_controller.solve(x, np.array([[0.0, 0.0, np.pi, 0.0, 0.0, 0.0, 0.0]]*N))
print(mpc(0, op_pt[0]))
# print(mpc_controller.warmstart)
# input()


swingup = SwingUpController(settings.FittedModelParametersOld, sim.eoms, 10, 0.01, 10)
# swingup.solve_full_trajectory(np.zeros(6), op_pt[0], 4, filename="trajectory5.json")
# controller = swingup.make_controller("trajectory.json")
thing_to_do = [
    ('trajectory.json', [0.0, 0.0, np.pi, 0.0, 0.0, 0.0, 0.0]),     # go to up-up
    ('trajectory2.json', [0.0, 0.0, 0.0, 0.0, np.pi, 0.0, 0.0]),    # go to down-up
    ('trajectory3.json', [0.0, 0.0, np.pi, 0.0, np.pi, 0.0, 0.0]),  # go to up-down
]
controller = swingup.make_mpc_controller(mpc_controller, *thing_to_do[0])
# print(controller(0, [0, 0, 0, 0, 0, 0])) # get the warmstart going
print(mpc(0, op_pt[0]))
x0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
for i in range(50):
    print(mpc(0, np.array(x0)))
    if mpc_controller.solver.stats()['success']: break
else:
    raise RuntimeError(f'initial state {x0} not feasible.')
print(mpc_controller.solver.stats())
# breakpoint()
# sim.run(mpc, x0, 5.0)
mpc_controller.set_observer_state(x0)
# sim.run(mpc_controller.run_observer, mpc, x0, 5.0)
sim.run(mpc_controller.run_observer, controller, x0, 5.0)

plt.plot(sim.solve_times)
plt.show()

fig, ax = plt.subplots(3, 2)
for i in range(6):
    ax.flatten()[i].plot(np.array(sim.xhat)[:, 6+i])
    # ax.flatten()[i].plot(np.array(sim.x)[:, i], label='x')
plt.show()
graphfig, graphaxs = sim.plot_graph()
graphaxs[0].set_title("Swing-Up Maneuver 3 with New Model Parameters")
tgrid = np.arange(0, swingup.dt*np.max(swingup.x_traj.shape), swingup.dt)
tgrid = np.array(list(range(swingup.x_traj.shape[1])))*swingup.dt
utgrid = np.array(list(range(np.max(swingup.u_traj.shape))))*swingup.dt
if False:
    graphaxs[0].plot(tgrid, swingup.x_traj[0, :], color='tab:blue', linestyle='dashed', label='$\\bar x$')
    graphaxs[0].plot(tgrid, swingup.x_traj[1, :], color='tab:orange', linestyle='dashed', label='$\\dot \\bar x$')
    graphaxs[1].plot(tgrid, swingup.x_traj[2, :], color='tab:blue', linestyle='dashed', label='$\\bar \\theta_1$')
    graphaxs[1].plot(tgrid, swingup.x_traj[3, :], color='tab:orange', linestyle='dashed', label='$\\dot \\bar \\theta_1$')
    graphaxs[2].plot(tgrid, swingup.x_traj[4, :], color='tab:blue', linestyle='dashed', label='$\\bar \\theta_2$')
    graphaxs[2].plot(tgrid, swingup.x_traj[5, :], color='tab:orange', linestyle='dashed', label='$\\dot \\bar \\theta_2$')
    graphaxs[3].plot(tgrid[:-1], swingup.u_traj.flatten(), color='tab:blue', linestyle='dashed', label='$\\bar u$')

xhat = np.array(sim.xhat)[:, 0:6, 0].T
tgrid = np.linspace(0, 5, xhat.shape[1])
graphaxs[0].plot(tgrid, xhat[0, :], color='tab:blue', linestyle='dashed', label='$\\hat x$')
graphaxs[0].plot(tgrid, xhat[1, :], color='tab:orange', linestyle='dashed', label='$\\dot \\hat x$')
graphaxs[1].plot(tgrid, xhat[2, :], color='tab:blue', linestyle='dashed', label='$\\hat \\theta_1$')
graphaxs[1].plot(tgrid, xhat[3, :], color='tab:orange', linestyle='dashed', label='$\\dot \\hat \\theta_1$')
graphaxs[2].plot(tgrid, xhat[4, :], color='tab:blue', linestyle='dashed', label='$\\hat \\theta_2$')
graphaxs[2].plot(tgrid, xhat[5, :], color='tab:orange', linestyle='dashed', label='$\\dot \\hat \\theta_2$')
# graphaxs[3].plot(tgrid[:-1], swingup.u_traj.flatten(), color='tab:blue', linestyle='dashed', label='$\\bar u$')
for ax in graphaxs: ax.legend()
anim, animfig, animax = sim.plot_anim()
animax.set_title("Swing-Up Maneuver 3 with New Model Parameters")


filename = input("save animation? (filename or empty for no): ")
if len(filename)>0:
    anim.save(filename+'.gif')
    graphfig.savefig(filename+'.png')
plt.show(block=False)

import casadi as ca
import json
x = np.array(ca.horzcat(*sim.x))
u = np.array(ca.horzcat(*sim.u))
print(x.shape, u.shape)
with open('trajectory_cl.json', 'w') as f:
    json.dump({'trajectory': x.tolist(), 'controls': u[:, :-1].tolist()}, f)
input('enter to close')