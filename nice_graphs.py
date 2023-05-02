import matplotlib.pyplot as plt
import numpy as np

# mpc
# with open('final_data_for_plotting_mpc.txt') as f:
#     data = np.array(eval(f.read()))[5000:]
# t = np.cumsum(data[:, 0]+0.00000035)
# x, th, dx, dth, u = data[:, (1,2,4,5,7)].T

# LQR
with open('LQR_demo_data_up.txt') as f:
    data = np.array(eval(f.read()))[1088:2120]#[1800:]
t = data[:, 0]/1_000_000_000
x, th, dx, dth, u = data[:, (2,3,5,6,7)].T

fig, ax = plt.subplots(3, 1, sharex=True)

ax[0].set_title('Cart Position and Velocity')
ax[0].plot(t, x, label='$x$ (m)')
ax[0].plot(t, dx, label='$\dot x$ (m/s)')

ax[1].set_title('Pendulum Angle and Angular Velocity')
ax[1].plot(t, (th)%(2*np.pi), label='$\\theta$ (rad)')
# ax[1].plot(t, th, label='$\\theta$ (rad)')
ax[1].plot(t, dth, label='$\dot \\theta$ (rad/s)')

ax[2].set_title('Control Input (Cart Acceleration)')
ax[2].plot(t, u, label='$u$ (m/s^2)')

ax[0].legend()
ax[1].legend()
ax[2].legend()

fig.suptitle('Pendulum State over Time: LQR Up')

# plt.legend()
plt.xlabel('Time (s)')
plt.show()
