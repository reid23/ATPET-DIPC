#%%
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import scipy as sp

#%%
df = pd.read_csv('pulley2.csv')
df.head()
# %%
for i in list(df.columns):
    df[i] = df[i][10:-10]
    if i[-1]=='t': continue
    df[i] = np.convolve(df[i], np.ones(5)/5, 'same')

#%%

plt.plot(df['1.7t'], df['1.7x'], label='1.7x')
plt.plot(df['1.7t'][1:], (np.diff(df['1.7x'])/np.diff(df['1.7t'])), label='1.7kg $\dot{x}$')

plt.plot(df['1.6t'], df['1.6x'], label='1.6x')
plt.plot(df['1.6t'][1:], (np.diff(df['1.6x'])/np.diff(df['1.6t'])), label='1.6kg $\dot{x}$')

plt.plot(df['1.5t'], df['1.5x'], label='1.5x')
plt.plot(df['1.5t'][1:], (np.diff(df['1.5x'])/np.diff(df['1.5t'])), label='1.5kg $\dot{x}$')

plt.plot(df['1.4t'], df['1.4x'], label='1.4x')
plt.plot(df['1.4t'][1:], (np.diff(df['1.4x'])/np.diff(df['1.4t'])), label='1.4kg $\dot{x}$')

plt.plot(df['1.3t'], df['1.3x'], label='1.3x')
plt.plot(df['1.3t'][1:], (np.diff(df['1.3x'])/np.diff(df['1.3t'])), label='1.3kg $\dot{x}$')

plt.plot(df['1.2t'], df['1.2x'], label='1.2x')
plt.plot(df['1.2t'][1:], (np.diff(df['1.2x'])/np.diff(df['1.2t'])), label='1.2kg $\dot{x}$')

plt.legend()
plt.xlabel('Time (s)')
plt.ylabel('Position (m)/Velocity (m/s)')
plt.title('Cart Position and Velocity vs. Time for Varying Hanging Masses')
plt.show()

# %%
plt.plot(df['1.7x'][1:], (np.diff(df['1.7x'])/np.diff(df['1.7t'])), label='1.7kg')
plt.plot(df['1.6x'][1:], (np.diff(df['1.6x'])/np.diff(df['1.6t'])), label='1.6kg')
plt.plot(df['1.5x'][1:], (np.diff(df['1.5x'])/np.diff(df['1.5t'])), label='1.5kg')
plt.plot(df['1.4x'][1:], (np.diff(df['1.4x'])/np.diff(df['1.4t'])), label='1.4kg')
plt.plot(df['1.3x'][1:], (np.diff(df['1.3x'])/np.diff(df['1.3t'])), label='1.3kg')
plt.plot(df['1.2x'][1:], (np.diff(df['1.2x'])/np.diff(df['1.2t'])), label='1.2kg')

plt.xlabel('Position (m)')
plt.ylabel('Velocity (m/s)')
plt.title('Cart Velocity vs. Position for Varying Hanging Masses')
plt.legend()
plt.show()

#%%


# t>0.65
# v>0
mask = list(np.logical_and((np.diff(df[f'{i[:-1]}x'])/np.diff(df[f'{i[:-1]}t']))>0.1, df[f'{i[:-1]}x'][1:]<0.7) for i in df.columns)
t = [np.array(df[f'1.{i+2}t'][1:][mask[i]]) for i in range(6)]
x = [np.array(df[f'1.{i+2}x'][1:][mask[i]]) for i in range(6)]
xdot = [np.diff(x[i])/np.diff(t[i]) for i in range(6)]
xddot = [np.diff(np.convolve(np.convolve(xdot[i], np.ones(10)/10, mode='same'), np.ones(10)/10, mode='same'))/np.diff(t[i])[1:] for i in range(6)]
# convolutions are smoothing so acceleration isn't all over the place

# x = list(filter(lambda x: not x is None, [np.array(df[f'{i[:-1]}x'][1:][np.logical_and((np.diff(df[f'{i[:-1]}x'])/np.diff(df[f'{i[:-1]}t']))>0.1, df[f'{i[:-1]}x'][1:]<0.7)]) if i[-1]=='x' else None for i in list(df.columns)]))

fig, ax = plt.subplots(6, 1, sharex=True)
for i in range(len(ax)):
    ax[i].set_title(list(df.columns)[i*2])
    ax[i].plot(t[i], x[i], label = 'pos')
    ax[i].plot(t[i][1:], xdot[i], label = 'vel')
    ax[i].plot(t[i][2:], xddot[i], label = 'acc')
    ax[i].legend()

plt.show()

#%%
# original declaration:
# def cost(c1, c2, c3, c4, c5, m2):
for i in range(6):
    x[i], xdot[i], xddot[i] = np.convolve(x[i], np.ones(10)/10, 'same'), np.convolve(xdot[i], np.ones(10)/10, 'same'), np.convolve(xddot[i], np.ones(10)/10, 'same'), 
def cost(c):
    """cost function for scipy optimization

    Args:
        c1 (float): viscous friction coefficient (N/(m/s))
        c2 (float): sin component of friction amplitude/2 (N)
        c3 (float): sin component of friction period (m)
        c4 (float): sin component of friction phase shift (m)
        c5 (float): coulomb (constant) friction coefficient (N)
        m2 (float): effective mass of cart (Kg)

    Returns:
        _type_: _description_
    """
    c1, c2, c3, c4, c5, m2 = c
    total = 0
    for i in range(6):
        for j in range(len(xddot[i])):
            m1=((i+2)*0.1)+1
            # total += (m1*9.8 - c5 - xdot[i][j]*(c1+c2*np.sin(2*np.pi*(x[i][j]-c4)/c3)) - m1*xddot[i][j] - m2*xddot[i][j])**2
            total += (m1*9.8 - c5 - xdot[i][j]*c1 - m1*xddot[i][j] - m2*xddot[i][j])**2
    return total

c0 = [0.5, 0.25, 0.06, 0, 0.5, 0.15]

#%%

# print(sp.optimize.minimize(cost, c0, bounds=[(0, 30), (0.5, 20), (0.05, 0.12), (0, 0.13), (0, 30), (0, 2)]))
# print(sp.optimize.minimize(cost, c0, bounds=[(0, 30), (0, 1000), (0, 0.12), (0, 0.13), (0, 30), (0, 2)]))
# print(sp.optimize.minimize(cost, c0))
# %%
