import numpy as np
import casadi as ca
import forcespro
import forcespro.nlp
import matplotlib.pyplot as plt

nsteps = 20
tstep = 0.1

dimU = 1
dimX = 4
dimZ = dimU+dimX

solverName = "forces_solver"
guessIncumbent = True

model = forcespro.nlp.SymbolicModel(nsteps)
model.nvar = dimZ # 4d state
model.neq = dimX  # 4d state + 1d input



model.xinitidx = range(dimU, dimZ)

model.lbidx[0] = range(0, dimU)
model.ubidx[0] = range(0, dimU)

for i in range(1, nsteps):
    model.lbidx[i] = range(0, dimZ)
    model.ubidx[i] = range(0, dimZ)

def continuous_dynamics(y, f, l=0.220451, c=0.139185):
    return ca.vertcat(
        y[2],
        y[3],
        f,
        -c*y[3] + (-9.8*ca.sin(y[1])-f*ca.cos(y[1]))/l
    )

model.continuous_dynamics = continuous_dynamics
model.E = np.concatenate([np.zeros((dimX, dimU)), np.eye(dimX)], axis=1)
# model.ineq = lambda x, u: 9*x[2]**4 + 50*x[2]**2 * u[0]**2 + 0.26*u[0]**4
# model.hu = 0.7
# model.hl = -0.7

model.objective =  lambda z: 0.1*(z[1]**2)
model.objectiveN = lambda x: 100*ca.cos(x[:, 1]) + (1/5)*x[:, 3]**2 + (5*x[:, 0])**2

# Define outputs
outputs = [('CartPos', range(0, model.N), 0),
           ('CartVel', range(0, model.N), 1),
           ('PendAng', range(0, model.N), 2),
           ('PendAngVel', range(0, model.N), 3)]


# Set code-generation options
codeoptions = forcespro.CodeOptions()
codeoptions.maxit = 200  # Maximum number of iterations
codeoptions.printlevel = 0  # Use printlevel = 2 to print progress (but not for timings)
codeoptions.optlevel = 3  # 0 no optimization, 1 optimize for size, 2 optimize for speed, 3 optimize for size & speed
codeoptions.nlp.integrator.Ts = tstep
codeoptions.nlp.integrator.nodes = 5
codeoptions.nlp.integrator.type = 'ERK4'
codeoptions.solvemethod = 'SQP_NLP'
codeoptions.sqp_nlp.rti = 1
codeoptions.sqp_nlp.maxSQPit = 1
codeoptions.nlp.hessian_approximation = 'gauss-newton'

# Generate SQP_NLP solver
solver = model.generate_solver(codeoptions, outputs)

# Set run-time parameters
problem = {}
problem[f"lb{1:03d}"] = [0]
problem[f"ub{1:03d}"] = [1]
for s in range(1, nsteps-1):
    problem[f"lb{s+1:03d}"] = np.concatenate([[-1], [-0.8, -0.3, -float('inf'), -20]])
    problem[f"ub{s+1:03d}"] = np.concatenate([[1],  [ 0.8,  0.3,  float('inf'),  20]])
problem[f"lb{nsteps:03d}"] = np.concatenate([[-1],   [-0.8, -0.3, -float('inf'), -20]])
problem[f"ub{nsteps:03d}"] = np.concatenate([[1],   [ 0.8,  0.3,  float('inf'),  20]])

# problem["x0"] = np.tile(np.zeros((dimZ, 1)), (nsteps, 1))
problem["xinit"] = np.array([0, np.pi/8, 0, 0])

sol, exitflag, info = solver.solve(problem)

# plot
time = np.arange(0, 4.96, codeoptions.nlp.integrator.Ts)

plt.step(time, 0.05236 * (2 * sol["CartPos"] - 1))
plt.grid('both')
plt.title('Tail deflection angle (rad)')
plt.tight_layout()

plt.figure()
plt.subplot(3, 1, 1)
plt.grid('both')
plt.title('Angle of attack (rad)')
plt.plot(time, sol["CartVel"], 'r')
plt.subplot(3, 1, 2)
plt.grid('both')
plt.title('Pitch angle (rad)')
plt.plot(time, sol["PendAng"], 'b')
plt.subplot(3, 1, 3)
plt.grid('both')
plt.title('Pitch angle rate (rad/sec)')
plt.plot(time, sol["PendAngVel"], 'b')
plt.tight_layout()
plt.show()

