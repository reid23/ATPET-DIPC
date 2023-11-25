#%%
import numpy as np
import casadi as ca
import forcespro
import forcespro.nlp
import matplotlib.pyplot as plt
#%%
nsteps = 20
tstep = 0.1

dimU = 1
dimX = 4
dimZ = dimU+dimX

solverName = "forces_solver"

model = forcespro.nlp.SymbolicModel(nsteps)

model.N = nsteps # not required if already specified in initializer
model.nvar = 5   # number of stage variables
model.neq = 4    # number of equality constraints
model.nh = 1     # number of nonlinear inequality constraints
# model.npar = 0   # number of runtime parameters

# for i in range(0, nsteps):
#     model.lbidx[i] = range(0, dimZ)
#     model.ubidx[i] = range(0, dimZ)

def continuous_dynamics(y, f, l=0.220451, c=0.139185):
    return ca.vertcat(
        y[2],
        y[3],
        f,
        -c*y[3] + (-9.8*ca.sin(y[1])-f*ca.cos(y[1]))/l
    )

model.continuous_dynamics = continuous_dynamics
model.E = np.concatenate([np.zeros((dimX, dimU)), np.eye(dimX)], axis=1)

model.objective =  lambda z: 0.1*(z[1]**2)
model.objectiveN = lambda x: 100*ca.cos(x[2]) + (1/5)*x[4]**2 + (5*x[1])**2

model.ineq = lambda z: 9*z[3]**4 + 50*z[3]**2 * z[0]**2 + 0.26*z[0]**4
model.hu = [ 0.7]*nsteps
model.hl = [-0.7]*nsteps

model.lbidx = range(1, 2)
model.ubidx = range(1, 2)

codeoptions = forcespro.CodeOptions()
codeoptions.maxit = 1000  # Maximum number of iterations
codeoptions.printlevel = 0  # Use printlevel = 2 to print progress (but not for timings)
codeoptions.optlevel = 3  # 0 no optimization, 1 optimize for size, 2 optimize for speed, 3 optimize for size & speed
codeoptions.nlp.integrator.Ts = tstep
codeoptions.nlp.integrator.nodes = 2
codeoptions.nlp.integrator.type = 'ERK4'
# codeoptions.solvemethod = 'SQP_NLP'
# codeoptions.sqp_nlp.rti = 1
# codeoptions.sqp_nlp.maxSQPit = 1
# codeoptions.nlp.hessian_approximation = 'gauss-newton'
codeoptions.nlp.stack_parambounds = True


solver = model.generate_solver(codeoptions)
#%%
# Set run-time parameters
problem = {}
problem["lb"] = [-0.8]*nsteps
problem["ub"] =  [0.8]*nsteps
problem["xinit"] = np.array([0, np.pi/8, 0, 0])

sol, exitflag, info = solver.solve(problem)
