from SI.sim.double_pendulum_model import dipc_model
from SI.data_collection import Pendulum

eigs = np.array([
    [-3.50], 
    [-3.51],
    [-3.52],
    [-3.53],
    [-3.54],
    [-3.55],
])

Q = np.diag([100, 10, 5, 1, 100, 50])
R = np.diag([10])

model = dipc_model().linearize(op_point=[0,sp.pi,sp.pi,0,0,0]).lambdify()
model.constants = []
model.construct_PP().construct_LQR()
print(model.K)