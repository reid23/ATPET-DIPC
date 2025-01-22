from Settings import Settings
from numpy.random import normal
import numpy as np
from enum import Enum

import os
hsl_avail = False
paths = ":".join([(linuxpath if (linuxpath:=os.environ.get('LD_LIBRARY_PATH')) is not None else ""),
                  (macospath if (macospath:=os.environ.get('DYLD_LIBRARY_PATH')) is not None else "")])
for folder in paths.split(":"):
    if len(folder)>1 and np.any(['libhsl' in j for j in os.listdir(folder)]):
        hsl_avail = True
        break

class IntegrationMethod(Enum):
    RungeKutta = 'rk'
    CVODES = 'cvodes'
    IDAS = 'idas'

class ModelParameters(metaclass=Settings):
    L_1 = 0.2
    L_2 = 0.14
    L_PEND = 0.30
    M_1 = 0.09
    M_2 = 0.045
    C_1 = 0.01
    C_2 = 0.01
    I_1 = 0.00035
    I_2 = 0.00041

class FittedModelParametersOld(metaclass=Settings):
    L_1 = 0.153351
    L_2 = 0.109039
    L_PEND = 0.3048
    M_1 = 0.146088
    M_2 = 0.166193
    C_1 = 0.0
    C_2 = 0.0
    I_1 = 0.00148645
    I_2 = 0.00177441

class FittedModelParameters(metaclass=Settings):
    L_1 = 0.139628
    L_2 = 0.133049
    L_PEND = 0.3048
    M_1 = 0.190044
    M_2 = 0.143263
    C_1 = 0.0
    C_2 = 0.0
    I_1 = 0.00267596
    I_2 = 0.0013376


# [0.153351, 0.109039, 0.3048, 0.146088, 0.166193, 0, 0, 0.00148645, 0.00177441]
# [0.152265, 0.103731, 0.3048, 0.150243, 0.1616, 0, 0, 0.0014427, 0.00161724]
# [0.139628, 0.133049, 0.3048, 0.190044, 0.143263, 0, 0, 0.00267596, 0.0013376]

class SimulatorSettings(metaclass=Settings):
    model_params: ModelParameters = ModelParameters
    dt: float = 0.001
    delay: float = 0.005
    noise = lambda: np.diag([0.1, 0.1, 1, 1, 1, 1])@normal(0.0, 0.00104694189, 6) # noise stdev taken from encoder datasheet
    integration_method = IntegrationMethod.RungeKutta
    A_MAX: float = 15


class HybridSimulatorSettings(metaclass=Settings):
    model_params = ModelParameters
    dt = 0.01
    integration_method = 'cvodes' # can be 'rk', 'cvodes', or 'idas'
    A_MAX = 10


class SolverIPOPT(metaclass=Settings):
    name = 'ipopt'
    opts = {'print_time': False, 'expand': False, 'ipopt': {'linear_solver': 'ma27' if hsl_avail else 'mumps', 'print_level': 0}}

class SolverWORHP(metaclass=Settings):
    name = 'worhp'
    opts = {'print_time': False, 'expand': False, 'worhp.NLPprint': -1}

class SolverFATROP(metaclass=Settings):
    name = 'fatrop'
    opts = {'structure_detection': 'auto', 'expand': False, 'debug': False, 'fatrop.print_level': 5} # 'equality' key must be passed by user!

class MPCSettings(metaclass=Settings):
    Q = np.diag([100, 10, 5, 1, 100, 50]) # state cost weight matrix
    R = np.diag([1]) # input cost weight matrix
    N = 20 # length of horizon
    p = ModelParameters
    dt = 0.025 # length of one timestep
    TRACK_LENGTH = 1.8 # in meters. x=0 is taken as center.
    CART_RAD_PER_M = 2*np.pi/0.16 # 80t gt2 pulley = 160mm/rotation = 0.16m per 2pi rad
    MAX_MOTOR_RPM = 1800 # reality is a bit higher but this is safe
    U_MAX = 15.0 # max acceleration (m/s^2)
    N_FIXED_U = 0 # number of timesteps of the linear controller to add on the end
    solver = SolverFATROP
    use_terminal_constraint = False
    recompute_lyap = False




