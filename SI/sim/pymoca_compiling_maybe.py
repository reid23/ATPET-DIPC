
#%%
import os
os.environ.update({'OPENMODELICAHOME':'D:\\OpenModelica'})
import pymoca.parser
import pymoca.backends.sympy.generator as generator
# import pylab as pl
import sympy
import control
sympy.init_printing()
import numpy as np
import casadi as ca
# from pylab import plot, hold, legend
with open('DIPC.mo', 'r') as f:
    modelica_source = f.read()

ast = pymoca.parser.parse(modelica_source)
ast.classes['DIPC'].symbols.keys()

src_code = generator.generate(ast, 'DIPC')
print(src_code)

# model = transfer_model('.', 'DIPC', {'generateSymbolicJacobian':True, 
#                                      'generateSymbolicLinearization':True, 
#                                      'symjacdumpverbose':True})
#%%
dae = model.dae_residual_function
X0 = model.states[0].start