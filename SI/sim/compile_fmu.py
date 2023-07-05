from OMPython import OMCSessionZMQ
import os
os.environ.update({'OPENMODELICAHOME':'D:\\OpenModelica'})


omc = OMCSessionZMQ()
if omc.loadFile('DIPC.mo').startswith('false'):
  raise Exception('Modelica compilation failed: {}'.format(omc.sendExpression('getErrorString()')))
omc.sendExpression('setCommandLineOptions(generateSymbolicJacobian=true)')
omc.sendExpression('setCommandLineOptions(generateSymbolicLinearization=true)')
# omc.sendExpression('setCommandLineOptions(postOptModules=)')
omc.sendExpression('setCommandLineOptions(symjacdumpverbose=true)')



fmu_file = omc.sendExpression('translateModelFMU(DIPC)')
flag = omc.sendExpression('getErrorString()')
if not fmu_file.endswith('.fmu'): raise Exception('FMU generation failed: {}'.format(flag))
print("translateModelFMU warnings:\n{}".format(flag))