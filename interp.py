import numpy as np
import scipy as sp
import scipy.interpolate
powers = [0.0,0.050000,0.1000000,0.15000,0.200000,0.250000,0.3000000,0.3500000,0.356563,0.306563,0.256563,0.2065633,0.1565633,0.1065633,0.05656338,0.00656338,-0.04343661,-0.0379097,0.01209023,0.06209024,0.1120902,0.1620902,0.212090,0.262090,0.312090,0.309552,0.2595525,0.2095525,0.1595525,0.109552,0.059552,0.00955250,-0.04044749,-0.0904474,-0.0842599,-0.0342599,0.01574005,0.065740,0.1157400,0.1657400,0.2157400,0.2619045,0.255604,0.2056043,0.1556043,0.1056043,0.0556043,0.0056043,-0.04439568,-0.0943956,-0.144395,-0.1575401,-0.1223243,-0.0723243,-0.02232431,0.0276756,0.0776756,0.127675,0.1776756,0.2265245,0.2256796,0.18419,0.134196,0.0841967,0.03419677,-0.0158032,-0.0658032,-0.1158032,-0.165803,-0.215625,-0.2364476,-0.221426,-0.1714261,-0.1214261,-0.0714261,-0.02142619,0.0285738,0.0785738,0.1160172,0.125207,0.1241909,0.1165773,0.0740555,0.02405554,-0.0259444,-0.075944,-0.1259444,-0.175944,-0.2259444,-0.1759444,-0.1259444,-0.1244348,-0.174434,-0.224434,-0.1744348,-0.1244348,-0.0744348,-0.0244348,0.02556513,0.0714802,0.0214802,-0.028519,-0.0785197,-0.0929256,-0.04292560,0.00707439,0.0566756,0.00667566,-0.04332433,-0.0933243,-0.1433243,-0.193324,-0.147847,-0.0978472,-0.0478472,0.00215279,0.052152,0.1021527,0.1521527,0.1987697,0.2334949,0.2531078,0.257546,0.2484867,0.228416,0.2000600,0.1660317,0.1286506,0.0898593,0.0512064,0.01386833,-0.02130902,-0.0537592,-0.0831433,-0.1093003,-0.1322043,-0.15192,-0.168607,-0.182428,-0.1935982,-0.202334,-0.208857,-0.2133830,-0.2161156,-0.2172489,-0.2169629,-0.2154241,-0.212785,-0.2091876,-0.2047589,-0.199617,-0.1938693,-0.1876138,-0.180940,-0.1739304,-0.1666590,-0.1591938,-0.151596,-0.1439237,-0.1362253,-0.1285470,-0.1209296,-0.1134094,-0.1060184,-0.0987847,-0.0917328,-0.084883,-0.0782549,-0.0718613,-0.065714,-0.0598247,-0.05419797,-0.0488393,-0.043751,-0.0389360,-0.0343917,-0.03011661,-0.0261073,-0.0223594,-0.0188673,-0.01562460,-0.01262408,-0.00985799,-0.00731800,-0.0049953,-0.00288106,-0.000965723,0.000760132,0.00230608,0.003681770,0.00489677,0.00596062,0.006882736,0.0076723,0.00833850,0.00889002,0.00933548,0.00968316,0.00994107,0.01011689,0.0102179,0.01025139,0.01022381,0.01014160,0.0100107,0.00983704,0.00962572,0.00938183,0.00911005,0.00881476,0.00849999,0.00816950,0.00782676,0.0074749,0.00711689,0.006755321,0.00639259,0.00603086,0.00567206,0.00531791,0.00496992,0.00462943,0.00429758,0.003975355,0.003663589,0.0033629,0.003074046,0.002797259,0.00253292,0.00228126,0.002042394,0.001816362,0.00160,0.001402593,0.001214584,0.00103888,0.00087523,0.000723311,0.000582787,0.000453287,0.0003344164,0.0002257609,0.0001268919]
powers = np.array(powers)
t = np.arange(0, 12.15, 0.05)
print(len(t))
print(len(powers))
cs = sp.interpolate.CubicSpline(t, powers)
print(cs.x)
print(cs.c)
# print([float(cs(i)) for i in t])

import matplotlib.pyplot as plt
print(list((500_000/(powers*1000)).astype(int)))
plt.plot(t, powers, label = 'real')
plt.plot(t, [cs(i) for i in t], label = 'fit')
plt.legend()
plt.show()
