#%%
import sympy as sp
# %%
m, M, L, g, d = sp.symbols('m,M,L,g,d')
k = sp.symbols('k:4')
y = sp.symbols('\lambda_:4')
A = sp.Matrix([
    [0,1,0,0],
    [0,-d/M,-m*g/M,0],
    [0,0,0,1],
    [0,-d/(M*L),-(m+M)*g/(M*L),0]
])
B = sp.Matrix([
    [0],
    [1/M],
    [0],
    [1/(M*L)]
]);
K = sp.Matrix([
    [k[0], k[1], k[2], k[3]]
])
I = sp.eye(4)
# %%
M = A-B*K
k_result = sp.Matrix(list(sp.solve([
    (M-y[0]*I).det(),
    (M-y[1]*I).det(),
    (M-y[2]*I).det(),
    (M-y[3]*I).det()
], K).values()))
# %%
print(sp.latex(k_result))
# %%
