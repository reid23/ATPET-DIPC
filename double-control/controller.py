import numpy as np
import control as ct
import casadi as ca
import scipy as sp
import polytope as pt
import matplotlib.pyplot as plt
import matplotlib
import os
import scipy as sp

if ca.__version__ == '3.5.5':
    import forcespro
    from forcespro import CodeOptions
    from forcespro.nlp.symbolicModel import SymbolicModel

hsl_avail = False
paths = ":".join([(linuxpath if (linuxpath:=os.environ.get('LD_LIBRARY_PATH')) is not None else ""),
                  (macospath if (macospath:=os.environ.get('DYLD_LIBRARY_PATH')) is not None else "")])
for folder in paths.split(":"):
    if len(folder)>1 and np.any(['libhsl' in j for j in os.listdir(folder)]):
        hsl_avail = True
        break


class mpcController:
    def __init__(self, eoms, A, B, Q, R, N, p, dt, TRACK_LENGTH, CART_RAD_PER_M, MAX_MOTOR_RPM, U_MAX, N_FIXED_U, solver, use_terminal_constraint=False, recompute_lyap=False):
        self.eoms = eoms
        self.A, self.B, _, _, self.dt = sp.signal.cont2discrete((A, B, np.zeros_like(A), np.zeros_like(B)), 0.03)
        # self.A = A*dt + np.eye(6)
        # self.B = B*dt
        self.Q = Q
        self.R = R
        self.N = N
        self.B_d = np.eye(6) if True else np.zeros((6, 6))
        self.C_d = np.eye(6) if True else np.zeros((6, 6))
        self.N_FIXED_U = N_FIXED_U
        self.nlpsolver = solver

        self.TRACK_LENGTH = TRACK_LENGTH
        self.CART_RAD_PER_M = CART_RAD_PER_M
        self.MAX_MOTOR_RPM = MAX_MOTOR_RPM
        self.U_MAX = U_MAX

        self.K, self.P, _ = ct.dlqr(self.A, self.B, self.Q, self.R)
        self.K_c, self.P, _ = ct.lqr(A, B, Q, R)
        self.P = self.P/dt
        self.K = -self.K # because ct.dlqr assumes u=-Kx
        self.K_c = -self.K_c # because ct.dlqr assumes u=-Kx
        self.dhat = np.zeros((6, 1))
        self.xhat = np.zeros((6, 1))

        val = ca.SX.sym('val')

        self.A_cl_time = np.linalg.matrix_power(self.A+self.B@self.K, 50)

        sgnval = ca.if_else(val<0, -1, 1)
        absval = ca.if_else(val<0, -val, val)

        saturation = ca.Function('saturation', [val], [sgnval*U_MAX*ca.tanh((absval/U_MAX)**5)**0.2])
        print(np.linalg.eigvals(self.P))
        print(p)
        ode = {
            'x': eoms['x'],
            'u': eoms['u'],
            'ode': ca.substitute(eoms['ode'], eoms['p'], list(p.values())),
        }
        # _x0=ca.MX.sym('x', 6)
        # _u=ca.MX.sym('u')
        # self.discrete_dynamics = ca.Function('dynamics', [_x0, _u], [ca.integrator('dynamics', 'rk', ode, 0, dt)(x0=_x0, u=_u)['xf']])
        f = ca.Function('f', [ode['x'], ode['u']], [ode['ode']])
        self.A_func = ca.Function('A', [ode['x'], ode['u']], [ca.jacobian(ode['ode'], ode['x'])])
        self.B_func = ca.Function('B', [ode['x'], ode['u']], [ca.jacobian(ode['ode'], ode['u'])])
        
        self.p = ca.SX.sym('p', 7, self.N+1+6)
        d = self.p[6:7, self.N+1:self.N+7].T
        dt_var = ca.SX.sym('dt_var')
        d_dynamics = ode['x'] + f(ode['x']+f(ode['x'], ode['u'])*(dt_var/2), ode['u'])*(dt_var/2) + ca.DM(self.B_d)@d
        self.discrete_dynamics_with_dt = ca.Function('dynamics', [ode['x'], ode['u'], d, dt_var], [d_dynamics])
        self.F_fn = ca.Function('F', [ode['x'], ode['u'], d, dt_var], [ca.jacobian(d_dynamics, ode['x'])])
        self.discrete_dynamics = ca.Function('dynamics', [ode['x'], ode['u'], d], [ode['x'] + f(ode['x']+f(ode['x'], ode['u'])*(dt/2), ode['u'])*(dt/2) + ca.DM(self.B_d)@d])
        self.opt_vars = ca.SX.sym('x', 7, N) # at each time, we have the state (6 vars) and the input (1 var)
        self.qref = self.p[0:6, 0:self.N]
        self.uref = self.p[6:7, 0:self.N]
        self.q0 = self.p[0:6, self.N]
        self.q = self.opt_vars[0:6, :]
        self.u = self.opt_vars[6:7, :]
        self.x = self.opt_vars[0, :]
        self.xdot = self.opt_vars[1, :]
        self.th1 = self.opt_vars[2, :]
        self.th1dot = self.opt_vars[3, :]
        self.th2 = self.opt_vars[4, :]
        self.th2dot = self.opt_vars[5, :]
        # self.q = ca.horzcat(self.q0, self.q[:, 0:])

        self.warmstart = None

        # Find maximal control invariant set using the LQR controller defined above
        self.A_cl = self.A + self.B @ self.K


        m_per_rev = (1/self.CART_RAD_PER_M)*2*np.pi
        X = pt.Polytope(np.array([
                                  [1.0, 0, 0, 0, 0, 0],
                                  [0, 1.0, 0, 0, 0, 0],
                                  [-1, 0, 0, 0, 0, 0],
                                  [0, -1, 0, 0, 0, 0]
                                ]),
                        np.array([
                                  [TRACK_LENGTH/2],
                                  [(MAX_MOTOR_RPM/60)*m_per_rev],
                                  [TRACK_LENGTH/2],
                                  [(MAX_MOTOR_RPM/60)*m_per_rev]
                                ]))

        U = pt.Polytope(np.array([1, -1]).reshape(2,1),np.array([self.U_MAX, self.U_MAX]).reshape(2,1))
        S = X.intersect(pt.Polytope(U.A @ self.K, U.b))
        S = X

        op_pt = ca.DM([0, 0, ca.pi, 0, 0, 0])

        
        self.term_set = self.Oinf(S, self.A_cl)
        self.A_term = self.term_set.A
        self.b_term = self.term_set.b
        print(self.term_set.volume)
        def sillyplot(x, y):
            fig, ax = plt.subplots()
            pt.Polytope(self.A_term[:, (x, y)], self.b_term).plot(ax)
            ax.autoscale_view()
            plt.show(block=False)

        # breakpoint()
        
        ode2 = {
            'x': ode['x'],
            'ode': ca.substitute(ode['ode'], ode['u'], saturation(ca.DM(self.K_c)@(ode['x']-op_pt)))
        }
        # self.P = sp.linalg.solve_discrete_lyapunov(self.A+self.B@self.K, self.Q)*dt
        V = (ode2['x'] - op_pt).T@ca.DM(self.P)@(ode2['x'] - op_pt)
        Vdot = ca.jacobian(V, ode2['x'])@ode2['ode']
        Vfn = ca.Function('Vfn', [ode2['x']], [V])
        Vdotfn = ca.Function('Vdotfn', [ode2['x']], [Vdot])
        nlp = {
            'x': ode2['x'],
            'f': V,
            'g': Vdot+100*ca.exp(-ca.sumsqr(200*(ode2['x']-op_pt))),
        }
        if recompute_lyap:
            s = ca.nlpsol('lyapsolver', 'ipopt', nlp, {'ipopt.linear_solver': 'ma27', 'ipopt.print_level': 0, 'print_time': False, 'expand': True})
            optima = []
            for i in range(10000):
                print(f'solve {i}')
                res = s(x0=op_pt + ca.DM(np.random.normal(0, 2, 6)), lbg=[0.0], ubg=[ca.inf])
                optima.append(float(res['f']))
            optima = np.array(optima)
            best = np.min(optima[optima>0.1])
            print(res['x'], res['f'])
            print(f"BEST: {best}")
        else:
            best = 121.28868696949601


        def makeplot(x, y, xscale=0.02, yscale=0.02, n_pts=200):
            vals, vecs = np.linalg.eig(self.P)
            vals = np.ones(7)
            vecs = np.hstack([np.array(res['x']).reshape((6, 1)), np.eye(6)])
            vecs[:, 0] /= np.linalg.norm(vecs[:, 0])
            vout = np.zeros((n_pts, n_pts))
            vdotout = np.zeros((n_pts, n_pts))
            pts = np.zeros((n_pts, n_pts, 2))
            for idx, i in enumerate(xgrid:=np.linspace(-vals[x]*xscale, vals[x]*xscale, n_pts)):
                for jdx, j in enumerate(ygrid:=np.linspace(-vals[y]*yscale, vals[y]*yscale, n_pts)):
                    pt = op_pt + ca.DM(vecs[:, (x, y)]@np.array([[i],[j]]))
                    pts[idx, jdx, :] = [i, j]
                    vout[idx, jdx] = Vfn(pt)
                    vdotout[idx, jdx] = Vdotfn(pt)

            trans = matplotlib.scale.SymmetricalLogTransform(10, 10, 1)
            vdotout = trans.transform(vdotout.flatten()).reshape(vdotout.shape)
            plt.contour(pts[:, :, 0], pts[:, :, 1], vdotout, levels=20)
            cbar = plt.colorbar()
            plt.contour(pts[:, :, 0], pts[:, :, 1], vout, levels=[best], colors='red')
            plt.contour(pts[:, :, 0], pts[:, :, 1], vdotout, levels=[0], colors='tab:orange')
            # cbar.set_ticks(trans.inverted().transform(cbar.get_ticks()))
            # cbar.vmin = np.min(cbar.get_ticks())
            # cbar.vmax = np.max(cbar.get_ticks())
            # breakpoint()

            # xs = pts[:, :, 0].flatten()
            # ys = pts[:, :, 1].flatten()
            # vout = vout.flatten()
            # vdotout = vdotout.flatten()
            # plt.scatter(xs[vdotout<=0], ys[vdotout<=0], c=vdotout[vdotout<=0])
            plt.xlim((-xscale*vals[x], xscale*vals[x]))
            plt.ylim((-yscale*vals[y], yscale*vals[y]))

            plt.show(block=False)
        # makeplot(0, 1, 0.00001, 0.0001)
        
        # breakpoint()

        #* these will store our constraint expressions (g) and bounds (lbg, ubg)
        self.g = []
        self.lbg = []
        self.ubg = []
        self.g_equalities = []
        #* table to keep track of indices of each constraint
        self.gtable = dict()
        self.glen = 0
        self._add_constraint(
            'initial state',
            g=self.q[:, 0]-self.q0,
            lbg=ca.DM([0.0]*6),
            ubg=ca.DM([0.0]*6),
            equality=True,
        )
        self._add_constraint(
            'dynamics',
            self.q[:, 1:] - self.discrete_dynamics.map(self.N-1)(self.q[:, :-1], self.u[:, :-1], d),
            lbg = ca.DM([0.0]*6*(self.N-1)),
            ubg = ca.DM([0.0]*6*(self.N-1)),
            equality=True,
        )

        # for i in range(self.N-1):
        #     self._add_constraint(
        #         f'dynamics{i}',
        #         g=self.q[:, i+1] - self.discrete_dynamics(self.q[:, i], self.u[:, i]),
        #         lbg = ca.DM([0.0]*6),
        #         ubg = ca.DM([0.0]*6),
        #         equality=True
        #     )
        self._add_constraint(
            "track length",
            g = self.x,
            lbg = ca.DM([-self.TRACK_LENGTH/2]*self.N),
            ubg = ca.DM([self.TRACK_LENGTH/2]*self.N),
        )

        
        self._add_constraint(
            'cart_speed',
            g=self.xdot,
            lbg = ca.DM([-(self.MAX_MOTOR_RPM/60)*m_per_rev]*self.N),
            ubg = ca.DM([(self.MAX_MOTOR_RPM/60)*m_per_rev]*self.N),
        )
        # https://www.omc-stepperonline.com/index.php?route=product/product/get_file&file=1342/23HS32-4004S_Torque_Curve.pdf
        # https://www.desmos.com/calculator/9uq2d20hcc
        # torque_curve = ca.interpolant('torque_curve', 'bspline', [0, 90, 210, 300, 390, 510, 600], [185, 185, 182.8, 178.4, 173.4, 150, 133.4])
        # yk, this is basically flat
        # we don't need to be that precise here
        # it'll also make things solve faster if we don't do a funny nonlinear motor limit
        # and i don't want to compute the torque required
        # rad_per_sec2rpm = 60/(2*np.pi)

        self._add_constraint(
            "input bound",
            g = self.u,
            lbg = ca.DM([-self.U_MAX]*self.N),
            ubg = ca.DM([self.U_MAX]*self.N),
        )
        # self._add_constraint(
        #     "end inputs",
        #     g=ca.vertcat(ca.DM.zeros(self.N-(N_FIXED_U+1)), (self.u[-(N_FIXED_U+1):]-ca.DM(self.K_c)@self.q[:, -(N_FIXED_U+1):]).T),
        #     lbg=ca.DM([0.0]*self.N),
        #     ubg=ca.DM([0.0]*self.N),
        #     equality=True,
        # )

        self._interleave_constraints(['track length', 'cart_speed', 'input bound'], 6+1)

        if use_terminal_constraint:
            self._add_constraint(
                'terminal constraint',
                g=ca.DM(self.A_term) @ (self.q[:, -1]-op_pt),
                lbg=ca.DM(np.tile(np.array(-ca.inf), self.b_term.shape)),
                ubg=ca.DM(self.b_term),
            )
            self._add_constraint(
                'terminal lyapunov',
                g=ca.bilin(self.P, self.q[:, -1]-op_pt),
                lbg=ca.DM([-ca.inf]),
                ubg=best,
            )
            # breakpoint()

        self.cost = ca.bilin(self.p[0:6, self.N+1:self.N+7], self.q[:, -1] - self.qref[:, -1])
        for i in range(N-1):
            self.cost += ca.bilin(self.Q, self.q[:, i] - self.qref[:, i])
            self.cost += ca.bilin(self.R, self.u[:, i] - self.uref[:, i])


        # breakpoint()
        # breakpoint()
        self.g = ca.vertcat(*self.g)
        self.lbg = ca.vertcat(*self.lbg)
        self.ubg = ca.vertcat(*self.ubg)
        self.g_equalities = np.array(ca.vertcat(*self.g_equalities)).astype(bool).flatten().tolist()

        self.nlp = {
            'x': ca.vec(self.opt_vars),
            'f': self.cost,
            'g': self.g,
            'p': self.p,
        }
        # print(self.p.shape)
        
        self.options = dict(**self.nlpsolver.opts, equality=self.g_equalities)
        self.solver = ca.nlpsol('solver', self.nlpsolver.name, self.nlp, self.options)
        # self.solver.print_options()
    def compile_and_reload(self, gcc_opt_flag='-Ofast'):
        print('generating C...')
        self.solver.generate_dependencies('mpc.c')
        print('compiling solver...')
        os.system(f'gcc -fPIC {gcc_opt_flag} -shared mpc.c -o mpc.so')
        print('loading solver...')
        self.solver = ca.nlpsol('solver', self.nlpsolver.name, 'mpc.so', self.options)
    def load_solver(self):
        self.solver = ca.nlpsol('solver', self.nlpsolver.name, 'mpc.so', self.options)
    def _add_constraint(self, name, g, lbg, ubg, equality=False):
        """utility funciton to add a constraint and keep track of all the indices

        Args:
            name (str): name for this constraint (human-readable)
            g (casadi symbolic expression): expression to be constrained
            lbg (float): lower bound for g
            ubg (float): upper bound for g
        """
        g = ca.vec(g)
        self.gtable[name] = (self.glen, self.glen+g.shape[0])
        self.glen += g.shape[0]
        self.g.append(g)
        self.lbg.append(lbg)
        self.ubg.append(ubg)
        self.g_equalities.append(ca.DM([equality]*g.shape[0]))
    def _interleave_constraints(self, constraints, n_dyn):

        # breakpoint()
        # self.g, self.lbg, self.ubg, self.g_equalities = [ca.vec(ca.vertcat(ca.horzcat(*g[:self.N]), ca.horzcat(*g[self.N:]).T)) for g in (self.g, self.lbg, self.ubg, self.g_equalities)]
        # self.g_equalities = np.array(self.g_equalities).flatten().astype(bool).tolist()
        # breakpoint()
        # return
        # dyn_equalities = np.array(self.g_equalities[:self.N])
        # other_equalities = np.array(self.g_equalities[self.N:])
        n_cons = len(constraints)
        self.g = ca.vertsplit(ca.vertcat(*self.g))
        self.lbg = ca.vertsplit(ca.vertcat(*self.lbg))
        self.ubg = ca.vertsplit(ca.vertcat(*self.ubg))
        self.g_equalities = ca.vertsplit(ca.vertcat(*self.g_equalities))
        
        # g_new = intersperse(self.g[6:((self.N-1)*self.n_dyn + 6)])

        gcp = self.g[:]
        lbgcp = self.lbg[:]
        ubgcp = self.ubg[:]
        g_equalities_cp = self.g_equalities[:]
        # n_dyn = nx + nu
        for cixs, c in enumerate(constraints):
            # print(cixs)
            c_idxs = self.gtable[c]
            constr = gcp[c_idxs[0]:c_idxs[1]]
            lbg = lbgcp[c_idxs[0]:c_idxs[1]]
            ubg = ubgcp[c_idxs[0]:c_idxs[1]]
            is_equality = g_equalities_cp[c_idxs[0]:c_idxs[1]]
            for i in range(self.N):
                extra = n_dyn + cixs-1
                if i==(self.N-1):
                    extra = 0
                # print(f"i: {i}, len: {len(constr)}")
                self.g.insert((i)*(n_dyn + cixs)+6+extra, constr[i])
                self.lbg.insert((i)*(n_dyn + cixs)+6+extra, lbg[i])
                self.ubg.insert((i)*(n_dyn + cixs)+6+extra, ubg[i])
                self.g_equalities.insert((i)*(n_dyn + cixs)+6+extra, is_equality[i])
        self.g = self.g[:self.glen]
        self.lbg = self.lbg[:self.glen]
        self.ubg = self.ubg[:self.glen]
        self.g_equalities = self.g_equalities[:self.glen]
        # breakpoint()

        end_idx = 6+n_dyn-1
        self.g = self.g[6:end_idx] + self.g[0:6] + self.g[end_idx:]
        self.lbg = self.lbg[6:end_idx] + self.lbg[0:6] + self.lbg[end_idx:]
        self.ubg = self.ubg[6:end_idx] + self.ubg[0:6] + self.ubg[end_idx:]
        self.g_equalities = self.g_equalities[6:end_idx] + self.g_equalities[0:6] + self.g_equalities[end_idx:]
        # for g in (self.g, self.lbg, self.ubg, self.g_equalities):
        #     g.pop((6+n_cons)*(self.N-1) + 6)
        # self.glen -= 1
        # breakpoint()

    def Oinf(self, Xset, A, Wset=pt.Polytope()):
        """utility function to find maximal invariant set

        Args:
            Xset (Polytope): feasible states
            A (np.array): dynamics matrix (Oinf will be invariant using this A)
            Wset (Polytope): feasible inputs
        """
        Omega = Xset
        Omegap = self.precursor(Omega, A).intersect(Omega)
        while not Omegap == Omega:
            Omega = Omegap
            if not pt.is_empty(Wset):
                Omegap = pt.reduce(self.precursor(Omega, A, Uset=Wset).intersect(Omega))
            else:
                Omegap = pt.reduce(self.precursor(Omega, A).intersect(Omega))
        return Omegap

    def precursor(self, Xset, A, Uset=pt.Polytope(), B=np.array([])):
        """utility function to find precursor term_set

        Args:
            Xset (Polytope): current term_set
            A (np.array): dynamics matrix
            Uset (Polytope): feasible inputs
            B (Np.array): input effect (column) vector
        """
        if not B.any():
            return pt.Polytope(Xset.A @ A, Xset.b)
        else:
            tmp  = self.minkowski_sum(Xset, pt.extreme(Uset) @ -B.T)
        return pt.Polytope(tmp.A @ A, tmp.b)

    def minkowski_sum(self, X, Y):
        """utility function to find minkowski sum of two sets

        Args:
            X (Polytope): first set
            Y (Polytope): second set
        """
        V_sum = []
        if isinstance(X, pt.Polytope):
            V1 = pt.extreme(X)
        else:
            # assuming vertices are in (N x d) shape. N # of vertices, d dimension
            V1 = X

        if isinstance(Y, pt.Polytope):
            V2 = pt.extreme(Y)
        else:
            V2 = Y

        for i in range(V1.shape[0]):
            for j in range(V2.shape[0]):
                V_sum.append(V1[i,:] + V2[j,:])
        return pt.qhull(np.asarray(V_sum))

    def set_observer_state(self, xhat):
        self.xhat = np.array(xhat).reshape((6, 1))

    # def run_observer(self, y_m, u, dt):
    #     Q = np.diag([100, 100, 1, 1, 1, 1,   1, 1, 1, 1, 1, 1])
    #     R = np.diag([1000, 1000, 1000, 1000, 1000, 1000])
    #     F = self.F_fn(self.xhat, u, self.dhat, dt)
    #     H = np.hstack([np.eye(6), np.zeros((6, 6))])
    #     xhat_next = self.discrete_dynamics_with_dt(self.xhat, u, self.dhat, dt)
    #     P_next = F@self.P_old@F.T + Q
    #     S = H@P_next@H.T + R
    #     K = P_next@H.T@np.linalg.inv(S)
    #     xhat_fixed = xhat_next + K@(y_m - self.xhat_next)
    #     P_fixed = (np.eye(12) - K@H)@P_next

    #     self.P_old = P_fixed
    #     self.xhat = xhat_fixed

    def run_observer(self, y_m, u, dt):
        

        u = np.array(u).reshape((1, 1))
        A = np.array(self.A_func(self.xhat, u))
        B = np.array(self.B_func(self.xhat, u))
        C = np.eye(6)
        D = np.zeros((6, 1))
        A, B, C, D, _ = sp.signal.cont2discrete((A, B, C, D), dt)
        B_d = np.array(self.B_d)
        C_d = np.array(self.C_d)

        Q = np.diag([1, 1, 1, 0.1, 1, 0.1,   1, 1, 1, 0.1, 1, 0.1])
        R = np.diag([1000, 1000, 10, 10, 10, 10])
        
        Aprime = np.block([[A, B_d], [np.zeros((6, 6)), np.eye(6)]])
        Bprime = np.block([[B],[np.zeros((6, 1))]])
        Cprime = np.block([[C, C_d]])
        Lprime, _, _ = ct.dlqr(Aprime.T, Cprime.T, Q, R, method='scipy')
        # print(Lprime.shape)
        L_x, L_d = Lprime[:, 0:6].T, Lprime[:, 6:12].T
        # Lprime = ct.place(Aprime.T, Cprime.T, 10*np.array([0.01, 0.02, 0.03, 0.04, 0.05, 0.06, -0.01, -0.02, -0.03, -0.04, -0.05, -0.06]))
        # print(Lprime.shape)
        # print(self.xhat.shape, self.dhat.shape, "shapes")
        xhatPrime = np.block([[self.xhat], [self.dhat]])
        self.xhat = np.array(self.discrete_dynamics_with_dt(self.xhat, u, self.dhat, dt) + B_d@self.dhat - L_x@(Cprime@xhatPrime - y_m))
        self.dhat -= L_d@(Cprime@xhatPrime - y_m)
        # xhatPrimeNext = Aprime@xhatPrime + Bprime@u - Lprime.T@(Cprime@xhatPrime - y_m)
        # self.xhat = xhatPrimeNext[0:6]
        # self.dhat = xhatPrimeNext[6:12]
        # print("hats:", self.xhat, self.dhat)
        return np.vstack([self.xhat, self.dhat])

    def solve(self, y_m, trajectory, P=None):
        """crunch the numbers.

        Args:
            left (np.ndarray): location of left cones. shape (N, 2).
            right (np.ndarray): location of right cones. shape (N, 2).

        Returns:
            dict: result of solve. keys 'z' (states), 'u' (controls), 't' (timestamps)
        """
        x0 = self.xhat
        # print(trajectory.shape, x0.shape)
        # xf, uf = trajectory[-1, 0:6], trajectory[-1, 6:7] 
        # P = np.pad((P/self.dt), ((0, 0), (0, 1)), constant_values=0)
        if P is None: P = self.P
        P = np.hstack((P/self.dt, self.dhat))
        # print(P.shape)
        if self.warmstart is None:
            self.soln = self.solver(
                lbg=self.lbg,
                ubg=self.ubg,
                p=ca.vertcat(ca.DM(trajectory), ca.DM(list(x0)+[0.0]).T, ca.DM(P)).T,
            )
        else:
            self.soln = self.solver(
                x0=self.warmstart,
                lbg=self.lbg,
                ubg=self.ubg,
                p=ca.vertcat(ca.DM(trajectory), ca.DM(list(x0)+[0.0]).T, ca.DM(P)).T,
            )
        self.warmstart = ca.DM(np.array(self.soln['x']))
        self.soln['x'] = np.array(ca.reshape(self.soln['x'], (7, self.N)))

        self.res=dict()
        self.res['q'] = np.array(self.soln['x'][0:6, :])
        self.res['u'] = np.array(self.soln['x'][6:7, :])
        return self.res['u'][0, 0]    
#%%

class mpcControllerForces:
    def __init__(self, eoms, A, B, Q, R, N, p, dt, TRACK_LENGTH, CART_RAD_PER_M, MAX_MOTOR_RPM, U_MAX, solver, recompile=False):
        self.eoms = {
            'x': eoms['x'],
            'u': eoms['u'],
            'ode': ca.substitute(eoms['ode'], eoms['p'], list(p.values())),
        }
        self.A = A*dt + np.eye(6)
        self.B = B*dt
        self.Q = ca.DM(Q)
        self.R = ca.DM(R)
        self.N = N
        self.nstate = 6
        self.nin = 1
        self.npar = 7
        self.dt = dt

        self.warmstart = None

        self.TRACK_LENGTH = TRACK_LENGTH
        self.CART_RAD_PER_M = CART_RAD_PER_M
        self.MAX_MOTOR_RPM = MAX_MOTOR_RPM
        self.U_MAX = U_MAX

        self.K, self.P, _ = ct.dlqr(self.A, self.B, self.Q, self.R)
        self.K = -self.K # because ct.dlqr assumes u=-Kx
        self.P = ca.DM(self.P)

        self.n_errors = 0

        if recompile:
            self.model = SymbolicModel(self.N)
            self.model.nvar = self.nin+self.nstate
            self.model.neq = self.nstate
            self.model.npar = self.npar
            continuous_dynamics = ca.Function('continuous_dynamics', [self.eoms['x'], self.eoms['u']], [self.eoms['ode']])
            # fix_angle = ca.Function('fix_angle', [state:=ca.SX.sym('state', 6)], [ca.vertcat(state[0], state[1], 2*ca.sin(state[2]/2), state[3], 2*ca.sin(state[4]/2), state[5])])
            self.model.eq = lambda z, p: forcespro.nlp.integrate(continuous_dynamics, z[self.nin:self.nin+self.nstate], z[0:self.nin], integrator=forcespro.nlp.integrators.RK4, stepsize=self.dt)
            self.model.E = np.hstack((np.zeros((self.nstate, self.nin)), np.eye(self.nstate)))
            
            self.model.lb = np.array([-self.U_MAX, -self.TRACK_LENGTH/2, -(self.MAX_MOTOR_RPM/60)*(2*np.pi/self.CART_RAD_PER_M), -np.inf, -np.inf, -np.inf, -np.inf])
            self.model.ub = np.array([ self.U_MAX,  self.TRACK_LENGTH/2,  (self.MAX_MOTOR_RPM/60)*(2*np.pi/self.CART_RAD_PER_M),  np.inf,  np.inf,  np.inf,  np.inf])
            self.model.xinitidx = range(1, 7)
            self.model.objective = lambda z, p: float(print(z.shape) is None) + (z[1:]-p[1:]).T@self.Q@(z[1:]-p[1:]) + (z[0]-p[0]).T@self.R@(z[0]-p[0])
            self.model.objectiveN = lambda z, p: (z[1:]-p[1:]).T@self.P@(z[1:]-p[1:]) # terminal cost
            self.codeoptions = CodeOptions("dipc_mpc")
            self.codeoptions.solvemethod = 'PDIP_NLP'
            self.codeoptions.server = 'https://forces.embotech.com'
            self.codeoptions.cleanup = 0
            self.codeoptions.printlevel = 1
            self.codeoptions.optlevel = 3
            self.codeoptions.overwrite = 1
            self.m = self.model.generate_solver(self.codeoptions)
        else:
            self.m = forcespro.nlp.Solver.from_directory("dipc_mpc")
    def solve(self, x0, trajectory):
        traj = np.vstack([trajectory.T[6:7, :], trajectory.T[0:6, :]])
        # print(traj)
        x0 = np.array(x0).flatten()[:, np.newaxis]
        outputs, exitflag, info = self.m.solve({
            'x0': traj if self.warmstart is None else self.warmstart,
            'xinit': x0,
            'all_parameters': traj
        })
        if exitflag!=1:
            self.n_errors += 1
        else:
            self.n_errors = 0
        if np.abs(outputs['x10'][1])>self.TRACK_LENGTH:
            print("UH OH!")
            if self.n_errors > 3: raise RuntimeError
        self.warmstart = np.vstack(list(outputs.values())).T
        print(self.warmstart.shape)
        return outputs['x01'][0]

#%%


# time_lookahead = 1
# frequency = 10
# N = int(time_lookahead * frequency)

# nstate = 6  # x, y, theta and their velocities
# npar = DriveParameters.num_parameters()

# if __name__ == '__main__':
# 	model = SymbolicModel(N)
# 	model.nvar = nin + nstate
# 	model.neq = nstate
# 	model.npar = npar

# 	integrator_stepsize = 0.1

# 	continuous_dynamics = lambda x, u, p: DriveParameters.from_array(p).continuous_dynamics(RobotState.from_array(casadi.vertcat(u, x)))
# 	model.eq = lambda z, p: forcespro.nlp.integrate(continuous_dynamics, z[nin:nin + nstate], z[0:nin], integrator=forcespro.nlp.integrators.RK4, stepsize=integrator_stepsize)
# 	model.E = np.hstack((np.zeros((nstate, nin)), np.eye(nstate)))

# 	model.lb = np.concatenate((np.ones(nin) * -1, np.ones(nstate) * -np.inf))
# 	model.ub = np.concatenate((np.ones(nin) * 1, np.ones(nstate) * np.inf))
# 	model.xinitidx = range(nin, nstate + nin)

# 	codeoptions = CodeOptions("mecanum_mpc")

# 	codeoptions.platform = 'AARCH-Cortex-A53'  # For the Control Hub's ARM processor

# 	codeoptions.solvemethod = 'PDIP_NLP'  # Nonlinear Primal-Dual Interior-Point method
# 	# codeoptions.server = 'https://forces-6-0-1.embotech.com'
# 	codeoptions.server = 'https://forces.embotech.com/'
# 	codeoptions.cleanup = 0

# 	codeoptions.printlevel = 0  # todo: should be 0 when deploying to hardware
# 	codeoptions.optlevel = 3  # todo: should be 3 when deploying to hardware

# 	codeoptions.overwrite = 1

# # 	codeoptions.sse = -1
# # 	codeoptions.avx = -1

# 	m = model.generate_solver(codeoptions)