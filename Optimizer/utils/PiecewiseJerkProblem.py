from scipy.sparse import csc_matrix
import osqp
import numpy as np
import matplotlib.pyplot as plt

class PiecewiseJerkProblem:
    def __init__(self, num_knots, delta_s, x_init):
        self.x_init = x_init
        self.num_knots = num_knots
        self.delta_s = delta_s
        self.w_x, self.w_dx, self.w_ddx, self.w_dddx = (1, 1, 1, 1)
        self.x_bounds, self.dx_bounds, self.ddx_bounds, self.dddx_bound = [], [], [], []
        self.w_x_ref = [0 for i in range(num_knots)]
        self.w_end_state = (0, 0, 0)
        self.x_ref = []
        self.end_state_ref = []
        self.scale_factor = (1, 1, 1)
        self.has_x_ref = False
        self.has_end_state_ref = False
        
    def set_w(self, w):
        self.w_x, self.w_dx, self.w_ddx, self.w_dddx = w
        
    def set_scale_factor(self, scale_factor):
        self.scale_factor = scale_factor
        
    def set_x_bounds(self, x_bounds):
        self.x_bounds = x_bounds
        
    def set_dx_bounds(self, dx_bounds):
        self.dx_bounds = dx_bounds
        
    def set_ddx_bounds(self, ddx_bounds):
        self.ddx_bounds = ddx_bounds
        
    def set_dddx_bound(self, dddx_bound):
        self.dddx_bound = dddx_bound
         
    def set_x_ref(self, w_x_ref, x_ref=None):
        self.w_x_ref = w_x_ref
        if x_ref is not None:
            self.x_ref = x_ref
            self.has_x_ref = True
            
    def set_end_state_ref(self, w_end_state, end_state_ref):
        self.w_end_state = w_end_state
        self.end_state_ref = end_state_ref
        self.has_end_state_ref = True
    
    def calculateKernel(self):
        n = self.num_knots
        num_variables = 3 * n
        columns = [[] for i in range(num_variables)]
        
        for i in range(self.num_knots - 1):
            columns[i].append((i, (self.w_x + self.w_x_ref[i]) / self.scale_factor[0]**2))
            columns[n+i].append((n+i, self.w_dx / self.scale_factor[1]**2))
            columns[2*n+i].append((2*n+i, (self.w_ddx + 2. * self.w_dddx / self.delta_s**2) / self.scale_factor[2]**2))
        
        columns[n-1].append((i, (self.w_x + self.w_x_ref[n-1] + self.w_end_state[0]) / self.scale_factor[0]**2))
        columns[2*n-1].append((2*n-1, (self.w_dx + self.w_end_state[1]) / self.scale_factor[1]**2))
        columns[2*n][-1] = ((2*n, (self.w_ddx + self.w_dddx / self.delta_s**2) / self.scale_factor[2]))
        columns[3*n-1].append((3*n-1, (self.w_ddx + self.w_dddx / self.delta_s**2 
                                       + self.w_end_state[2]) / self.scale_factor[2]))
        
        for i in range(self.num_knots - 1):
            columns[2*n+i].append((2*n+i+1, (-2*self.w_dddx / self.delta_s**2)/self.scale_factor[2]))
             
        p_indptr = []
        p_indices = []
        p_data = []
        ind_p = 0
        for i in range(num_variables):
            p_indptr.append(ind_p)
            for first, second in columns[i]:
                p_indices.append(first)
                p_data.append(second)
                ind_p += 1
        p_indptr.append(ind_p)
        
        return p_data, p_indices, p_indptr
    
    def calculateAffineConstraint(self):
        n = self.num_knots
        num_variables = 3 * n
        num_constraints = num_variables + 3 * n
        
        lower_bounds = [0 for i in range(num_constraints)]
        upper_bounds = [0 for i in range(num_constraints)]
        
        variables = [[] for i in range(num_variables)]
        
        constraint_index = 0
        
        for i in range(num_variables):
#             set x, x', x'' bounds
            if i < n:
                variables[i].append((constraint_index, 1))
                lower_bounds[constraint_index] = self.x_bounds[i][0] * self.scale_factor[0]
                upper_bounds[constraint_index] = self.x_bounds[i][1] * self.scale_factor[0]
            elif i < 2*n:
                variables[i].append((constraint_index, 1))
                lower_bounds[constraint_index] = self.dx_bounds[i-n][0] * self.scale_factor[1]
                upper_bounds[constraint_index] = self.dx_bounds[i-n][1] * self.scale_factor[1]
            else:
                variables[i].append((constraint_index, 1))
                lower_bounds[constraint_index] = self.ddx_bounds[i-2*n][0] * self.scale_factor[2]
                upper_bounds[constraint_index] = self.ddx_bounds[i-2*n][1] * self.scale_factor[2]
                
            constraint_index += 1
        
#         x(i->i+1)''' = (x(i+1)'' - x(i)'') / delta_s
        for i in range(n-1):
            variables[2*n + i].append((constraint_index, -1))
            variables[2*n + i + 1].append((constraint_index, 1))
            lower_bounds[constraint_index] = self.dddx_bound[0] * self.scale_factor[2]
            upper_bounds[constraint_index] = self.dddx_bound[1] * self.scale_factor[2]
            constraint_index += 1
        
#         x(i+1)' - x(i)' - 0.5 * delta_s * x(i)'' - 0.5 * delta_s * x(i+1)'' = 0
        for i in range(n-1):
            variables[n+i].append((constraint_index, -1*self.scale_factor[2]))
            variables[n+i+1].append((constraint_index, 1*self.scale_factor[2]))
            variables[2*n+i].append((constraint_index, -0.5*self.delta_s*self.scale_factor[1]))
            variables[2*n+i+1].append((constraint_index, -0.5*self.delta_s*self.scale_factor[1]))
            lower_bounds[constraint_index] = 0
            upper_bounds[constraint_index] = 0
            constraint_index += 1
        
#         x(i+1) - x(i) - delta_s * x(i)' - 1/3 * delta_s^2 * x(i)'' - 1/6 * delta_s^2 * x(i+1)'' = 0
        for i in range(n-1):
            variables[i].append((constraint_index, -1*self.scale_factor[1]*self.scale_factor[2]))
            variables[i+1].append((constraint_index, 1*self.scale_factor[1]*self.scale_factor[2]))
            variables[n+i].append((constraint_index, -self.delta_s*self.scale_factor[0]*self.scale_factor[2]))
            variables[2*n+i].append((constraint_index, 
                                     -self.delta_s**2 / 3 * self.scale_factor[0]*self.scale_factor[1]))
            variables[2*n+i+1].append((constraint_index, 
                                     -self.delta_s**2 / 6 * self.scale_factor[0]*self.scale_factor[1])) 
            lower_bounds[constraint_index] = 0
            upper_bounds[constraint_index] = 0  
            constraint_index += 1
        
#         constrain on x_init
        variables[0].append((constraint_index, 1))
        lower_bounds[constraint_index] = self.x_init[0] * self.scale_factor[0]
        upper_bounds[constraint_index] = self.x_init[0] * self.scale_factor[0]
        constraint_index += 1
        
        variables[n].append((constraint_index, 1))
        lower_bounds[constraint_index] = self.x_init[1] * self.scale_factor[1]
        upper_bounds[constraint_index] = self.x_init[1] * self.scale_factor[1]
        constraint_index += 1
        
        variables[2*n].append((constraint_index, 1))
        lower_bounds[constraint_index] = self.x_init[2] * self.scale_factor[2]
        upper_bounds[constraint_index] = self.x_init[2] * self.scale_factor[2]
        constraint_index += 1
        
        A_data = []
        A_indices = []
        A_indptr = []
        ind_p = 0
        for i in range(num_variables):
            A_indptr.append(ind_p)
            for first, second in variables[i]:
                A_indices.append(first)
                A_data.append(second)
                ind_p += 1
        A_indptr.append(ind_p)
        
        return (A_data, A_indices, A_indptr), (lower_bounds, upper_bounds)
    
    def calculateOffset(self):
        n = self.num_knots
        kNumParam = 3 * n
        
        q = [0 for i in range(kNumParam)]
        
        if self.has_x_ref:
            for i in range(n):
                q[i] += -2 * self.w_x_ref[i] * self.x_ref[i] / self.scale_factor[0]
                
        if self.has_end_state_ref:
            q[n-1] += -2 * self.w_end_state[0] * self.end_state_ref[0] / self.scale_factor[0]
            q[2*n-1] += -2 * self.w_end_state[1] * self.end_state_ref[1] / self.scale_factor[1]            
            q[3*n-1] += -2 * self.w_end_state[2] * self.end_state_ref[2] / self.scale_factor[2]
            
        return q
    
    def formulateProblem(self):
        P = self.calculateKernel()
        A, bounds = self.calculateAffineConstraint()
        q = self.calculateOffset()
        
        kernel_dim = 3 * self.num_knots
        num_affine_constraints = len(bounds[0])
        
        osqp_data = {}
        
        osqp_data['P'] = csc_matrix(P, shape=(kernel_dim, kernel_dim))
        osqp_data['A'] = csc_matrix(A, shape=(num_affine_constraints, kernel_dim))
        osqp_data['q'] = np.array(q)
        osqp_data['l'] = np.array(bounds[0])
        osqp_data['u'] = np.array(bounds[1])
        
        return osqp_data
    
    def optimize(self, max_iter):
        osqp_data = self.formulateProblem()
        A = osqp_data['A']
        P = osqp_data['P']
        q = osqp_data['q']
        l = osqp_data['l']
        u = osqp_data['u']
        
        problem = osqp.OSQP()
        problem.setup(P, q, A, l, u, alpha = 1.0, warm_start = 1, polish = 0, scaling = 10, max_iter = max_iter, 
                      eps_abs = 1.0e-03, eps_rel = 1.0e-03, time_limit = 0, verbose = True)
        res = problem.solve()
        
        if res.info.status_val != 1 and res.info.status_val != -2:
            return None, None, None, None
        
        x = []
        dx = []
        ddx = []
        
        for i in range(self.num_knots):
            x.append(res.x[i] / self.scale_factor[0])
            dx.append(res.x[i + self.num_knots] / self.scale_factor[1])            
            ddx.append(res.x[i + 2 * self.num_knots] / self.scale_factor[2])
        
        plt.figure(figsize=(6,6))
        s = np.linspace(0, (self.num_knots-1)*self.delta_s, self.num_knots)
        plt.scatter(s, l[:self.num_knots], c='red')
        plt.scatter(s, u[:self.num_knots], c='green')
        plt.plot(s, x)
        plt.grid()
        plt.show()
            
        return s, x, dx, ddx