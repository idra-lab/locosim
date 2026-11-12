import cvxpy as cp
import numpy as np
# Problem data.
center = np.array([1,0,0])
side = 0.5
input_point =np.array([1.5,0.2,0.5])

A = np.array([[0.0, 0.0, 1.0],
             [0.0, 0.0, -1.0],
             [0.0, 1.0, 0.0],
             [0.0, -1.0, 0.0],
             [1.0, 0.0, 0.0],
             [-1.0, 0.0, 0.0]])
x = cp.Variable(3)
objective = cp.Minimize(cp.norm2(x - input_point))
constraints = [ A @ (x - center) <= side/2]
problem = cp.Problem(objective, constraints)
# The optimal objective value is returned by `prob.solve()`.
result = problem.solve()
# The optimal value for x is stored in `x.value`.
if problem.status not in ["infeasible", "unbounded"]:
    # Otherwise, problem.value is inf or -inf, respectively.
    print("Optimal value: %s" % problem.value)
for variable in problem.variables():
    print("Variable %s: value %s" % (variable.name(), variable.value))


