import cvxpy as cp
import numpy as np
# Problem data.
height =2
center = np.array([1,0,0])
radius = 2;

input_point =np.array([2,0,2.5])

base_n = np.array([[0],[0] ,[1]])
base_tg = np.eye(3) - base_n.dot(base_n.T)
d = height /2


x = cp.Variable(3)
objective = cp.Minimize(cp.norm2(x - input_point))
constraints = [ base_n.T @ (x - center) <= d,
                -base_n.T @ (x - center) <= d,
                cp.norm2(base_tg.T @  (x -center)) <= radius]
problem = cp.Problem(objective, constraints)
# The optimal objective value is returned by `prob.solve()`.
result = problem.solve()
# The optimal value for x is stored in `x.value`.
if problem.status not in ["infeasible", "unbounded"]:
    # Otherwise, problem.value is inf or -inf, respectively.
    print("Optimal value: %s" % problem.value)
for variable in problem.variables():
    print("Variable %s: value %s" % (variable.name(), variable.value))





