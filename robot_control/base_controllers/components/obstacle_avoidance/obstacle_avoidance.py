import cvxpy as cp
import numpy as np
import pinocchio as pin
# Problem data.
height =2
center = np.array([1,0,0])
radius = 2;

input_point =np.array([2,0,2.5])

class ObstacleAvoidance():

    def __init__(self,  verbose = False):
        self.x = cp.Variable(3)
        self.verbose = verbose

        self.control_point_frames = ['tool0',  'upper_arm_link_middle', 'forearm_link',    'forearm_link_middle', 'forearm_link_end']
        self.control_point_pos = len(self.control_point_frames)*[None]
        self.control_point_jac = len(self.control_point_frames)*[None]
        self.f_repulsive_cube = len(self.control_point_frames)*[None]
        self.f_repulsive_cyl = len(self.control_point_frames) * [None]

        self.goal_influence = 0.2
        self.kr = 0.1
        self.ka = 8
        self.d0 = 0.3

    def setCylinderParameters(self, cyl_radius, cyl_height, cyl_center_pos):
        self.cyl_radius = cyl_radius
        self.cyl_height = cyl_height
        self.cyl_center_pos = cyl_center_pos

    def setCubeParameters(self,  cube_side, cube_center_pos):
        self.cube_side = cube_side
        self.cube_center_pos = cube_center_pos

    def getCylinderDistance(self, x_ee):

        # debug
        # self.cyl_height = 2.0
        # self.cyl_center_pos = np.array([1.0, 0.0, 0.0])
        # self.cyl_radius = 2.0;
        # x_ee = np.array([2, 0, 2.5])

        base_n = np.array([[0], [0], [1]])
        base_tg = np.eye(3) - base_n.dot(base_n.T)
        objective = cp.Minimize(cp.norm2(self.x - x_ee))
        constraints = [ base_n.T @ (self.x - self.cyl_center_pos) <= self.cyl_height/2,
                        -base_n.T @ (self.x - self.cyl_center_pos) <= self.cyl_height/2,
                        cp.norm2(base_tg.T @  (self.x - self.cyl_center_pos)) <= self.cyl_radius]
        problem = cp.Problem(objective, constraints)
        # The optimal objective value is returned by `prob.solve()`.
        result = problem.solve()
        # The optimal value for x is stored in `x.value`.
        if problem.status not in ["infeasible", "unbounded"]:
            # Otherwise, problem.value is inf or -inf, respectively.
            if (self.verbose):
                print("Optimal value: %s" % problem.value)
        if (self.verbose):
                print("Closest point of the cylinder to the end effector %s is : %s " %  (x_ee, self.x.value))

        return np.linalg.norm(x_ee - self.x.value), self.x.value

    def getCubeDistance(self, x_ee):

        A = np.array([[0.0, 0.0, 1.0],
                     [0.0, 0.0, -1.0],
                     [0.0, 1.0, 0.0],
                     [0.0, -1.0, 0.0],
                     [1.0, 0.0, 0.0],
                     [-1.0, 0.0, 0.0]])
        x = cp.Variable(3)
        objective = cp.Minimize(cp.norm2(self.x - x_ee))
        constraints = [ A @ (self.x - self.cube_center_pos) <= self.cube_side/2]
        problem = cp.Problem(objective, constraints)
        result = problem.solve()
        if problem.status not in ["infeasible", "unbounded"]:
            # Otherwise, problem.value is inf or -inf, respectively.
            if (self.verbose):
                print("Optimal value: %s" % problem.value)
        if (self.verbose):
                print("Closest point of the cube to the end effector %s is : %s " %  (x_ee, self.x.value))

        return np.linalg.norm(x_ee - self.x.value), self.x.value

    def evaluatePotentials(self, goal, base_offset, robot, q):
        tau_field = np.zeros(robot.na)
        for i in  range(0, len(self.control_point_frames)):
            # in WF
            self.control_point_pos[i] =  base_offset + robot.framePlacement(q, robot.model.getFrameId(self.control_point_frames[i])).translation
            self.control_point_jac[i] =   robot.frameJacobian(q, robot.model.getFrameId(self.control_point_frames[i]), False, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3, :]

            #compute cube repulsive force
            self.f_repulsive_cube[i]  = self.computeCubeRepulsiveForce(self.control_point_pos[i])

            # compute cylinder repulsive force
            self.f_repulsive_cyl[i] = self.computeCylinderRepulsiveForce(self.control_point_pos[i])

            # compute cube attractive force
            if (self.control_point_frames[i] == 'tool0'):
                self.f_attractive = self.computeAttractiveForce(self.control_point_pos[i], goal)
                tau_field += self.control_point_jac[i].T.dot(self.f_attractive)

            # print("attractive", self.f_attractive)
            # print("repulsive cyl",self.f_repulsive_cyl)
            # print("repulsive cube", self.f_repulsive_cube)
            # print(d_cyl)
            # print(d_cube)

            #map to joints
            tau_field += self.control_point_jac[i].T.dot(self.f_repulsive_cyl[i] + self.f_repulsive_cube[i])

        return tau_field, self.f_repulsive_cube, self.f_repulsive_cyl, self.f_attractive


    def computeAttractiveForce(self, actual_pos, goal):
        d_goal = np.linalg.norm( actual_pos  - goal)
        if d_goal < self.goal_influence:  # paraboloic behaviour
            return self.ka * (goal - actual_pos)
        else:                           # conical behaviour
            return self.ka * self.goal_influence * (goal - actual_pos) / d_goal

    def computeCylinderRepulsiveForce(self, actual_pos):
        d_cyl, closest_cyl = self.getCylinderDistance(actual_pos)
        grad_distance_cyl = (actual_pos - closest_cyl) / d_cyl
        if (d_cyl < self.d0):
            return self.kr * (1 / d_cyl - 1 / self.d0) * (1 / pow(d_cyl, 2)) * grad_distance_cyl
        else:
            return np.zeros(3)

    def computeCubeRepulsiveForce(self, actual_pos):
        d_cube, closest_cube = self.getCubeDistance(actual_pos)
        grad_distance_cube = (actual_pos - closest_cube) / d_cube
        if (d_cube< self.d0):
            return self.kr * (1/d_cube - 1/self.d0)*(1/pow(d_cube,2) )* grad_distance_cube
        else:
            return np.zeros(3)

    def computeTorques(self, p, goal):
        if  (p.time > 1.5):
            # add some damping but remove P in position
            p.pid.setPDs(0.0, 5., 0.0)
            # since the obstacles are defined in the WF I need to add the offset
            d_cyl, cyl_closest_point = p.obs_avoidance.getCylinderDistance(p.x_ee + p.base_offset)
            d_cube, cube_closest_point = p.obs_avoidance.getCubeDistance(p.x_ee + p.base_offset)
            tau_field, f_repulsive_cube, f_repulsive_cyl, f_attractive = p.obs_avoidance.evaluatePotentials(goal, p.base_offset, p.robot, p.q)

            p.ros_pub.add_marker(cyl_closest_point, radius=0.05, color="blue")
            p.ros_pub.add_marker(cube_closest_point, radius=0.05, color="blue")
            p.ros_pub.add_marker(goal, radius=0.15, color="green")
            p.ros_pub.add_arrow(p.x_ee + p.base_offset, f_repulsive_cube[0], "red")
            p.ros_pub.add_arrow(p.x_ee + p.base_offset, f_repulsive_cyl[0], "red")
            p.ros_pub.add_arrow(p.x_ee + p.base_offset, f_attractive, "green")
            tau = p.h + np.copy(tau_field)
        else:
            tau = p.h + np.zeros(p.robot.na)
        return tau

