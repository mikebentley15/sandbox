#!/usr/bin/env python3
"""
This is the primary file for the assignment, and is the ONLY file you should
need to edit to complete it. Each homework problem has its own function in the 
class LBR4TrajectoryOptimization that sets up the optimization problem for it
(e.g. compute_trajectory_1_1 is self-contained to do the optimization for 
Problem 1.1 in the assignment). The parts you are to complete are clearly marked
with TODO tags and boxed off by comment lines.

There is a main function at the end of this file. Please see README.txt for
examples on running if you are unclear how to use command line options.

The other related file (on which this one depends) is lbr4_kinematics.py, which
implements forward kinematics for the LBR4 robot. It has a few useful functions
you will need, so you should read through that file first to know what you're
working with.


Author: Adam Conkey
Created: Fall 2018
"""

import sys
import os
import time
from lbr4_kinematics import LBR4Kinematics
try:
    import vrep
except ImportError:
    print(
        "Import of vrep failed. Make sure the 'vrep.py' and 'remoteAPI.dylib'"
        " files are in this directory.")
    sys.exit(1)
try:
    import numpy as np
except ImportError:
    print("Import of numpy failed. Make sure it's installed on your system.")
    sys.exit(1)
try:
    from casadi import SX, MX, Function, nlpsol, vertcat, norm_2
except ImportError:
    print("Import of casadi failed. Make sure it's installed on your system.")
    sys.exit(1)


_NUM_DOF = 7  # Number of robot arm degrees of freedom
_3D_DOF = 3  # Degrees of freedom for task space
    

class LBR4TrajectoryOptimization:
    def __init__(self):
        self.render_obstacle = False
        self.lbr4 = LBR4Kinematics()
        # These are the goal/obstacle positions for collision avoidance
        self.goal_position = [0.0, 0.8, 0.5]
        self.obstacle_position = [0.0, 0.4, 1.0]

        # Initialize optimization lists. These store the optimization variables
        # and any constraints for each timestep.
        self.num_timesteps = 300   # Number of timesteps to optimize over (N)
        self.theta_trajectory = [] # Joint position trajectory, length N
        self.theta_guess = []      # Initial trajectory for optimization, length N
        self.lb_theta = []         # Lower bounds on joint positions, length N
        self.ub_theta = []         # Upper bounds on joint positions, length N
        self.g = []                # Constraints to be satisfied
        self.lbg = []              # Lower bounds on constraints 
        self.ubg = []              # Upper bounds on constraints
        self.running_cost = 0      # Running cost for optimization

    def compute_trajectory_1_1(self):
        """
        Setup the optimization problem for 1.1 and run the solver.
        """

        #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
        # TODO: Write the instantaneous cost function for problem 1.1        #
        #                                                                    #
        def cost_function(x_goal, theta):
            """
            This is the computed cost for one timestep in the optimization.

            Input:
                x_goal: Symbolic variable for EE goal position (3-DOF)
                theta: Symbolic variable for joint angles (7-DOF)
            Output:
                Symbolic computation of cost according to 1.1 description
            """

            # YOUR CODE HERE (make sure you change the return value)

            return None 
        #                                                                    #
        # END cost function definition                                       #
        #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

        # Setup the CasADi function for computing cost given defined cost function
        theta = SX.sym('theta', _NUM_DOF)  # Joint angles in configuration space
        x_goal = SX.sym('x_goal', _3D_DOF)  # 3D position of goal
        cost = cost_function(x_goal, theta)
        F = Function('F', [theta, x_goal], [cost], ['theta', 'x_goal'],
                     ['cost'])

        # Add initial conditions
        theta_0 = MX.sym('theta_0', _NUM_DOF)
        self.theta_trajectory += [theta_0]
        self.theta_guess += [0.0 for _ in range(_NUM_DOF)]
        # Setting upper/lower bounds the same => equality constraint
        self.lb_theta += [0.0 for _ in range(_NUM_DOF)]
        self.ub_theta += [0.0 for _ in range(_NUM_DOF)]

        # Set the EE goal position for the robot to reach to
        self.goal_position = self.lbr4.generate_random_ee_position()

        # Add symbolic variables to build the optimization problem for N-2
        # timesteps (Note: it's N-2 because we handle the first and last
        # timesteps differently)
        for k in range(self.num_timesteps - 2):  # Sans first/last steps

            # Create a symbolic variable for joint state at this timestep
            theta_k = MX.sym('theta_' + str(k + 1), _NUM_DOF)

            # Build symbolic trajectory, these are the variables being optimized
            self.theta_trajectory += [theta_k]

            # Add upper/lower limits on the state (joint limit constraints)
            self.lb_theta += self.lbr4.lower_joint_lims
            self.ub_theta += self.lbr4.upper_joint_lims

            # Provide initial guess for solution.
            # NOTE: you can likely get better or faster solutions by providing
            # a guess closer to what the actual solution would be, but for
            # simplicity we just feed it all zeros. It seems to work fine.
            self.theta_guess += [0 for _ in range(_NUM_DOF)]

            # Compute the cost for this timestep and add to running cost
            cost_k = F(theta=theta_k, x_goal=self.goal_position)['cost']
            self.running_cost += cost_k

        # Handle final timestep
        theta_N = MX.sym('theta_N', _NUM_DOF)
        self.theta_trajectory += [theta_N]
        self.lb_theta += self.lbr4.lower_joint_lims
        self.ub_theta += self.lbr4.upper_joint_lims
        self.theta_guess += [0 for _ in range(_NUM_DOF)]
        cost_N = F(theta=theta_N, x_goal=self.goal_position)
        self.running_cost += cost_N['cost']

        # Let's solve the optimization problem!
        problem = {
            'x': vertcat(*self.theta_trajectory),
            'g': vertcat(*self.g),
            'f': self.running_cost
        }
        solver = nlpsol("nlp", "ipopt", problem)
        soln = solver(
            x0=self.theta_guess,
            lbx=self.lb_theta,
            ubx=self.ub_theta,
            lbg=self.lbg,
            ubg=self.ubg)

        trajectory = np.vsplit(soln['x'].full(), self.num_timesteps)
        return trajectory

    def compute_trajectory_1_2(self, gamma=0.005):
        """
        Setup the optimization problem for 1.2 and run the solver.
        Input:
            gamma: Absolute value of constraint on instantaneous joint velocity
        """

        #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
        # TODO: Write the cost function for problem 1.2                      #
        #                                                                    #
        def cost_function(x_goal, theta):
            """
            This is the computed cost for one timestep in the optimization.

            Input:
                x_goal: Symbolic variable for EE goal position (3-DOF)
                theta: Symbolic variable for joint angles (7-DOF)
            Output:
                Symbolic computation of cost according to 1.2 description
            """

            # COPY your cost function from 1.1

            return None 
        #                                                                    #
        # END cost function definition                                       #
        #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

        # Setup the CasADi function for computing cost given defined cost function
        theta = SX.sym('theta', _NUM_DOF)  # Joint angles in configuration space
        x_goal = SX.sym('x_goal', _3D_DOF)  # 3D position of goal
        cost = cost_function(x_goal, theta)
        F = Function('F', [theta, x_goal], [cost], ['theta', 'x_goal'],
                     ['cost'])

        # Add initial conditions
        theta_0 = MX.sym('theta_0', _NUM_DOF)
        theta_prev = theta_0
        self.theta_trajectory += [theta_0]
        self.theta_guess += [0.0 for _ in range(_NUM_DOF)]
        # Setting upper/lower bounds the same => equality constraint
        self.lb_theta += [0.0 for _ in range(_NUM_DOF)]
        self.ub_theta += [0.0 for _ in range(_NUM_DOF)]

        # Set the EE goal position for the robot to reach to
        self.goal_position = self.lbr4.generate_random_ee_position()

        # Add symbolic variables to build the optimization problem
        for k in range(self.num_timesteps - 2):  # Sans first/last steps

            # Create a symbolic variable for joint state at this timestep
            theta_k = MX.sym('theta_' + str(k + 1), _NUM_DOF)

            # Build symbolic trajectory, these are the variables being optimized
            self.theta_trajectory += [theta_k]

            # Add upper/lower limits on the state (joint limit constraints)
            self.lb_theta += self.lbr4.lower_joint_lims
            self.ub_theta += self.lbr4.upper_joint_lims

            # Provide initial guess for solution.
            self.theta_guess += [0 for _ in range(_NUM_DOF)]

            # Compute the cost for this timestep and add to running cost
            cost_k = F(theta=theta_k, x_goal=self.goal_position)['cost']
            self.running_cost += cost_k

            #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
            # TODO: Add inequality constraint for this timestep to limit      #
            # instaneous joint velocity.                                      #
            #-----------------------------------------------------------------#
            # Toy example: If we wanted to express a constraint 0 <= A <= 10, #
            # where A is a vector of length 5, then we would write            #
            #     self.g += [A]                                               #
            #     self.lbg += [0 for _ in range(5)]                           #
            #     self.ubg += [10 for _ in range(5)]                          #
            #-----------------------------------------------------------------#
            #                                                                 #

                # YOUR CODE HERE

            #                                                                 #
            # END add inequlaity constaint                                    #
            #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

            theta_prev = theta_k

        # Handle final timestep
        theta_N = MX.sym('theta_N', _NUM_DOF)
        self.theta_trajectory += [theta_N]
        self.lb_theta += self.lbr4.lower_joint_lims
        self.ub_theta += self.lbr4.upper_joint_lims
        self.theta_guess += [0 for _ in range(_NUM_DOF)]
        cost_N = F(theta=theta_N, x_goal=self.goal_position)
        self.running_cost += cost_N['cost']

        #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
        # TODO: Add an equality constraint that forces the end-effector       #
        # position to coincide with the goal position.                        #
        #---------------------------------------------------------------------#
        # Toy Example: If we have a scalar variable A that we want to be      #
        # constrained to equal 8, then we would write                         #
        #     self.g += [A]                                                   #
        #     self.lbg += [8]                                                 #
        #     self.ubg += [8]                                                 #
        # (Note: equality constraints just set upper/lower bounds on          #
        # inequality constraints to be equal.)                                #
        #---------------------------------------------------------------------#
        #                                                                     #
        
            # YOUR CODE HERE

        #                                                                     #
        # END add equality constraint                                         #
        #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#


        #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
        # TODO: Add an equality constraint that forces the joint velocity at  #
        # the final timestep to be zero.                                      #
        #                                                                     #
        
            # YOUR CODE HERE

        #                                                                     #
        # END add equality constraint                                         #
        #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

        # Let's solve the optimization problem!
        problem = {
            'x': vertcat(*self.theta_trajectory),
            'g': vertcat(*self.g),
            'f': self.running_cost
        }
        solver = nlpsol("nlp", "ipopt", problem)
        soln = solver(
            x0=self.theta_guess,
            lbx=self.lb_theta,
            ubx=self.ub_theta,
            lbg=self.lbg,
            ubg=self.ubg)

        trajectory = np.vsplit(soln['x'].full(), self.num_timesteps)
        return trajectory

    def compute_trajectory_1_3(self, alpha=0.0):
        """
        Setup the optimization problem for 1.3 and run the solver.
        Input:
            alpha: Parameter governing the influence of smoothness term in
                   cost function
        """

        #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
        # TODO: Write the cost function for problem 1.3                      #
        #                                                                    #
        def cost_function(x_goal, theta, theta_prev, alpha):
            """
            This is the computed cost for one timestep in the optimization.

            Input:
                x_goal: Symbolic variable for EE goal position (3-DOF)
                theta: Symbolic variable for current joint angles (7-DOF)
                theta_prev: Symbolic varialbe for previous joint angles (7-DOF)
                alpha: Scalar parameter governing influence of smoothness term
            Output:
                Symbolic computation of cost according to 1.3 description
            """
 
            # YOUR CODE HERE (make sure you change the return value)

            return None 
        #                                                                    #
        # END cost function definition                                       #
        #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

        # Setup the CasADi function for computing cost given defined cost function
        theta = SX.sym('theta', _NUM_DOF)  # Joint angles in configuration space
        theta_p = SX.sym('theta_prev', _NUM_DOF)  # Previous joint angles
        x_goal = SX.sym('x_goal', _3D_DOF)  # 3D position of goal
        cost = cost_function(x_goal, theta, theta_p, alpha)
        F = Function('F', [theta, theta_p, x_goal], [cost],
                     ['theta', 'theta_prev', 'x_goal'], ['cost'])

        # Add initial conditions
        theta_0 = MX.sym('theta_0', _NUM_DOF)
        theta_prev = theta_0
        self.theta_trajectory += [theta_0]
        self.theta_guess += [0.0 for _ in range(_NUM_DOF)]
        # Setting upper/lower bounds the same => equality constraint
        self.lb_theta += [0.0 for _ in range(_NUM_DOF)]
        self.ub_theta += [0.0 for _ in range(_NUM_DOF)]

        # Set the EE goal position for the robot to reach to
        self.goal_position = self.lbr4.generate_random_ee_position()

        # Add symbolic variables to build the optimization problem
        for k in range(self.num_timesteps - 2):  # Sans first/last steps

            # Create a symbolic variable for joint state at this timestep
            theta_k = MX.sym('theta_' + str(k + 1), _NUM_DOF)

            # Build symbolic trajectory, these are the variables being optimized
            self.theta_trajectory += [theta_k]

            # Add upper/lower limits on the state (joint limit constraints)
            self.lb_theta += self.lbr4.lower_joint_lims
            self.ub_theta += self.lbr4.upper_joint_lims

            # Provide initial guess for solution.
            self.theta_guess += [0 for _ in range(_NUM_DOF)]

            # Compute the cost for this timestep and add to running cost
            cost_k = F(
                theta=theta_k,
                theta_prev=theta_prev,
                x_goal=self.goal_position)['cost']
            self.running_cost += cost_k

            theta_prev = theta_k

        # Handle final timestep
        theta_N = MX.sym('theta_N', _NUM_DOF)
        self.theta_trajectory += [theta_N]
        self.lb_theta += self.lbr4.lower_joint_lims
        self.ub_theta += self.lbr4.upper_joint_lims
        self.theta_guess += [0 for _ in range(_NUM_DOF)]
        cost_N = F(theta=theta_N, x_goal=self.goal_position)
        self.running_cost += cost_N['cost']

        #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
        # TODO: Add an equality constraint that forces the end-effector       #
        # position to coincide with the goal position.                        #
        #                                                                     #
        
            # YOUR CODE HERE (copy this constraint from the previous problem)

        #                                                                     #
        # END add equality constraint                                         #
        #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

        # Let's solve the optimization problem!
        problem = {
            'x': vertcat(*self.theta_trajectory),
            'g': vertcat(*self.g),
            'f': self.running_cost
        }
        solver = nlpsol("nlp", "ipopt", problem)
        soln = solver(
            x0=self.theta_guess,
            lbx=self.lb_theta,
            ubx=self.ub_theta,
            lbg=self.lbg,
            ubg=self.ubg)

        trajectory = np.vsplit(soln['x'].full(), self.num_timesteps)
        return trajectory

    def compute_trajectory_2(self, gamma=0.005, beta=0.0):
        """
        Setup the optimization problem for 2 and run the solver.
        Input:
            gamma: Absolute value of constraint on instantaneous joint velocity
            beta: Parameter governing influence of obstacle avoidance term
        """

        #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
        # TODO: Write the cost function for problem 2                        #
        #                                                                    #
        def cost_function(x_goal, x_obst, theta, beta):
            """
            This is the computed cost for one timestep in the optimization.

            Input:
                x_goal: Symbolic variable for EE goal position (3-DOF)
                x_obst: Symbolic variable for obstacle centroid position (3-DOF)
                theta: Symbolic variable for joint angles (7-DOF)
                beta: Scalar value governing influence of obstacle avoidance
            Output:
                Symbolic computation of cost according to 2 description
            """
 
            # YOUR CODE HERE (make sure you change the return value)

            return None 
        #                                                                    #
        # END cost function definition                                       #
        #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

        # Setup the CasADi function for computing cost given defined cost function
        theta = SX.sym('theta', _NUM_DOF)  # Joint angles in configuration space
        x_goal = SX.sym('x_goal', _3D_DOF)  # 3D position of goal
        x_obst = SX.sym('x_obst', _3D_DOF)  # 3D position of obstacle
        cost = cost_function(x_goal, x_obst, theta, beta)
        F = Function('F', [theta, x_goal, x_obst], [cost],
                     ['theta', 'x_goal', 'x_obst'], ['cost'])

        # Add initial conditions
        theta_0 = MX.sym('theta_0', _NUM_DOF)
        theta_prev = theta_0
        self.theta_trajectory += [theta_0]
        self.theta_guess += [0.0 for _ in range(_NUM_DOF)]
        # Setting upper/lower bounds the same => equality constraint
        self.lb_theta += [0.0 for _ in range(_NUM_DOF)]
        self.ub_theta += [0.0 for _ in range(_NUM_DOF)]

        # Add symbolic variables to build the optimization problem
        for k in range(self.num_timesteps - 2):  # Sans first/last steps

            # Create a symbolic variable for joint state at this timestep
            theta_k = MX.sym('theta_' + str(k + 1), _NUM_DOF)

            # Build symbolic trajectory, these are the variables being optimized
            self.theta_trajectory += [theta_k]

            # Add upper/lower limits on the state (joint limit constraints)
            self.lb_theta += self.lbr4.lower_joint_lims
            self.ub_theta += self.lbr4.upper_joint_lims

            # Provide initial guess for solution
            self.theta_guess += [0 for _ in range(_NUM_DOF)]

            # Compute the cost for this timestep and add to running cost
            cost_k = F(
                theta=theta_k,
                x_goal=self.goal_position,
                x_obst=self.obstacle_position)['cost']
            self.running_cost += cost_k

            #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
            # TODO: Add inequality constraint for this timestep to limit      #
            # instaneous joint velocity.                                      #
            #                                                                 #
            
                # COPY this constraint from previous problem

            #                                                                 #
            # END add inequlaity constaint                                    #
            #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

            theta_prev = theta_k

        # Handle final timestep
        theta_N = MX.sym('theta_N', _NUM_DOF)
        self.theta_trajectory += [theta_N]
        self.lb_theta += self.lbr4.lower_joint_lims
        self.ub_theta += self.lbr4.upper_joint_lims
        self.theta_guess += [0 for _ in range(_NUM_DOF)]
        cost_N = F(theta=theta_N, x_goal=self.goal_position)
        self.running_cost += cost_N['cost']

        #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
        # TODO: Add an equality constraint that forces the velocity at the    #
        # final timestep to be zero.                                          #
        #                                                                     #

            # COPY this constraint from previous problem
        
        #                                                                     #
        # END add equality constraint                                         #
        #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

        #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
        # TODO: Add an equality constraint that forces the end-effector       #
        # position to coincide with the goal position.                        #
        #                                                                     #
      
            # COPY this constraint from previous problem

        #                                                                     #
        # END add equality constraint                                         #
        #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

        # Let's solve the optimization problem!
        problem = {
            'x': vertcat(*self.theta_trajectory),
            'g': vertcat(*self.g),
            'f': self.running_cost
        }
        solver = nlpsol("nlp", "ipopt", problem)
        soln = solver(
            x0=self.theta_guess,
            lbx=self.lb_theta,
            ubx=self.ub_theta,
            lbg=self.lbg,
            ubg=self.ubg)

        trajectory = np.vsplit(soln['x'].full(), self.num_timesteps)
        return trajectory

    def execute_trajectory(self, trajectory):
        """
        Creates a remote session with V-REP and sends joint position commands
        to the LBR4 robot. Commanded at 100Hz.

        Input:
            trajectory: List of N joint positions, each position a vector of
            length 7.

        NOTE: You do not need to change anything in this function.
        """

        try:
            vrep.simxFinish(-1)
            clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
            if clientID == -1:
                print("Failed connecting to remote API server")
                sys.exit(1)
            print("Successfully connected to remote API server.")

            # Stop the simulation in case it was running from previous run
            vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
            time.sleep(1.0)
            # Start the simulation
            vrep.simxSynchronous(clientID, True)
            vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking)

            # Get handles for robot joints and objects
            joint_names = ["LBR4p_joint{}".format(i) for i in range(1, 8)]
            joint_handles = [
                vrep.simxGetObjectHandle(clientID, joint,
                                         vrep.simx_opmode_blocking)[1]
                for joint in joint_names
            ]
            _, target_handle = vrep.simxGetObjectHandle(
                clientID, "Target", vrep.simx_opmode_blocking)
            _, obstacle_handle = vrep.simxGetObjectHandle(
                clientID, "Obstacle", vrep.simx_opmode_blocking)

            # Set the object positions
            vrep.simxSetObjectPosition(clientID, target_handle, -1,
                                       self.goal_position,
                                       vrep.simx_opmode_oneshot)
            if self.render_obstacle:
                vrep.simxSetObjectPosition(clientID, obstacle_handle, -1,
                                           self.obstacle_position,
                                           vrep.simx_opmode_oneshot)

            # Command the desired joint position
            print("Arm should be moving right now...")
            for joint_angles in trajectory:
                for i in range(_NUM_DOF):
                    vrep.simxSetJointTargetPosition(clientID, joint_handles[i],
                                                    joint_angles[i],
                                                    vrep.simx_opmode_oneshot)
                vrep.simxSynchronousTrigger(clientID)
                time.sleep(0.01)  # Operating at 100Hz
        finally:
            vrep.simxGetPingTime(clientID)
            vrep.simxFinish(clientID)
            print("Simulation complete.")


if __name__ == '__main__':
    """
    Provides command line option interface using argparse. See README.txt if
    you are unfamiliar with running programs with command line options.
    """
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-p',
        '--problem',
        dest='problem',
        type=str,
        choices={'1.1', '1.2', '1.3', '2'},
        required=True,
        help="Homework problem to execute")
    parser.add_argument(
        '-g',
        '--gamma',
        dest='gamma',
        type=float,
        help="Absolute value of limit on joint velocity")
    parser.add_argument(
        '-a',
        '--alpha',
        dest='alpha',
        type=float,
        help="Scalar factor on smoothness term in 1.3")
    parser.add_argument(
        '-b',
        '--beta',
        dest='beta',
        type=float,
        help="Scalar factor on obstacle avoidance term in 2.1")
    args = parser.parse_args(sys.argv[1:])

    lbr4_traj_opt = LBR4TrajectoryOptimization()
    if args.problem == '1.1':
        trajectory = lbr4_traj_opt.compute_trajectory_1_1()
    elif args.problem == '1.2':
        gamma = args.gamma if args.gamma else 0.005
        trajectory = lbr4_traj_opt.compute_trajectory_1_2(gamma)
    elif args.problem == '1.3':
        alpha = args.alpha if args.alpha else 0.0
        trajectory = lbr4_traj_opt.compute_trajectory_1_3(alpha)
    elif args.problem == '2':
        gamma = args.gamma if args.gamma else 0.005
        beta = args.beta if args.beta else 0.0
        trajectory = lbr4_traj_opt.compute_trajectory_2(gamma, beta)
        lbr4_traj_opt.render_obstacle = True
    else:
        print("Unknown homework problem: {}".format(args.problem))
        sys.exit(1)

    lbr4_traj_opt.execute_trajectory(trajectory)
