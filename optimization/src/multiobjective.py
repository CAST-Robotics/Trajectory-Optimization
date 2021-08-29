#!/usr/bin/env python3

import rospy
import numpy as np
import os

from optimization.srv import Optimization
from optimization.srv import OptimizationRequest
from optimization.srv import OptimizationResponse

from pymoo.algorithms.nsga2 import NSGA2
from pymoo.optimize import minimize
from pymoo.visualization.scatter import Scatter
from pymoo.model.problem import Problem
from pymoo.factory import get_sampling, get_crossover, get_mutation, get_problem
from pymoo.util.termination.default import MultiObjectiveDefaultTermination as mdt

import pandas as pd


class surena_offline(Problem):
    """
        This class inherites from Pymoo Problem Base class
        and is used for multiobjective problem
    """
    def __init__(self):
        super().__init__(n_var=5,
                         n_obj=2,
                         n_constr=0,
                         xl=np.array([0.2,0.1,0.5,0.65,0.025]),
                         xu=np.array([0.7,0.5,1.3,0.7,0.075]))

    def _evaluate(self, X, out, *args, **kwargs):
        f1 = np.empty(X.shape[0])
        f2 = np.empty_like(f1)
        #g = np.empty_like(f2)

        for i in range(X.shape[0]):
            rospy.wait_for_service('optimization')
            optimization_client = rospy.ServiceProxy('optimization',Optimization)
            #res = optimization_client(X[i,0]._value, X[i,1]._value, X[i,2]._value, X[i,3]._value, X[i,4]._value, 5)
            res = optimization_client(X[i,0], X[i,1], X[i,2], X[i,3], X[i,4], 5)
            f1[i] = res.j[0]
            f2[i] = res.j[1]
            #g[i] = res.j[2]
        out["F"] = np.column_stack([f1, f2])
        #out["G"] = g

if __name__ == "__main__":
    rospy.wait_for_service('optimization')
    optimization_client = rospy.ServiceProxy('optimization',Optimization)
    
    problem = surena_offline()
    # problem = get_problem("bnh")

    algorithm = NSGA2(pop_size=75,
                        #n_offsprings=1,
                        sampling=get_sampling("real_random"),
                        crossover=get_crossover("real_sbx", prob=0.9, eta=10),
                        mutation=get_mutation("real_pm", eta=15),
                        eliminate_duplicates=True)
    termination = mdt(x_tol=1e-8,
        cv_tol=1e-6,
        f_tol=0.005,
        nth_gen=5,
        n_last=20,
        n_max_gen=100,
        n_max_evals=5000)

    res = minimize(problem,
                algorithm,
                termination,
                seed=1,
                verbose=True, save_history=True)
    
    plot = Scatter()
    plot.add(problem.pareto_front(), plot_type="line", color="black", linewidth=2)
    plot.add(res.F, color="red")
    plot.show()

    os.chdir("/home/cast/SurenaProject/Code/SurenaV/Optim_ws")
    print(res.X.shape)
    df = pd.DataFrame(np.hstack((res.X,res.F)))

    df.to_excel("multiObject.xlsx")