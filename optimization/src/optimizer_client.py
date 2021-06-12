#! /usr/bin/env python3
import rospy
import sys
from optimization.srv import Optimization
from optimization.srv import OptimizationRequest
from optimization.srv import OptimizationResponse
import numpy as np
from geneticalgorithm import geneticalgorithm as ga




def f(X):
    rospy.wait_for_service('optimization')
    optimization_client = rospy.ServiceProxy('optimization',Optimization)
    result = optimization_client(X[0], X[1], X[2], X[3], X[4])
    return(result.result)

if __name__ == '__main__':

    algorithm_param = {'max_num_iteration': 200,\
                   'population_size':100,\
                   'mutation_probability':0.1,\
                   'elit_ratio': 0.01,\
                   'crossover_probability': 0.5,\
                   'parents_portion': 0.3,\
                   'crossover_type':'uniform',\
                   'max_iteration_without_improv':30}
    varbound=np.array([[0.2 ,0.7], [0.01, 2.5], [0.1, 0.5], [0.1, 0.5], [0.5, 0.65]])
    model=ga(function=f,dimension=5,variable_type='real',variable_boundaries=varbound, algorithm_parameters=algorithm_param )
    model.run()   
