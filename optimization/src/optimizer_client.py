#! /usr/bin/env python3
import rospy
import sys
from optimization.srv import Optimization
from optimization.srv import OptimizationRequest
from optimization.srv import OptimizationResponse
import numpy as np
from geneticalgorithm import geneticalgorithm as ga

##      Optimization Mode       ##
#       1 ---> Energy            #
#       2 ---> Joint Vel         #
#       3 ---> Joint Torque      #
#       4 ---> ZMP               #
#   5 ---> Multi Objective       #
mode = 4

def f(X):
    rospy.wait_for_service('optimization')
    optimization_client = rospy.ServiceProxy('optimization',Optimization)
    result = optimization_client(X[0], X[1], X[2], X[3], X[4], mode)
    return(result.result)

if __name__ == '__main__':

    algorithm_param = {'max_num_iteration': 800,\
                   'population_size':100,\
                   'mutation_probability':0.08,\
                   'elit_ratio': 0.03,\
                   'crossover_probability': 0.8,\
                   'parents_portion': 0.3,\
                   'crossover_type':'uniform',\
                   'max_iteration_without_improv':30}
    varbound=np.array([[0.2 ,0.7], [0.1, 0.45], [0.1, 0.5], [0.5, 0.7], [0.025,0.075]])
    model=ga(function=f,dimension=5,variable_type='real',variable_boundaries=varbound, algorithm_parameters=algorithm_param, function_timeout = 40 )
    model.run()   
