#! /usr/bin/env python3
import rospy
from optpkg.srv import Optimization
from optpkg.srv import OptimizationRequest
from optpkg.srv import OptimizationResponse

def handlerFunction(req):
    result = req.alpha + req.t_double_support + req.t_step + req.step_length + req.COM_height

    return OptimizationResponse(result)


if __name__ == '__main__':
    rospy.init_node('optimizer_server')
    rospy.Service('optimization', Optimization, handlerFunction)
    rospy.spin()




