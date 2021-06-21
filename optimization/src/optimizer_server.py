#! /usr/bin/env python3
import rospy
from optpkg.srv import Optimization
from optpkg.srv import OptimizationRequest
from optpkg.srv import OptimizationResponse

def handlerFunction(req):
    result = req.alpha + req.t_ds_ratio + req.t_step + req.ankle_height + req.COM_height + req.mode

    return OptimizationResponse(result)


if __name__ == '__main__':
    rospy.init_node('optimizer_server')
    rospy.Service('optimization', Optimization, handlerFunction)
    rospy.spin()
