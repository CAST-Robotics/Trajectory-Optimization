#!/usr/bin/env python3

import pybullet_data
import pybullet
import time
import numpy as np
import rospy
from trajectory_planner.srv import JntAngs

class robot_sim:
    def __init__(self, real_time = False):
        ## rosrun bullet_sim bullet.py
        ## run above command in catkin_work space
        
        self.real_time = real_time
        self.iter = 0

        rospy.init_node('surena_sim')
        self.rate = rospy.Rate(240)

        phisycsClient = pybullet.connect(pybullet.GUI)
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())

        self.robotID = None
        self.planeID = None
        self.reset()

        self.reset_server = rospy.Service('reset_service',self.reset_sim, self.reset_handle)

        pass

    def run(self):
        
        rospy.wait_for_service('jntAngs')
        try:
            service_handle = rospy.ServiceProxy('JntAngs', JntAngs)
            All = service_handle(self.iter)
            leftConfig = All[6:12]
            rightConfig = All[0:6]
            for index in range (6):
                pybullet.setJointMotorControl2(bodyIndex=self.robotID,
                                        jointIndex=index,
                                        controlMode=pybullet.POSITION_CONTROL,
                                        targetPosition = rightConfig[index])
                pybullet.setJointMotorControl2(bodyIndex=self.robotID,
                                        jointIndex=index + 6,
                                        controlMode=pybullet.POSITION_CONTROL,
                                        targetPosition = leftConfig[index])
            pybullet.stepSimulation()
            
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        pass

    def zmp_1(self,is_right):
        ans = np.zeros(3)
        if is_right:
            sole_id = 5
        else:
            sole_id = 11
        force = 0.0
        
        if len(pybullet.getContactPoints(self.robotID, self.planeID, sole_id)) == 0:
            return [ans, 0.0]
        
        for point in pybullet.getContactPoints(self.robotID, self.planeID, sole_id):
            ans += np.array(point[6]) * point[9]
            force += point[9]
        return [ans / force, force]

    def zmp_all(self,zmp_r, zmp_l, total_f_r, total_f_l):
        if total_f_r == 0.0:    # right foot swings
            return zmp_l
        elif total_f_l == 0.0:  # left foot swings
            return zmp_r
        else:                # double support
            return (zmp_r * total_f_r + zmp_l * total_f_l)/(total_f_r + total_f_l)
    
    def reset(self):
        
        self.iter = 0
        pybullet.resetSimulation()
        self.planeID = pybullet.loadURDF("plane.urdf")
        pybullet.setGravity(0,0,-9.81)
        self.robotID = pybullet.loadURDF("src/Trajectory-Optimization/bullet_sim/meshes/simple_robot.urdf",
                                [0.0,0.0,0.0],pybullet.getQuaternionFromEuler([0.0,0.0,0.0]),useFixedBase = 0)
        
        if self.real_time:
            pybullet.setRealTimeSimulation(1)
        else:
            pybullet.setRealTimeSimulation(0)
        pass

    def reset_sim(self,req):
        rospy.loginfo("Restarting Simulation...")
        try:
            self.reset()
            rospy.loginfo("Simulation Restarted")
            return 1
        except:
            rospy.loginfo("Service failed")
            return 0

    def close():
        pybullet.disconnect()


if __name__ == "__main__":
    robot = robot_sim()
    robot.run()
    pass