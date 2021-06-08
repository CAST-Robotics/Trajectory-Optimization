#!/usr/bin/env python3

import pybullet_data
import pybullet
import time
import numpy as np
import rospy
from trajectory_planner.srv import JntAngs
from optpkg.srv import Optimization

class robot_sim:
    def __init__(self, robot_vel = 0.7, time = 5.0, real_time = False, freq = 240.0):
        ## rosrun bullet_sim bullet.py
        ## run above command in catkin_work space
        
        self.real_time = real_time
        self.iter = 0
        self.robotVel = robot_vel
        self.simTime = time
        self.freq = freq

        rospy.init_node('surena_sim')
        self.rate = rospy.Rate(self.freq)

        self.phisycsClient = pybullet.connect(pybullet.GUI)
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())

        self.robotID = None
        self.planeID = None
        self.reset()

        # Check the service names with other packages
        self.resetServer = rospy.Service('reset_service',self.reset_sim, self.reset_handle)
        self.optimServer = rospy.Service('optimizer', Optimization, self.run)

        pass

    def simulationSpin(self):
        rospy.spin()
        pass

    def run(self, optim_req):
        
        # Call Servic to generate trajectory
        # TODO:
        #   1 - update service names from trajectory package
        #   2 - add optimization modes
        rospy.wait_for_service("/jnt_angs")
        trajectory_handle = rospy.ServiceProxy("/jnt_angs", JntAngs)
        done = trajectory_handle(optim_req.alpha,optim_req.t_double_support,optim_req.t_step,
                    optim_req.step_length,optim_req.COM_height)

        while not done:
            print("Trajectory generation failed, calling again...")
            done = trajectory_handle(optim_req.alpha,optim_req.t_double_support,optim_req.t_step,
                    optim_req.step_length,optim_req.COM_height)
        j_E = 0.0
        j_ZMP = 0.0
        j_torque = 0.0
        j_vel = 0.0

        while self.iter < self.simTime * self.freq:
            rospy.wait_for_service("/jnt_angs")
            try:
                joint_state_handle = rospy.ServiceProxy("/jnt_angs", JntAngs)
                All = joint_state_handle(self.iter)
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
                
                j_E += self.calcEnergy()
                j_torque += self.calcTorque()
                j_vel += self.calcVel()
                
                zmp = self.calcZMP()
                if self.zmpViolation(zmp):
                    j_ZMP += self.zmpOffset(zmp)
                else:
                    j_ZMP -= self.zmpOffset

            except rospy.ServiceException as e:
                print("Jntangls Service call failed: %s"%e)

        return j_E
    
    def calcEnergy(self):
        energy = 0
        for i in range(12):
            energy += abs(pybullet.getJointState(self.robotID, i)[1] * pybullet.getJointState(self.robotID, i)[3])
        return energy

    def calcTorque(self):
        torque = 0
        for i in range(12):
            torque += abs(pybullet.getJointState(self.robotID, i)[3])
        return torque
    
    def calcVel(self):
        total_vel = 0
        for i in range(12):
            total_vel += abs(pybullet.getJointState(self.robotID, i)[1])
        return total_vel

    def calcZMP(self):
        p_r, f_r = self.zmp_1(True)
        p_l, f_l = self.zmp_1(False)
        return (self.zmp_all(p_r,p_l,f_r, f_l))

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
    
    def point2line(self,p0,p1,p2):
        # This function calculates distance between
        # point p0 and the line passing through p2,p1
        # https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line

        return abs((p2[0]-p1[0]) * (p1[1]-p0[1]) - (p1[0]-p0[0]) * (p2[1]-p1[1])) / ((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)

    def zmpViolation(self, zmp):
        # checks if zmp is inside the polygon shaped by
        # vertexes V using Windings algorithm
        # inspiration: http://www.dgp.toronto.edu/~mac/e-stuff/point_in_polygon.py

        V = list("")
        for point in pybullet.getContactPoints(self.robotID, self.planeID, 5):
            V.append(np.array(point))
        for point in pybullet.getContactPoints(self.robotID, self.planeID, 11):
            V.append(np.array(point))
        V.append(V[0])  # add first point again to close polygon

        cn = 0     # crossing number
        for i in range(len(V) - 1):
            if ((V[i][1] <= zmp[1] and V[i+1][1] > zmp[1])   # an upward crossing
                or (V[i][1] > zmp[1] and V[i+1][1] <= zmp[1])):  # a downward crossing
                # compute the actual edge-ray intersect x-coordinate
                vt = (zmp[1] - V[i][1]) / float(V[i+1][1] - V[i][1])
                if zmp[0] < V[i][0] + vt * (V[i+1][0] - V[i][0]): # P[0] < intersect
                    cn += 1  # a valid crossing of y=P[1] right of P[0]

        if cn % 2 == 1:
            return False
        else:
            return True

    def zmpOffset(self, zmp):
        min_dist = np.inf

        V = list("")
        for point in pybullet.getContactPoints(self.robotID, self.planeID, 5):
            V.append(np.array(point))
        for point in pybullet.getContactPoints(self.robotID, self.planeID, 11):
            V.append(np.array(point))

        for i in range(len(V) - 1):
            dist = self.point2line(V[i],V[i+1], zmp)
            if dist < min_dist:
                min_dist = dist
        return dist
    
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
    robot.simulationSpin()
    pass