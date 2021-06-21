#pragma once

#include <ros/ros.h>
#include "trajectory_planner/JntAngs.h"
#include "trajectory_planner/Trajectory.h"

#include "DCM.h"
#include "Link.h"
#include "PID.h"
#include "Ankle.h"

#include "fstream"

using namespace std;

class Robot{
    friend class Surena;
    public:
        Robot(ros::NodeHandle *nh);

        // vector<double> spinOnline(VectorXd forceSensor, Vector3d gyro, Vector3d accelerometer, double time);
        void spinOffline(int iter, double* config);
        bool jntAngsCallback(trajectory_planner::JntAngs::Request  &req,
                            trajectory_planner::JntAngs::Response &res);
        bool trajGenCallback(trajectory_planner::Trajectory::Request  &req,
                            trajectory_planner::Trajectory::Response &res);
    private:

        double thigh_;
        double shank_;
        double torso_;


        double joints_[12];

        PID* DCMController_;
        PID* CoMController_;

        void doIK(MatrixXd pelvisP, Matrix3d pelvisR, MatrixXd leftAnkleP, Matrix3d leftAnkleR, MatrixXd rightAnkleP, Matrix3d rightAnkleR);
        double* geometricIK(MatrixXd p1, MatrixXd r1, MatrixXd p7, MatrixXd r7, bool isLeft);

        Matrix3d Rroll(double phi);
        Matrix3d RPitch(double theta);

        Vector3d* com_;
        Vector3d* rAnkle_;
        Vector3d* lAnkle_;

        ros::ServiceServer jntAngsServer_;
        ros::ServiceServer trajGenServer_;
        bool isTrajAvailable_;
};
