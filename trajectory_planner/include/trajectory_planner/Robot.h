#pragma once

#include "headers/DCM.h"
#include "headers/Link.h"
#include "headers/PID.h"
#include "headers/Ankle.h"

#include "fstream"

using namespace std;

class Robot{
    friend class Surena;
    public:
        Robot();

        vector<double> spinOnline(VectorXd forceSensor, Vector3d gyro, Vector3d accelerometer, double time);
        vector<double> spinOffline(int iter);

    private:

        DCMPlanner* trajectoryPlanner_;
        Ankle* anklePlanner_;

        vector<_Link> joints_;

        PID* DCMController_;
        PID* CoMController_;

        void doIK(MatrixXd pelvisP, Matrix3d pelvisR, MatrixXd leftAnkleP, Matrix3d leftAnkleR, MatrixXd rightAnkleP, Matrix3d rightAnkleR);
        double* geometricIK(MatrixXd p1, MatrixXd r1, MatrixXd p7, MatrixXd r7, bool isLeft);

        Matrix3d Rroll(double phi);
        Matrix3d RPitch(double theta);

        Vector3d* com_;
        Vector3d* rAnkle_;
        Vector3d* lAnkle_;
};
