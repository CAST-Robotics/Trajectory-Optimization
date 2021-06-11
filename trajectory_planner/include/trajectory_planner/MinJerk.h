#pragma once

#include"Eigen/Dense"
#include "Eigen/Core"
#include <Eigen/Geometry>

#include "iostream"
#include "fstream"
#include <vector>
#include "math.h"
#include "cmath"

using namespace Eigen;
using namespace std;

class MinJerk{
    public:

    protected:
        Vector3d* cubicInterpolate(Vector3d theta_ini, Vector3d theta_f, Vector3d theta_dot_ini, Vector3d theta_dot_f, double tf);
        Vector3d* poly6Interpolate(Vector3d x_ini, Vector3d x_mid, Vector3d x_f, double tf);
        Vector3d* poly6Interpolate(Vector3d x_ini, Vector3d x_mid, Vector3d x_f, double tf, Vector3d x_d_ini, Vector3d _d_f, Vector3d x_dd_ini, Vector3d x_dd_f);
        Vector3d* ankle5Poly(Vector3d x0, Vector3d xf, double z_max, double tf);
        void write2File(Vector3d* input, int size, string file_name);
};