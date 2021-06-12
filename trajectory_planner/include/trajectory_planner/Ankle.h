# pragma once

#include "MinJerk.h"

class Ankle: private MinJerk{
    public:
        Ankle(double step_time, double ds_time, double height, double alpha, short int num_step, double dt);
        ~Ankle();
        void updateFoot(Vector3d foot_pose[]);
        void generateTrajectory(); 
        Vector3d* getTrajectoryL();
        Vector3d* getTrajectoryR();

    private:
        double tStep_;
        double tDS_;
        double dt_;
        short int num_step;  //؟؟؟
        double alpha_;
        int stepCount_;
        bool leftFirst_;
        double height_;

        Vector3d* footPose_;
        Vector3d* lFoot_;
        Vector3d* rFoot_;
       
        void updateTrajectory(bool left_first);
};