#include "headers/Ankle.h"

Ankle::Ankle(double step_time, double ds_time, double height, double alpha, short int num_step, double dt){
    this->tStep_ = step_time;
    this->tDS_ = ds_time;
    this->alpha = alpha;
    this->stepCount = num_step;
    this->height_ = height;
    this->dt_ = dt;

    cout << "ankle Trajectory Planner initialized\n";
}

void Ankle::updateFoot(Vector3d foot_pose[]){
    /*
        begining and end steps (Not included in DCM foot planner)
        should be given too. 
    */
    this->footPose_ = new Vector3d[stepCount + 2];
    footPose_ = foot_pose;
    if(foot_pose[0](1) > 0)
        leftFirst = true;   // First Swing foot is left foot
    else
        leftFirst = false;  // First Swing foot is right foot
}

Vector3d* Ankle::getTrajectoryL(){
    return lFoot_;
}

Vector3d* Ankle::getTrajectoryR(){
    return rFoot_;
}

void Ankle::generateTrajectory(){

    lFoot_ = new Vector3d[int(stepCount * tStep_ / dt_)];
    rFoot_ = new Vector3d[int(stepCount * tStep_ / dt_)];

    if(leftFirst)
        updateTrajectory(true);
    else
        updateTrajectory(false);

    ofstream file("log/ankle.csv");
    for(int i = 0; i < int(stepCount * tStep_ / dt_); ++i){
        file << lFoot_[i](0) << ","<< lFoot_[i](1) << ","<< lFoot_[i](2) << "," << rFoot_[i](0) << ","<< rFoot_[i](1) << ","<< rFoot_[i](2) << "\n"; 
    }
    file.close();
}

void Ankle::updateTrajectory(bool left_first){
    int index = 0;

    if (left_first){
        for (int step = 1; step < stepCount + 1 ; step ++){
            if (step % 2 == 0){     // Left is support, Right swings
                for (double time = 0; time < (1 - alpha) * tDS_; time += dt_){
                    lFoot_[index] = footPose_[step];
                    rFoot_[index] = footPose_[step - 1];
                    index ++;
                }
                Vector3d* coefs = ankle5Poly(footPose_[step-1],footPose_[step+1], height_,tStep_-tDS_);
                for(double time = 0.0; time < tStep_ - tDS_; time += dt_){
                    lFoot_[index] = footPose_[step];
                    rFoot_[index] = coefs[0] + coefs[1] * time + coefs[2] * pow(time,2) + coefs[3] * pow(time,3) + coefs[4] * pow(time,4) + coefs[5] * pow(time,5);
                    index ++;
                }
                delete coefs;
                for (double time = 0; time < (alpha) * tDS_; time += dt_){
                    lFoot_[index] = footPose_[step];
                    rFoot_[index] = footPose_[step + 1];
                    index ++;
                }
            }
            else{                  // Right is support, Left swings
                for (double time = 0; time < (1 - alpha) * tDS_; time += dt_){
                    lFoot_[index] = footPose_[step - 1];
                    rFoot_[index] = footPose_[step];
                    index ++;
                }
                Vector3d* coefs = ankle5Poly(footPose_[step-1],footPose_[step+1], height_,tStep_-tDS_);
                for(double time = 0.0; time < tStep_ - tDS_; time += dt_){
                    rFoot_[index] = footPose_[step];
                    lFoot_[index] = coefs[0] + coefs[1] * time + coefs[2] * pow(time,2) + coefs[3] * pow(time,3) + coefs[4] * pow(time,4) + coefs[5] * pow(time,5);
                    index ++;
                }
                delete coefs;
                for (double time = 0; time < (alpha) * tDS_; time += dt_){
                    lFoot_[index] = footPose_[step + 1];
                    rFoot_[index] = footPose_[step];
                    index ++;
                }
            }
        }
    }

    else{       // Right Foot Swings first
        for (int step = 1; step < num_step + 1 ; step ++){
            if (step % 2 != 0){     // Left is support, Right swings
                for (double time = 0; time < (1 - alpha) * tDS_; time += dt_){
                    lFoot_[index] = footPose_[step];
                    rFoot_[index] = footPose_[step - 1];
                    index ++;
                }
                Vector3d* coefs = ankle5Poly(footPose_[step-1],footPose_[step+1], height_, tStep_-tDS_);
                for(double time = 0.0; time < tStep_ - tDS_; time += dt_){
                    lFoot_[index] = footPose_[step];
                    rFoot_[index] = coefs[0] + coefs[1] * time + coefs[2] * pow(time,2) + coefs[3] * pow(time,3) + coefs[4] * pow(time,4) + coefs[5] * pow(time,5);
                    index ++;
                }
                delete coefs;
                for (double time = 0; time < (alpha) * tDS_; time += dt_){
                    lFoot_[index] = footPose_[step];
                    rFoot_[index] = footPose_[step + 1];
                    index ++;
                }
            }
            else{                  // Right is support, Left swings
                for (double time = 0; time < (1 - alpha) * tDS_; time += dt_){
                    lFoot_[index] = footPose_[step - 1];
                    rFoot_[index] = footPose_[step];
                    index ++;
                }
                Vector3d* coefs = ankle5Poly(footPose_[step-1],footPose_[step+1],height_,tStep_-tDS_);
                for(double time = 0.0; time < tStep_ - tDS_; time += dt_){
                    rFoot_[index] = footPose_[step];
                    lFoot_[index] = coefs[0] + coefs[1] * time + coefs[2] * pow(time,2) + coefs[3] * pow(time,3) + coefs[4] * pow(time,4) + coefs[5] * pow(time,5);
                    index ++;
                }
                delete coefs;
                for (double time = 0; time < (alpha) * tDS_; time += dt_){
                    lFoot_[index] = footPose_[step + 1];
                    rFoot_[index] = footPose_[step];
                    index ++;
                }
            }
        }
    }
}