#include "Ankle.h"

Ankle::Ankle(double step_time, double ds_time, double height, double alpha, short int num_step, double dt){
    this->tStep_ = step_time;
    this->tDS_ = ds_time;
    this->alpha_ = alpha;
    this->stepCount_ = num_step;
    this->height_ = height;
    this->dt_ = dt;

    cout << "ankle Trajectory Planner initialized\n";
}

void Ankle::updateFoot(Vector3d foot_pose[]){
    /*
        begining and end steps (Not included in DCM foot planner)
        should be given too. 
    */
    this->footPose_ = new Vector3d[stepCount_ + 2];
    footPose_ = foot_pose;
//    for (int i=0; i<stepCount_ + 2; i++)
//        cout << footPose_[i] << endl;
    if(foot_pose[0](1) > 0)
        leftFirst_= true;   // First Swing foot is left foot
    else
        leftFirst_ = false;  // First Swing foot is right foot
}

Vector3d* Ankle::getTrajectoryL(){
    return lFoot_;
}

Vector3d* Ankle::getTrajectoryR(){
    return rFoot_;
}

void Ankle::generateTrajectory(){

    int length = int(((stepCount_ + 2) * tStep_ + 1) / dt_) + 100;  // +1 second is for decreasing robot's height from COM_0 to deltaZ
    cout << length << endl;
    lFoot_ = new Vector3d[length];
    rFoot_ = new Vector3d[length];

    if(leftFirst_)
        updateTrajectory(true);
    else
        updateTrajectory(false);
}

void Ankle::updateTrajectory(bool left_first){
    int index = 0;
    int length = int(((stepCount_ + 2) * tStep_ + 1) / dt_) + 100;
    // decreasing robot's height
    for (int i = 0; i < (1 + tStep_)/dt_; i++){
        double time = dt_ * i;
        if(footPose_[0](1) > footPose_[1](1)){
            lFoot_[index] = footPose_[0];
            rFoot_[index] = footPose_[1];
        }else{
            lFoot_[index] = footPose_[1];
            rFoot_[index] = footPose_[0];
        }
        index ++;
    }

    if (left_first){

        for (int step = 1; step < stepCount_ + 1 ; step ++){
            if (step % 2 == 0){     // Left is support, Right swings
                for (double time = 0; time < (1 - alpha_) * tDS_; time += dt_){
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
                for (double time = 0; time < (alpha_) * tDS_; time += dt_){
                    lFoot_[index] = footPose_[step];
                    rFoot_[index] = footPose_[step + 1];
                    index ++;
                }
            }
            else{                  // Right is support, Left swings
                for (double time = 0; time < (1 - alpha_) * tDS_; time += dt_){
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
                for (double time = 0; time < (alpha_) * tDS_; time += dt_){
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
                for (double time = 0; time < (1 - alpha_) * tDS_; time += dt_){
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
                for (double time = 0; time < (alpha_) * tDS_; time += dt_){
                    lFoot_[index] = footPose_[step];
                    rFoot_[index] = footPose_[step + 1];
                    index ++;
                }
            }
            else{                  // Right is support, Left swings
                for (double time = 0; time < (1 - alpha_) * tDS_; time += dt_){
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
                for (double time = 0; time < (alpha_) * tDS_; time += dt_){
                    lFoot_[index] = footPose_[step + 1];
                    rFoot_[index] = footPose_[step];
                    index ++;
                }
            }
        }
    }
    Vector3d temp_left = lFoot_[index - 1];
    Vector3d temp_right = rFoot_[index - 1];
    for (int i = 0; i < (tStep_)/dt_; i++){
        double time = dt_ * i;
        lFoot_[index] = temp_left;
        rFoot_[index] = temp_right;
        index ++;
    }
    cout << index << endl;
    MinJerk::write2File(lFoot_, length, "lFoot");
    MinJerk::write2File(rFoot_, length, "rFoot");
    //cout << lFoot_[index - 1] << endl;
    //cout << rFoot_[index - 1] << endl;
    //cout << rFoot_[0] << endl;
    //cout << rFoot_[1] << endl;
}

Ankle::~Ankle(){
    delete[] footPose_;
    delete[] lFoot_;
    delete[] rFoot_;
}