#include "headers/PID.h"

PID::PID(double kp, double ki, double kd, double timeStep){
    this->kp_ = kp;
    this->ki_ = ki;
    this->kd_ = kd;
    
    this->intI_ = 0.0;
    this->prevoiusError_ = 0.0;
    this->dt_ = timeStep;

}

double PID::getOutput(double desiredValue, double currentValue){
    /*
        computes next plant input
    */
    double error = currentValue - desiredValue;
    this->intI_ += error * dt_;
    double deriv = (error - this->prevoiusError_) / dt_;
    this->prevoiusError_ = error;

    return kp_ * error + ki_ * this->intI_ + kd_ * deriv;
}