#include "Robot.h" 

Robot::Robot(ros::NodeHandle *nh){

    jntAngsServer_ = nh->advertiseService("/jnt_angs", 
            &Robot::jntAngsCallback, this);

    short int pelvis_children[3] = {0, 13, 26};
    _Link pelvis(30,"pelvis",30,3,pelvis_children,0.0);

    ///////////// Right Leg Parameters ////////////////////
    short int rHipR_children[1] = {1};      // RHipP
    _Link rHipR(0,"rHipR",30,1,rHipR_children,0.09);
    short int rHipP_children[1] = {2};      // RHipY
    _Link rHipP(1,"rHipP",0,1,rHipP_children,0.0);
    short int rHipY_children[1] = {3};      // RKnee
    _Link rHipY(2,"rHipY",1,1,rHipY_children,0.0);
    short int rKnee_children[1] = {4};      // RAnkleP
    _Link rKnee(3,"rKnee",2,1,rKnee_children,0.3535);
    short int rAnkleP_children[1] = {5};    // RAnkleR
    _Link rAnkleP(4,"rAnkleP",3,1,rAnkleP_children,0.3);
    _Link rAnkleR(1,"rAnkleR",4,0,nullptr,0.0);

    // joints_.push_back(rHipR);
    // joints_.push_back(rHipP);
    // joints_.push_back(rHipY);
    // joints_.push_back(rKnee);
    // joints_.push_back(rAnkleP);
    // joints_.push_back(rAnkleR);

    ///////////// Left Leg Parameters ////////////////////
    short int lHipR_children[1] = {14};      // LHipP
    _Link lHipR(13,"lHipR",30,1,lHipR_children,0.09);
    short int lHipP_children[1] = {15};      // LHipY
    _Link lHipP(14,"lHipP",13,1,rHipP_children,0.0);
    short int lHipY_children[1] = {16};      // LKnee
    _Link lHipY(15,"lHipY",14,1,rHipY_children,0.0);
    short int lKnee_children[1] = {17};      // LAnkleP
    _Link lKnee(16,"lKnee",15,1,rKnee_children,0.3535);
    short int lAnkleP_children[1] = {18};    // LAnkleR
    _Link lAnkleP(17,"lAnkleP",16,1,rAnkleP_children,0.3);
    _Link lAnkleR(18,"lAnkleR",17,0,nullptr,0.0);

    // joints_.push_back(lHipR);
    // joints_.push_back(lHipP);
    // joints_.push_back(lHipY);
    // joints_.push_back(lKnee);
    // joints_.push_back(lAnkleP);
    // joints_.push_back(lAnkleR);

    trajectoryPlanner_ = new DCMPlanner(0.6, 1.0, 0.3, 0.001, 6, 0.5);
    anklePlanner_ = new Ankle(1.0, 0.3, 0.05,0.5,6,0.001);

    ////////////////////////////// create simple foot step plan /////////////////////////////////
    Vector3d* f = new Vector3d[6];
    
    f[0] << 0.0, -0.09, 0.0;
    f[1] << 0.4, 0.09, 0.0;
    f[2] << 0.8, -0.09, 0.0;
    f[3] << 1.2, 0.09, 0.0;
    f[4] << 1.6, -0.09, 0.0;
    f[5] << 2.0, 0.09, 0.0;
    
    trajectoryPlanner_->setFoot(f);
    trajectoryPlanner_->getXiTrajectory();
    Vector3d com(0.0,0.0,0.713);
    com_ = trajectoryPlanner_->getCoM(com);
    
    delete f;
    Vector3d* f2 = new Vector3d[8];
    f2[0] << 0.0, 0.09, 0.0;
    f2[1] << 0.0, -0.09, 0.0;
    f2[2] << 0.4, 0.09, 0.0;
    f2[3] << 0.8, -0.09, 0.0;
    f2[4] << 1.2, 0.09, 0.0;
    f2[5] << 1.6, -0.09, 0.0;
    f2[6] << 2.0, 0.09, 0.0;
    f2[7] << 2.0, -0.09, 0.0;
    anklePlanner_->updateFoot(f2);
    anklePlanner_->generateTrajectory();
    lAnkle_ = anklePlanner_->getTrajectoryL();
    rAnkle_ = anklePlanner_->getTrajectoryR();
    delete f2;
    /////////////////////////////////////////////////////////////////////////////////////////////

    cout << "Robot Object has been Created" << endl;
}

vector<double> Robot::spinOnline(VectorXd forceSensor, Vector3d gyro, Vector3d accelerometer, double time){
    // TODO
    // Add CoM Estimation
    // Add DCM + CoM controllers
}

void Robot::spinOffline(int iter, double* config){

    MatrixXd lfoot(3,1);
    MatrixXd rfoot(3,1);
    Matrix3d attitude = MatrixXd::Identity(3,3);
    MatrixXd pelvis(3,1);
    lfoot << lAnkle_[iter](0), lAnkle_[iter](1), lAnkle_[iter](2);
    rfoot << rAnkle_[iter](0), rAnkle_[iter](1), rAnkle_[iter](2);
    pelvis << com_[iter](0), com_[iter](1), com_[iter](2);
    doIK(pelvis,attitude,lfoot,attitude,rfoot,attitude);

    for(int i = 0; i < 12; i++)
        config[i] = joints_[i];     // right leg(0-5) & left leg(6-11)
}

void Robot::doIK(MatrixXd pelvisP, Matrix3d pelvisR, MatrixXd leftAnkleP, Matrix3d leftAnkleR, MatrixXd rightAnkleP, Matrix3d rightAnkleR){
    // Calculates and sets Robot Leg Configuration at each time step
    double* q_left = this->geometricIK(pelvisP, pelvisR, leftAnkleP, leftAnkleR, true);
    double* q_right = this->geometricIK(pelvisP, pelvisR, rightAnkleP, rightAnkleR, false);
    for(int i = 0; i < 6; i ++){
        joints_[i] = q_right[i];
        joints_[i+6] = q_left[i];
    }
}

Matrix3d Robot::Rroll(double phi){
    // helper Function for Geometric IK
    MatrixXd R(3,3);
    double c=cos(phi);
    double s=sin(phi);
    R<<1,0,0,0,c,-1*s,0,s,c;
    return R;
}

Matrix3d Robot::RPitch(double theta){
    // helper Function for Geometric IK
    MatrixXd Ry(3,3);
    double c=cos(theta);
    double s=sin(theta);
    Ry<<c,0,s,0,1,0,-1*s,0,c;
    return Ry;

}

double* Robot::geometricIK(MatrixXd p1, MatrixXd r1, MatrixXd p7, MatrixXd r7, bool isLeft){
    /*
        Geometric Inverse Kinematic for Robot Leg (Section 2.5  Page 53)
        Reference: Introduction to Humanoid Robotics by Kajita        https://www.springer.com/gp/book/9783642545351
        1 ----> Body        7-----> Foot
    */
    double a = 0.3535;
    double b = 0.3;
    double e = 0;
    MatrixXd E(3,1);
    E << 0.0, 0.0, -e;
    double* q = new double[6];  
    MatrixXd D(3,1);
    if (isLeft)
        //D << 0.0,0.09,0.0;
        D << 0.0,0.09,0.0;
    else
        //D << 0.0,-0.09,0.0;
        D << 0.0,-0.09,0.0;
        
    //MatrixXd r = r7.transpose() * (p1 + r1 * D + r1 * e - p7); // r1 * e ??
    MatrixXd r = r7.transpose() * (p1 + r1 * D - p7);
    double C = r.norm();
    double c3 = (pow(C,2) - pow(a,2) - pow(b,2))/(2 * a * b);
    if (c3 >= 1){
        q[3] = 0.0;
        // Raise error
    }else if(c3 <= -1){
        q[3] = M_PI;
        // Raise error
    }else{
        q[3] = acos(c3);       // Knee Pitch
    }
    double q4a = asin((a/C) * sin(M_PI - q[3]));
    q[5] = atan2(r(1,0),r(2,0));   //Ankle Roll
    if (q[5] > M_PI/2){
        q[5] = q[5] - M_PI;
        // Raise error
    }
    else if (q[5] < -M_PI / 2){
        q[5] = q[5] + M_PI;
        // Raise error
    }
    int sign_r2 = 1;
    if(r(2,0) < 0)
        sign_r2 = -1;
    q[4] = -atan2(r(0,0),sign_r2 * sqrt(pow(r(1,0),2) + pow(r(2,0),2))) - q4a;      // Ankle Pitch
    Matrix3d R = r1.transpose() * r7 * Rroll(-q[5]) * RPitch(-q[3] - q[4]);
    q[0] = atan2(-R(0,1),R(1,1));         // Hip Yaw
    q[1] = atan2(R(2,1), -R(0,1) * sin(q[0]) + R(1,1) * cos(q[0]));           // Hip Roll
    q[2] = atan2(-R(2,0), R(2,2));        // Hip Pitch
    //return q;
    double* choreonoid_only= new double[6] {q[1],q[2],q[0],q[3],q[4],q[5]};
    return choreonoid_only;
}

bool Robot::jntAngsCallback(trajectory_planner::JntAngs::Request  &req,
                            trajectory_planner::JntAngs::Response &res)
{
    double jnt_angs[12];
    this->spinOffline(req.iter, jnt_angs);
    for(int i = 0; i < 12; i++)
        res.jnt_angs[i] = jnt_angs[i];
    return true;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "optimization_node");
    ros::NodeHandle nh;
    Robot surena(&nh);
    ROS_INFO("Ready to calculate joint angles.");
    ros::spin();
}