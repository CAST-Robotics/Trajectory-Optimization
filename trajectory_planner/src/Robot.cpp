#include "Robot.h" 

Robot::Robot(ros::NodeHandle *nh){

    trajGenServer_ = nh->advertiseService("/traj_gen", 
            &Robot::trajGenCallback, this);
    jntAngsServer_ = nh->advertiseService("/jnt_angs", 
            &Robot::jntAngsCallback, this);
    
    // SURENA IV geometrical params
    
    shank_ = 0.36;
    thigh_ = 0.37;
    torso_ = 0.115;
    isTrajAvailable_ = false;

    cout << "Robot Object has been Created" << endl;
}

// vector<double> Robot::spinOnline(VectorXd forceSensor, Vector3d gyro, Vector3d accelerometer, double time){
    // TODO
    // Add CoM Estimation
    // Add DCM + CoM controllers
// }

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
    delete[] q_left;
    delete[] q_right;
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
   
    double* q = new double[6];  
    MatrixXd D(3,1);

    if (isLeft)
        D << 0.0,torso_,0.0;
    else
        D << 0.0,-torso_,0.0;
        
    MatrixXd r = r7.transpose() * (p1 + r1 * D - p7);
    double C = r.norm();
    double c3 = (pow(C,2) - pow(thigh_,2) - pow(shank_,2))/(2 * thigh_ * shank_);
    if (c3 >= 1){
        q[3] = 0.0;
        // Raise error
    }else if(c3 <= -1){
        q[3] = M_PI;
        // Raise error
    }else{
        q[3] = acos(c3);       // Knee Pitch
    }
    double q4a = asin((thigh_/C) * sin(M_PI - q[3]));
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
    return q;
}

bool Robot::trajGenCallback(trajectory_planner::Trajectory::Request  &req,
                            trajectory_planner::Trajectory::Response &res)
{
    /*
        ROS service for generating robot COM & ankles trajectories
    */
    ROS_INFO("Generating Trajectory started.");
    double alpha = req.alpha;
    double t_ds = req.t_double_support;
    double t_s = req.t_step;
    double COM_height = req.COM_height;
    double step_len = req.step_length;
    int num_step = req.step_count;
    double dt = 1.0/240.0;
    double swing_height = req.ankle_height;
    double init_COM_height = thigh_ + shank_;  // SURENA IV initial height 
    
    DCMPlanner* trajectoryPlanner = new DCMPlanner(COM_height, t_s, t_ds, dt, num_step, alpha);
    Ankle* anklePlanner = new Ankle(t_s, t_ds, swing_height, alpha, num_step, dt);
    Vector3d* dcm_rf = new Vector3d[num_step];  // DCM rF
    Vector3d* ankle_rf = new Vector3d[num_step + 2]; // Ankle rF
    

    for (int i = 0; i < num_step; i++){
        dcm_rf[i] << i * step_len, pow(-1, i + 1) * torso_, 0.0;  // pow(-1, i + 1) : for specifing that first swing leg is left leg
        ankle_rf[i+1] << i * step_len, pow(-1, i + 1) * torso_, 0.0;
    }
    ankle_rf[0] << 0.0, -ankle_rf[1](1), 0.0;
    ankle_rf[num_step + 1] << ankle_rf[num_step](0), -ankle_rf[num_step](0), 0.0;
    trajectoryPlanner->setFoot(dcm_rf);
    trajectoryPlanner->getXiTrajectory();
    Vector3d com(0.0,0.0,init_COM_height);
    com_ = trajectoryPlanner->getCoM(com);
    delete[] dcm_rf;

    anklePlanner->updateFoot(ankle_rf);
    anklePlanner->generateTrajectory();
    lAnkle_ = anklePlanner->getTrajectoryL();
    rAnkle_ = anklePlanner->getTrajectoryR();
    delete[] ankle_rf;
    ROS_INFO("trajectory generated");
    res.result = true;
    isTrajAvailable_ = true;
    return true;
}

bool Robot::jntAngsCallback(trajectory_planner::JntAngs::Request  &req,
                            trajectory_planner::JntAngs::Response &res)
{
    /*
        ROS service for returning joint angles. before calling this service, 
        you must first call traj_gen service. 
    */
    if (isTrajAvailable_)
    {
        double jnt_angs[12];
        this->spinOffline(req.iter, jnt_angs);
        for(int i = 0; i < 12; i++)
            res.jnt_angs[i] = jnt_angs[i];
        ROS_INFO("joint angles requested");
    }else{
        ROS_INFO("First call traj_gen service");
        return false;
    }
    ROS_INFO("joint angles returned");
    return true;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "trajectory_node");
    ros::NodeHandle nh;
    Robot surena(&nh);
    ros::spin();
}