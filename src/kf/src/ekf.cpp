#include "kf/ekf.h"


EKF::EKF(int argc, char **argv)
{
    ros::init(argc,argv , "extended_kalman_filter_node");
    n = new ros::NodeHandle("~");

    // Subscribers
    gpsSub = n->subscribe("/gps/gps", 10, &EKF::gpsCallback,this);
    imuSub = n->subscribe("/gps/imu", 10, &EKF::imuCallback,this);

    // Publishers
    ekfPub = n->advertise<kf::EkfMsg>("/estimatedPosition", 10);

    debug = false;

    ros::spin();
}

void EKF::gpsCallback(const gps_common::GPSFix::ConstPtr& msg)
{
    measurementGps(msg);
    matricesInitializer();
    predict();
    update();

    m_errorVector = calculateError(m_currentSystemState , m_ZMatrix);

    publishPosition();
    debugging();

}

void EKF::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    measurementImu(msg);
}


void EKF::matricesInitializer()
{
    // Set F matrix
    double eq02 = (-m_imuMeasurement[1]*m_gpsMeasurement[2]*cos(m_imuMeasurement[0]) + m_imuMeasurement[2]*sin(m_imuMeasurement[0]) 
                   - m_imuMeasurement[2]*sin(m_dt *m_imuMeasurement[1] + m_imuMeasurement[0]) 
                   + (m_dt*m_imuMeasurement[1]*m_imuMeasurement[2] + m_imuMeasurement[1]*m_gpsMeasurement[2])*cos(m_dt*m_imuMeasurement[1] + m_imuMeasurement[0]))/pow(m_imuMeasurement[1],2);
    
    double eq12 = (-m_imuMeasurement[1]*m_gpsMeasurement[2]*sin(m_imuMeasurement[0]) - m_imuMeasurement[2]*cos(m_imuMeasurement[0]) 
                  + m_imuMeasurement[2]*cos(m_dt *m_imuMeasurement[1] + m_imuMeasurement[0])
                  - (-m_dt*m_imuMeasurement[1]*m_imuMeasurement[2]- m_imuMeasurement[1]*m_gpsMeasurement[2])*sin(m_dt*m_imuMeasurement[1] + m_imuMeasurement[0]))/pow(m_imuMeasurement[1],2);

    double eq03 = (-m_imuMeasurement[1]*sin(m_imuMeasurement[0]) + m_imuMeasurement[1]*sin(m_dt *m_imuMeasurement[1] + m_imuMeasurement[0]))/ pow(m_imuMeasurement[1],2);      

    double eq13 = (m_imuMeasurement[1]*cos(m_imuMeasurement[0]) - m_imuMeasurement[1]*cos(m_dt *m_imuMeasurement[1] + m_imuMeasurement[0]))/ pow(m_imuMeasurement[1],2);  

    double eq04 = ((-m_dt*m_imuMeasurement[2]*sin(m_dt *m_imuMeasurement[1] + m_imuMeasurement[0]) + m_dt*(m_dt*m_imuMeasurement[1]*m_imuMeasurement[2] + m_imuMeasurement[1]*m_gpsMeasurement[2])*cos(m_dt *m_imuMeasurement[1] + m_imuMeasurement[0])
                  -m_gpsMeasurement[2]*sin(m_imuMeasurement[0]) + (m_dt*m_imuMeasurement[2]+m_gpsMeasurement[2])*sin(m_dt *m_imuMeasurement[1] + m_imuMeasurement[0]))/pow(m_imuMeasurement[1],2))
                  - (2*(-m_imuMeasurement[1]*m_gpsMeasurement[2]*sin(m_imuMeasurement[0]) - m_imuMeasurement[2]*cos(m_imuMeasurement[0]) 
                  + m_imuMeasurement[2]*cos(m_dt *m_imuMeasurement[1] + m_imuMeasurement[0])
                  + (m_dt* m_imuMeasurement[1]*m_imuMeasurement[2] + m_imuMeasurement[1]*m_gpsMeasurement[2])*sin(m_dt *m_imuMeasurement[1] + m_imuMeasurement[0])))/pow(m_imuMeasurement[1],3); 

    double eq14 = ((m_dt*m_imuMeasurement[2]*cos(m_dt *m_imuMeasurement[1] + m_imuMeasurement[0]) - m_dt*(-m_dt*m_imuMeasurement[1]*m_imuMeasurement[2] - m_imuMeasurement[1]*m_gpsMeasurement[2])*sin(m_dt *m_imuMeasurement[1] + m_imuMeasurement[0])
                  +m_gpsMeasurement[2]*cos(m_imuMeasurement[0]) + (-m_dt*m_imuMeasurement[2]- m_gpsMeasurement[2])*cos(m_dt *m_imuMeasurement[1] + m_imuMeasurement[0]))/pow(m_imuMeasurement[1],2))
                  - (2*(m_imuMeasurement[1]*m_gpsMeasurement[2]*cos(m_imuMeasurement[0]) - m_imuMeasurement[2]*sin(m_imuMeasurement[0]) 
                  + m_imuMeasurement[2]*sin(m_dt *m_imuMeasurement[1] + m_imuMeasurement[0])
                  + (-m_dt* m_imuMeasurement[1]*m_imuMeasurement[2] - m_imuMeasurement[1]*m_gpsMeasurement[2])*cos(m_dt *m_imuMeasurement[1] + m_imuMeasurement[0])))/pow(m_imuMeasurement[1],3); 

    double eq05 = (m_dt*m_imuMeasurement[1]*sin(m_dt *m_imuMeasurement[1] + m_imuMeasurement[0]) - cos(m_imuMeasurement[0]) + cos(m_dt *m_imuMeasurement[1] + m_imuMeasurement[0]))/pow(m_imuMeasurement[1],2);

    double eq15 = (-m_dt*m_imuMeasurement[1]*cos(m_dt *m_imuMeasurement[1] + m_imuMeasurement[0]) - sin(m_imuMeasurement[0]) + sin(m_dt *m_imuMeasurement[1] + m_imuMeasurement[0]))/pow(m_imuMeasurement[1],2);

    std::cout << "eq02: " << eq02 << "\n";
    std::cout << "-------------------------\n";
        std::cout << "eq12: " << eq12 << "\n";
    std::cout << "-------------------------\n";
        std::cout << "eq03: " << eq03 << "\n";
    std::cout << "-------------------------\n";
        std::cout << "eq13: " << eq13 << "\n";
    std::cout << "-------------------------\n";
        std::cout << "eq04: " << eq04 << "\n";
    std::cout << "-------------------------\n";
        std::cout << "eq14: " << eq14 << "\n";
    std::cout << "-------------------------\n";
        std::cout << "eq05: " << eq05 << "\n";
    std::cout << "-------------------------\n";
            std::cout << "eq15: " << eq15 << "\n";
    std::cout << "-------------------------\n";
    // F Jacobian
    m_FMatrix <<  1 , 0 , eq02,  eq03 , eq04 , eq05,
                  0 , 1 , eq12 , eq13 , eq14 , eq15,
                  0 , 0 , 1 ,    0 ,    m_dt,  0,
                  0 , 0 , 0 ,    1,     0,     m_dt,
                  0 , 0 , 0 ,    0 ,    1 ,    0, 
                  0 , 0 , 0 ,    0,     0 ,    1;

    // Set Q matrix
    m_QMatrix << m_gpsNoiseP,  0 ,           0,               0 ,               0 ,             0,
                  0 ,          m_gpsNoiseP,  0 ,              0 ,               0 ,             0,
                  0,           0,            m_phiNoiseP , 0 ,               0, 0,
                  0 ,          0 ,           0 ,              m_velocityNoiseP, 0  ,            0,
                  0 ,          0 ,           0 ,              0  ,              m_phiDotNoiseP, 0 , 
                  0 ,          0 ,           0 ,              0,                0,              m_accNoiseP;

    // Set H matrix
    m_HMatrix << 1 , 0, 0 , 0 , 0 , 0,
                 0 , 1, 0 , 0 , 0 , 0,
                 0 , 0, 1 , 0 , 0 , 0,
                 0 , 0, 0 , 1 , 0 , 0,
                 0 , 0, 0 , 0 , 1 , 0,
                 0 , 0, 0 , 0 , 0 , 1;
                 
    // Set P matrix as diagonal 
    for (int i = 0; i < 6; ++i) {
        m_currentPMatrix(i, i) = 500;
    }

    m_RMatrix << m_gpsMeasurement[3] , 0 , 0 , 0, 0, 0,
                 0, m_gpsMeasurement[4], 0 , 0, 0, 0,
                0, 0 , 0.5, 0, 0, 0,
                0 , 0 , 0, m_gpsMeasurement[5], 0, 0,
                0 , 0 , 0 , 0, 0, 0,
                0 , 0 , 0 , 0, 0, 0;
    
    measurement();


}
void EKF::measurementGps(const gps_common::GPSFix::ConstPtr& msg)
{
    
    m_gpsMeasurement[0]=msg->longitude;
    m_gpsMeasurement[1]=msg->latitude;
    m_gpsMeasurement[2]=msg->speed;
    m_gpsMeasurement[3]=msg->err_horz;
    m_gpsMeasurement[4]=msg->err_vert;
    m_gpsMeasurement[5]=msg->err_speed;
    
}

void EKF::measurementImu(const sensor_msgs::Imu::ConstPtr& msg)
{
    Eigen::Quaternion<double> quat(msg->orientation.w , msg->orientation.x , msg->orientation.y , msg->orientation.z);

    m_imuMeasurement[0]= quat.toRotationMatrix().eulerAngles(2,1,0)[0];
    m_imuMeasurement[1]=msg->angular_velocity.z;
    m_imuMeasurement[2]=msg->linear_acceleration.x;
    m_imuMeasurement[3]=0.6;
    m_imuMeasurement[4]=0.0;
    m_imuMeasurement[5]=0.0;

}
void EKF::predict()
{
    m_predictionSystemState = systemStatesEquation(m_currentSystemState);
    m_predictPMatrix = covarianceExtrapolationEquation(m_currentPMatrix);
}


void EKF::update()
{
    m_KGain = KalmanGainCalculation(m_predictPMatrix);
    m_currentSystemState = updateCurrentState(m_predictionSystemState);
    m_currentPMatrix = updateCurrentEstimateUncertainty(m_predictPMatrix);

}

void EKF::measurement()
{
    m_ZMatrix[0] = m_gpsMeasurement[0];
    m_ZMatrix[1] = m_gpsMeasurement[1];
    m_ZMatrix[2] = m_imuMeasurement[0];
    m_ZMatrix[3] = m_gpsMeasurement[2];
    m_ZMatrix[4] = m_imuMeasurement[1];
    m_ZMatrix[5] = m_imuMeasurement[2];
}


Eigen::VectorXd EKF::systemStatesEquation(Eigen::VectorXd  currentSystemState) 
{

    Eigen::VectorXd predictionSystemState = Eigen::VectorXd::Zero(6);

    predictionSystemState = m_FMatrix*currentSystemState;

    return predictionSystemState;     
}

Eigen::MatrixXd EKF::covarianceExtrapolationEquation(Eigen::MatrixXd currentCovarianceMatrix)
{

    Eigen::MatrixXd predictPMatrix = Eigen::MatrixXd::Zero(6, 6);

    predictPMatrix = (m_FMatrix*currentCovarianceMatrix*m_FMatrix.transpose()) + m_QMatrix;


    return predictPMatrix;
}

Eigen::MatrixXd  EKF::KalmanGainCalculation(Eigen::MatrixXd predictPMatrix)
{
    Eigen::MatrixXd KGain = Eigen::MatrixXd::Zero(6, 6);

    KGain = predictPMatrix*m_HMatrix.transpose()*((m_HMatrix*predictPMatrix*(m_HMatrix.transpose()) + m_RMatrix).inverse());

    m_KGain = KGain;
    return KGain;
    
}

Eigen::VectorXd  EKF::updateCurrentState(Eigen::VectorXd estimationSystemState)
{
    estimationSystemState = Eigen::VectorXd::Zero(6);

    estimationSystemState = m_predictionSystemState + m_KGain * (m_ZMatrix - m_HMatrix*m_predictionSystemState);
    // std::cout << m_KGain <<"k_G\n";
    // std::cout << "---------------------------\n";
    // std::cout << m_predictionSystemState <<"prrrrrrrrrrrrrrrrr\n";
    // std::cout << "---------------------------\n";
    // std::cout << m_FMatrix <<"\n";
    // std::cout << "---------------------------\n";
    // std::cout << "---------------------------\n";
    // std::cout<<estimationSystemState<<"esystem\n";

    m_estimationSystemState = estimationSystemState;

    return estimationSystemState;
}

Eigen::MatrixXd  EKF::updateCurrentEstimateUncertainty(Eigen::MatrixXd estimationPMatrix)
{
    estimationPMatrix = Eigen::MatrixXd::Zero(6, 6);
    estimationPMatrix = (m_I - m_KGain * m_HMatrix) * m_predictPMatrix * (m_I - m_KGain * m_HMatrix).transpose() + m_KGain * m_RMatrix * m_KGain.transpose();
    m_estimationPMatrix = estimationPMatrix;

    return estimationPMatrix; 
}

Eigen::VectorXd EKF::calculateError(Eigen::VectorXd estimateStates, Eigen::VectorXd measurements)
{
    Eigen::VectorXd results = Eigen::VectorXd::Zero(6);
    results[0] = estimateStates[0] - measurements[0];
    results[1] = estimateStates[3] - measurements[1];

    return results;
}

void EKF::publishPosition()
{
    kf::EkfMsg result;
    result.longitude = m_currentSystemState[0];
    result.latitude = m_currentSystemState[1];
    result.yaw = m_currentSystemState[2];
    result.velocity = m_currentSystemState[3];
    result.yawRate = m_currentSystemState[4];
    result.longAcceleration = m_currentSystemState[5];
    ekfPub.publish(result);
}

void EKF::debugging()
{
    if (debug)
    {
        std::cout << "Current System State: \n " << m_currentSystemState << "\n";
        std::cout << "--------------------------------------------------------------\n";
        std::cout << "Predict System State: \n" << m_predictionSystemState << "\n";
        std::cout << "--------------------------------------------------------------\n";
        std::cout << "Covariance Matrix of The Current State Estimation: \n" << m_currentPMatrix << "\n";
        std::cout << "--------------------------------------------------------------\n";
        std::cout << "Covariance Matrix of Prediction of The Current State: \n " << m_predictPMatrix << "\n";
        std::cout << "--------------------------------------------------------------\n";
        std::cout << "Measurements: \n" << m_ZMatrix << "\n";
        std::cout << "--------------------------------------------------------------\n";
        std::cout << "Kalman Gain of States: \n" << m_KGain << "\n";
        std::cout << "--------------------------------------------------------------\n";
        // std::cout << "Process Noise Matrix: \n" << m_QMatrix << "\n";
        // std::cout << "--------------------------------------------------------------\n";
        std::cout << "Errors of State: \n" << m_errorVector << "\n";
        std::cout << "--------------------------------------------------------------\n";
    }

}