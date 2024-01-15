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
    rmsePub = n->advertise<kf::EkfMsg> ("/rmse",10);
    debug = false;


    ros::spin();
}

void EKF::gpsCallback(const gps_common::GPSFix::ConstPtr& msg)
{
    measurementGps(msg);
    matricesInitializer();
    predict();
    update();

    m_RMSE = calculateRMSE(m_gpsVec , m_imuVec , m_predictionSystemStateArray);
    publishResults();
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
    m_gpsHeader = msg->header;
    m_gpsMeasurement[0]=msg->longitude;
    m_gpsMeasurement[1]=msg->latitude;
    m_gpsMeasurement[2]=msg->speed;
    m_gpsMeasurement[3]=msg->err_horz;
    m_gpsMeasurement[4]=msg->err_vert;
    m_gpsMeasurement[5]=msg->err_speed;

    gps_common::GPSFix gpsData;
    gpsData.longitude = msg->longitude;
    gpsData.latitude = msg->latitude;
    gpsData.speed = msg->speed;

    m_gpsVec.push_back(gpsData);
    
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

    sensor_msgs::Imu imudata;
    imudata.orientation.z = quat.toRotationMatrix().eulerAngles(2,1,0)[0];
    imudata.angular_velocity = msg->angular_velocity;
    imudata.linear_acceleration.x = msg->linear_acceleration.x;

    m_imuVec.push_back(imudata);

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
    
    m_predictionSystemStateArray(0,m_temp) = predictionSystemState[0];
    m_predictionSystemStateArray(1,m_temp) = predictionSystemState[1];
    m_predictionSystemStateArray(2,m_temp) = predictionSystemState[2];
    m_predictionSystemStateArray(3,m_temp) = predictionSystemState[3];
    m_predictionSystemStateArray(4,m_temp) = predictionSystemState[4];
    m_predictionSystemStateArray(5,m_temp) = predictionSystemState[5];
    

    m_temp+=1;


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

Eigen::VectorXd EKF::calculateRMSE(std::vector<gps_common::GPSFix> gpsMeas, std::vector<sensor_msgs::Imu> imuMeas , Eigen::MatrixXd estimatedMeas)
{
    Eigen::VectorXd RMSEs = Eigen::VectorXd::Zero(6);
    double sumSquaredDiffLong = 0.0;
    double sumSquaredDiffLat = 0.0;
    double sumSquaredDiffYaw = 0.0;
    double sumSquaredDiffVel = 0.0;
    double sumSquaredDiffYawRate = 0.0;
    double sumSquaredDiffAcc = 0.0;

    for (size_t i = 0; i < gpsMeas.size(); ++i) {
        double diffLong = gpsMeas[i].longitude - estimatedMeas(0,i);
        double diffLat = gpsMeas[i].latitude - estimatedMeas(1,i);
        double diffYaw = imuMeas[i].orientation.z - estimatedMeas(2,i);
        double diffVel = gpsMeas[i].speed - estimatedMeas(3,i);
        double diffYawRate = imuMeas[i].angular_velocity.z - estimatedMeas(4,i);
        double diffAcc = imuMeas[i].linear_acceleration.x - estimatedMeas(5,i);
        sumSquaredDiffLong += pow(diffLong,2);
        sumSquaredDiffLat += pow(diffLat,2);
        sumSquaredDiffYaw += pow(diffYaw,2);
        sumSquaredDiffVel += pow(diffVel,2);
        sumSquaredDiffYawRate += pow(diffYawRate,2);
        sumSquaredDiffAcc += pow(diffAcc,2);
    }

    double meanSquaredDiffLong = sumSquaredDiffLong / gpsMeas.size();
    double rmseLong = std::sqrt(meanSquaredDiffLong);

    double meanSquaredDiffLat = sumSquaredDiffLat / gpsMeas.size();
    double rmseLat = std::sqrt(meanSquaredDiffLat);

    double meanSquaredDiffYaw = sumSquaredDiffYaw / gpsMeas.size();
    double rmseYaw = std::sqrt(meanSquaredDiffYaw);

    double meanSquaredDiffVel = sumSquaredDiffVel / gpsMeas.size();
    double rmseVel = std::sqrt(meanSquaredDiffVel);

    double meanSquaredDiffYawRate = sumSquaredDiffYawRate / gpsMeas.size();
    double rmseYawRate = std::sqrt(meanSquaredDiffYawRate);

    double meanSquaredDiffAcc = sumSquaredDiffAcc / gpsMeas.size();
    double rmseAcc = std::sqrt(meanSquaredDiffAcc);

    RMSEs[0] = rmseLong;
    RMSEs[1] = rmseLat;
    RMSEs[2] = rmseYaw;
    RMSEs[3] = rmseVel;
    RMSEs[4] = rmseYawRate;
    RMSEs[5] = rmseAcc;
    
    std::cout << "RMSEs: " << RMSEs << "\n";

    return RMSEs;
}

void EKF::publishResults()
{
    kf::EkfMsg result;
    kf::EkfMsg rmse;
    
    int temp = 0;

    result.header = m_gpsHeader;
    result.longitude = m_currentSystemState[0];
    result.latitude = m_currentSystemState[1];
    result.yaw = m_currentSystemState[2];
    result.velocity = m_currentSystemState[3];
    result.yawRate = m_currentSystemState[4];
    result.longAcceleration = m_currentSystemState[5];
    
    rmse.header = m_gpsHeader;
    rmse.longitude = m_RMSE(0,temp);
    rmse.latitude = m_RMSE(1,temp);
    rmse.yaw = m_RMSE(2,temp);
    rmse.velocity = m_RMSE(3,temp);
    rmse.yawRate = m_RMSE(4,temp);
    rmse.longAcceleration = m_RMSE(5,temp);

    ekfPub.publish(result);
    rmsePub.publish(rmse);

    temp+=1;

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
        std::cout << "m_RMSE: \n" << m_RMSE << "\n";
        std::cout << "--------------------------------------------------------------\n";
    }

}