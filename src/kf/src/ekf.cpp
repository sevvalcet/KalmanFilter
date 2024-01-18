#include "../include/kf/ekf.h"

 /**
  * The ros::init() function must be called before using any part of the  ROS .
  * 
  * @param argc It can perform any ROS arguments and name remapping that were provided 
  * @param argv It can perform any ROS arguments and name remapping that were provided 
  */

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

    ros::spin();
}

 /**
  * This function is a callback which calling the functions. 
  * 
  * @param msg A message name of gps_common::GPSFix type.
  */
void EKF::gpsCallback(const gps_common::GPSFix::ConstPtr& msg)
{
    // Calling Functions
    measurementGps(msg);
    matricesInitializer();
    predict();
    update();

    m_RMSE = calculateRMSE(m_gpsVec , m_imuVec , m_predictionSystemStateArray);
    publishResults();
    debugging();

}

 /**
  * This function is a callback which calling the functions. 
  * 
  * @param msg A message name of sensor_msgs::Imu type.
  */
void EKF::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    measurementImu(msg);
}

  /**
   *This function assigns values to the defined matrices(m_FMatrix, m_QMatrix, m_HMatrix, m_currentPMatrix and m_RMatrix).
   */
void EKF::matricesInitializer()
{
    // Set F matrix' equations
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

    // The state transition matrix F Jacobian
    m_FMatrix <<  1 , 0 , eq02,  eq03 , eq04 , eq05,
                  0 , 1 , eq12 , eq13 , eq14 , eq15,
                  0 , 0 , 1 ,    0 ,    m_dt,  0,
                  0 , 0 , 0 ,    1,     0,     m_dt,
                  0 , 0 , 0 ,    0 ,    1 ,    0, 
                  0 , 0 , 0 ,    0,     0 ,    1;

    // Set the process noise matrix Q matrix
    m_QMatrix << m_gpsNoiseP,  0 ,           0,               0 ,               0 ,             0,
                  0 ,          m_gpsNoiseP,  0 ,              0 ,               0 ,             0,
                  0,           0,            m_phiNoiseP , 0 ,               0, 0,
                  0 ,          0 ,           0 ,              m_velocityNoiseP, 0  ,            0,
                  0 ,          0 ,           0 ,              0  ,              m_phiDotNoiseP, 0 , 
                  0 ,          0 ,           0 ,              0,                0,              m_accNoiseP;

    // Set observation H matrix
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

    // Set the measurement variance R matrix
    m_RMatrix << m_gpsMeasurement[3] , 0 , 0 , 0, 0, 0,
                 0, m_gpsMeasurement[4], 0 , 0, 0, 0,
                0, 0 , 0.5, 0, 0, 0,
                0 , 0 , 0, m_gpsMeasurement[5], 0, 0,
                0 , 0 , 0 , 0, 0, 0,
                0 , 0 , 0 , 0, 0, 0;
    
    // Z matrix
    measurement();

}

/**
   *This function assigns values from msg of type gps_common::GPSFix to m_gpsMeasurement.  
   *Also to assign longitude, latitude and speed values to the m_gpsVec vector to be used in RMSE calculation.
   *
   * @param msg A message name of gps_common::GPSFix type.
   */
void EKF::measurementGps(const gps_common::GPSFix::ConstPtr& msg)
{
    // Assigning values to m_gpsMeasurement from msg of type gps_common::GPSFix
    m_gpsHeader = msg->header;
    m_gpsMeasurement[0]=msg->longitude;
    m_gpsMeasurement[1]=msg->latitude;
    m_gpsMeasurement[2]=msg->speed;
    m_gpsMeasurement[3]=msg->err_horz;
    m_gpsMeasurement[4]=msg->err_vert;
    m_gpsMeasurement[5]=msg->err_speed;

    // Assigning the GPS data to a vector (m_gpsVec) to be used for RMSE calculation
    gps_common::GPSFix gpsData;
    gpsData.longitude = msg->longitude;
    gpsData.latitude = msg->latitude;
    gpsData.speed = msg->speed;

    m_gpsVec.push_back(gpsData);
  
}

/**
   *This function assigns values from msg of type sensor_msgs::Imu to m_imuMeasurement.  
   *Also to assign longitude, latitude and speed values to the m_gpsVec vector to be used in RMSE calculation.
   *
   * @param msg A message name of sensor_msgs::Imu type.
   */
void EKF::measurementImu(const sensor_msgs::Imu::ConstPtr& msg)
{
    // Vector created for Quaternion
    Eigen::Quaternion<double> quat(msg->orientation.w , msg->orientation.x , msg->orientation.y , msg->orientation.z);

    //Assigning values to m_imuMeasurement from msg of type sensor_msgs::IMU
    m_imuMeasurement[0]= quat.toRotationMatrix().eulerAngles(2,1,0)[0];    // Yaw value obtained from Quaternion data
    m_imuMeasurement[1]=msg->angular_velocity.z; // Yaw rate
    m_imuMeasurement[2]=msg->linear_acceleration.x; // Acceleration
    m_imuMeasurement[3]=0.6; // Variance of yaw
    m_imuMeasurement[4]=0.0; // Variance of yaw rate
    m_imuMeasurement[5]=0.0; // Variance of acceleration

    // Assigning the IMU data to a vector (m_imuVec) to be used for RMSE calculation
    sensor_msgs::Imu imudata;
    imudata.orientation.z = quat.toRotationMatrix().eulerAngles(2,1,0)[0]; // Yaw
    imudata.angular_velocity.z = msg->angular_velocity.z; // Yaw Rate
    imudata.linear_acceleration.x = msg->linear_acceleration.x; // Acceleration

    m_imuVec.push_back(imudata);
}

/**
   *This function calls the functions that predict the system state and covariance matrix.
   */
void EKF::predict()
{
    // The functions that predict system state and P matrix are referred to here.
    m_predictionSystemState = systemStatesEquation(m_currentSystemState);
    m_predictPMatrix = covarianceExtrapolationEquation(m_currentPMatrix);

}

/**
   *This function calls the functions that update the system state and covariance matrix.
   */
void EKF::update()
{
    // The functions that update the system state and P matrix and the function that calculates the Kalman Gain are referred to here.
    m_KGain = KalmanGainCalculation(m_predictPMatrix);
    m_currentSystemState = updateCurrentState(m_predictionSystemState);
    m_currentPMatrix = updateCurrentEstimateUncertainty(m_predictPMatrix);

}

/**
   *This function assigns the data received from GPS and IMU to a matrix.
   */
void EKF::measurement()
{
    // Assigning sensor data to the m_ZMatrix.
    m_ZMatrix[0] = m_gpsMeasurement[0];
    m_ZMatrix[1] = m_gpsMeasurement[1];
    m_ZMatrix[2] = m_imuMeasurement[0];
    m_ZMatrix[3] = m_gpsMeasurement[2];
    m_ZMatrix[4] = m_imuMeasurement[1];
    m_ZMatrix[5] = m_imuMeasurement[2];
}

/**
 * This function predicts the system state and creates an array of this data for RMSE calculation.
 *
 * @param currentSystemState Current System state.
 *
 * @return Prediction of System state.
 */
Eigen::VectorXd EKF::systemStatesEquation(Eigen::VectorXd  currentSystemState) 
{
    
    Eigen::VectorXd predictionSystemState = Eigen::VectorXd::Zero(6);
    // Calculate the prediction system state.
    predictionSystemState = m_FMatrix*currentSystemState;
    
    // Assigning the calculated predict values to the m_predictionSystemStateArray for RMSE calculation
    m_predictionSystemStateArray(0,m_temp) = predictionSystemState[0];
    m_predictionSystemStateArray(1,m_temp) = predictionSystemState[1];
    m_predictionSystemStateArray(2,m_temp) = predictionSystemState[2];
    m_predictionSystemStateArray(3,m_temp) = predictionSystemState[3];
    m_predictionSystemStateArray(4,m_temp) = predictionSystemState[4];
    m_predictionSystemStateArray(5,m_temp) = predictionSystemState[5];
    
    //Limit of data to be used in RMSE calculation
    m_temp+=1;

    if (m_temp == 1001)
    {
        m_temp=0;
    }

    return predictionSystemState;     
}

/**
 * This function predicts the covariance matrix of the system state.
 *
 * @param currentCovarianceMatrix Current Covariance matrix
 *
 * @return Prediction of Covariance matrix
 */
Eigen::MatrixXd EKF::covarianceExtrapolationEquation(Eigen::MatrixXd currentCovarianceMatrix)
{
    // Calculate the prediction covariance matrix.
    Eigen::MatrixXd predictPMatrix = Eigen::MatrixXd::Zero(6, 6);

    predictPMatrix = (m_FMatrix*currentCovarianceMatrix*m_FMatrix.transpose()) + m_QMatrix;


    return predictPMatrix;
}

/**
 * This function calculates the Kalman Gain.
 *
 * @param predictPMatrix Prediction of Covariance matrix
 *
 * @return Kalman Gain
 */
Eigen::MatrixXd  EKF::KalmanGainCalculation(Eigen::MatrixXd predictPMatrix)
{
    // Calculated the Kalman Gain.
    Eigen::MatrixXd KGain = Eigen::MatrixXd::Zero(6, 6);

    KGain = predictPMatrix*m_HMatrix.transpose()*((m_HMatrix*predictPMatrix*(m_HMatrix.transpose()) + m_RMatrix).inverse());

    m_KGain = KGain;

    return KGain;
}

/**
 * This function updated system state.
 *
 * @param estimationSystemState Estimation System State.
 *
 * @return  Estimated System State
 */
Eigen::VectorXd  EKF::updateCurrentState(Eigen::VectorXd estimationSystemState)
{
    // Estimated system state vector
    estimationSystemState = Eigen::VectorXd::Zero(6);

    estimationSystemState = m_predictionSystemState + m_KGain * (m_ZMatrix - m_HMatrix*m_predictionSystemState);

    m_estimationSystemState = estimationSystemState;

    return estimationSystemState;
}

/**
 * This function updated covariance matrix.
 *
 * @param estimationPMatrix Estimation Covariance Matrix.
 *
 * @return  Estimated Covariance Matrix
 */
Eigen::MatrixXd  EKF::updateCurrentEstimateUncertainty(Eigen::MatrixXd estimationPMatrix)
{   
    // Estimated covariance matrix of the current state.
    estimationPMatrix = Eigen::MatrixXd::Zero(6, 6);
    estimationPMatrix = (m_I - m_KGain * m_HMatrix) * m_predictPMatrix * (m_I - m_KGain * m_HMatrix).transpose() + m_KGain * m_RMatrix * m_KGain.transpose();
    m_estimationPMatrix = estimationPMatrix;


    return estimationPMatrix; 
}

/**
 * This function calculates  RMSE.
 *
 * @param gpsMeas Vector of GPS measurements
 * @param imuMeas Vector of IMU measurements
 * @param estimatedMeas Vector of estimated measurements
 *
 * @return  RMSE
 */
Eigen::VectorXd EKF::calculateRMSE(std::vector<gps_common::GPSFix> gpsMeas, std::vector<sensor_msgs::Imu> imuMeas , Eigen::MatrixXd estimatedMeas)
{
    // A vector was created for RMSE values.
    Eigen::VectorXd RMSEs = Eigen::VectorXd::Zero(6);
    // The initial value is assigned to the sum of the differences between sensor measurements and estimated values.
    double sumSquaredDiffLong = 0.0;
    double sumSquaredDiffLat = 0.0;
    double sumSquaredDiffYaw = 0.0;
    double sumSquaredDiffVel = 0.0;
    double sumSquaredDiffYawRate = 0.0;
    double sumSquaredDiffAcc = 0.0;

    // Calculate the sum of the differences between sensor measurements and estimated values.
    for (int i = 0; i < m_temp; ++i) 
    {
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
    
    // The RMSE value was calculated by finding the square root of the mean sum of differences.
    double meanSquaredDiffLong = sumSquaredDiffLong / m_temp;
    double rmseLong = std::sqrt(meanSquaredDiffLong);
    double meanSquaredDiffLat = sumSquaredDiffLat / m_temp;
    double rmseLat = std::sqrt(meanSquaredDiffLat);
    double meanSquaredDiffYaw = sumSquaredDiffYaw / m_temp;
    double rmseYaw = std::sqrt(meanSquaredDiffYaw);
    double meanSquaredDiffVel = sumSquaredDiffVel / m_temp;
    double rmseVel = std::sqrt(meanSquaredDiffVel);
    double meanSquaredDiffYawRate = sumSquaredDiffYawRate / m_temp;
    double rmseYawRate = std::sqrt(meanSquaredDiffYawRate);
    double meanSquaredDiffAcc = sumSquaredDiffAcc / m_temp;
    double rmseAcc = std::sqrt(meanSquaredDiffAcc);

    // The calculated RMSE values were assigned to the vector.
    RMSEs[0] = rmseLong;
    RMSEs[1] = rmseLat;
    RMSEs[2] = rmseYaw;
    RMSEs[3] = rmseVel;
    RMSEs[4] = rmseYawRate;
    RMSEs[5] = rmseAcc;
    
    // Debugging the RMSE values.
    if (m_temp==1000 && debug)
    {    
        std::cout << "RMSEs: " << RMSEs << "\n";
        std::cout << "--------------------------------------------------------------\n";

    }

    return RMSEs;
}

/**
 * This function publishes results and RMSE.
 *
 */
void EKF::publishResults()
{
    kf::EkfMsg result;
    kf::EkfMsg rmse;
    
    //Limit of data to be used in RMSE calculation
    int temp = 0;

    // The outputs estimated from GPS and IMU are assigned to the 'result' of the custom message type created for publishing 
    result.header = m_gpsHeader;
    result.longitude = m_currentSystemState[0];
    result.latitude = m_currentSystemState[1];
    result.yaw = m_currentSystemState[2];
    result.velocity = m_currentSystemState[3];
    result.yawRate = m_currentSystemState[4];
    result.longAcceleration = m_currentSystemState[5];
 
    // The root mean square error (RMSE) results are assigned to the 'rmse' of the custom message type created for publishing 
    rmse.header = m_gpsHeader;
    rmse.longitude = m_RMSE(0,temp);
    rmse.latitude = m_RMSE(1,temp);
    rmse.yaw = m_RMSE(2,temp);
    rmse.velocity = m_RMSE(3,temp);
    rmse.yawRate = m_RMSE(4,temp);
    rmse.longAcceleration = m_RMSE(5,temp);

    // Published the ekfPub and rmsePub.
    ekfPub.publish(result);
    rmsePub.publish(rmse);

    temp+=1;

    if (temp==1001)
    {
        temp=0;
    }


}

/**
 * This function was created to debug calculated values and display them on the terminal.  
 */
void EKF::debugging()
{
    // If the 'debug' is set to true, the following values will be printed:   
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
        std::cout << "Process Noise Matrix: \n" << m_QMatrix << "\n";
        std::cout << "--------------------------------------------------------------\n";
        
    }

}