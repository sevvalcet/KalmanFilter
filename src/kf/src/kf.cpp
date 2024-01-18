#include "kf/kf.h"
#include "../include/kf/kf.h"

/**
  * The ros::init() function must be called before using any part of the  ROS .
  * 
  * @param argc It can perform any ROS arguments and name remapping that were provided 
  * @param argv It can perform any ROS arguments and name remapping that were provided 
  */
KF::KF(int argc, char **argv)
{
    ros::init(argc,argv , "kalman_filter_node");
    n = new ros::NodeHandle("~");

    ros::Subscriber gpsSub = n->subscribe("/gps/gps", 10, &KF::gpsCallback,this);
    gpsPub = n->advertise<gps_common::GPSFix>("/estimatedPosition", 10);
    rmsePub = n->advertise<gps_common::GPSFix>("/RMSE", 10);


    debug = true;

    ros::spin();

}

 /**
  * This function is a callback which calling the functions. 
  * 
  * @param msg A message name of gps_common::GPSFix type.
  */
void KF::gpsCallback(const gps_common::GPSFix::ConstPtr& msg)
{

    matricesInitializer();
    predict();
    measurement(msg);
    update();

    m_errorVector = calculateRMSE(m_predictionSystemStateArray , m_ZMatrix);

    publishResults();
    debugging();

}

  /**
   *This function assigns values to the defined matrices(m_FMatrix, m_QMatrix, m_HMatrix and m_currentPMatrix).
   */
void KF::matricesInitializer()
{
    // Set F matrix
    m_FMatrix << 1 , m_dt , 0.5*pow(m_dt , 2), 0 , 0 , 0,
                  0 , 1 , m_dt , 0 , 0 , 0,
                  0 , 0, 1 , 0 , 0, 0,
                  0 ,0 ,0 , 1, m_dt, 0.5*pow(m_dt , 2),
                  0 , 0 , 0 , 0 , 1 ,m_dt, 
                  0 ,0 ,0 ,0, 0 ,1;

    // Set Q matrix
    m_QMatrix << pow(m_dt , 4)/4 , pow(m_dt , 3)/2  , pow(m_dt , 2)/2 , 0 , 0 , 0,
                  pow(m_dt , 3)/2  , pow(m_dt , 2), m_dt , 0 , 0 , 0,
                  pow(m_dt , 2)/2, m_dt, 1 , 0 , 0, 0,
                  0 ,0 ,0 , pow(m_dt , 4)/4 , pow(m_dt , 3)/2  , pow(m_dt , 2)/2,
                  0 , 0 , 0 ,pow(m_dt , 3)/2  , pow(m_dt , 2), m_dt , 
                  0 ,0 ,0 ,pow(m_dt , 2)/2, m_dt, 1;
    m_QMatrix = m_QMatrix * m_accVariance;

    // Set H matrix
    m_HMatrix << 1 , 0, 0 , 0 , 0 , 0,
                 0 , 0, 0 , 1 , 0 , 0;
                 
    // Set P matrix as diagonal 
    for (int i = 0; i < 6; ++i) {
        m_currentPMatrix(i, i) = 500;
    }


}

/**
   *This function calls the functions that predict the system state and covariance matrix.
   */
void KF::predict()
{
    m_predictionSystemState = systemStatesEquation(m_currentSystemState);
    m_predictPMatrix = covarianceExtrapolationEquation(m_currentPMatrix);
}

/**
   *This function assigns the data received from GPS to a matrix and measurement variance R matrix.
   */
void KF::measurement(const gps_common::GPSFix::ConstPtr& msg)
{
    Eigen::Vector2d ZMatrix = Eigen::Vector2d::Zero(2);   // Measurements
    m_gpsHeader = msg->header;

    ZMatrix[0]=msg->latitude;
    ZMatrix[1]=msg->longitude;

    m_ZMatrix=ZMatrix;

    // measurement variance R matrix
    m_RMatrix << msg->err_horz , 0, // From GPS
                 0 , msg->err_vert;

}

/**
   *This function calls the functions that update the system state and covariance matrix.
   */
void KF::update()
{
    m_KGain = KalmanGainCalculation(m_predictPMatrix);
    m_currentSystemState = updateCurrentState(m_predictionSystemState);
    m_currentPMatrix = updateCurrentEstimateUncertainty(m_predictPMatrix);

}


/**
 * This function predicts the system state and creates an array of this data for RMSE calculation.
 *
 * @param currentSystemState Current System state.
 *
 * @return Prediction of System state.
 */
Eigen::VectorXd KF::systemStatesEquation(Eigen::VectorXd  currentSystemState) 
{

    Eigen::VectorXd predictionSystemState = Eigen::VectorXd::Zero(6);

    predictionSystemState = m_FMatrix * currentSystemState;
    m_predictionSystemStateArray(0,m_temp) = predictionSystemState[0];
    m_predictionSystemStateArray(1,m_temp) = predictionSystemState[3];

    return predictionSystemState;     
}

/**
 * This function predicts the covariance matrix of the system state.
 *
 * @param currentCovarianceMatrix Current Covariance matrix
 *
 * @return Prediction of Covariance matrix
 */
Eigen::MatrixXd KF::covarianceExtrapolationEquation(Eigen::MatrixXd currentCovarianceMatrix)
{

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

Eigen::MatrixXd  KF::KalmanGainCalculation(Eigen::MatrixXd predictPMatrix)
{
    Eigen::MatrixXd KGain = Eigen::MatrixXd::Zero(6, 2);

    KGain = predictPMatrix*m_HMatrix.transpose()*((m_HMatrix*predictPMatrix*(m_HMatrix.transpose())+m_RMatrix).inverse());

    return KGain;
    
}

/**
 * This function updated system state.
 *
 * @param estimationSystemState Estimation System State.
 *
 * @return  Estimated System State
 */
Eigen::VectorXd  KF::updateCurrentState(Eigen::VectorXd estimationSystemState)
{
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
Eigen::MatrixXd  KF::updateCurrentEstimateUncertainty(Eigen::MatrixXd estimationPMatrix)
{
    estimationPMatrix = Eigen::MatrixXd::Zero(6, 6);
    estimationPMatrix = (m_I - m_KGain * m_HMatrix) * m_predictPMatrix * (m_I - m_KGain * m_HMatrix).transpose() + m_KGain * m_RMatrix * m_KGain.transpose();
    m_estimationPMatrix = estimationPMatrix;

    return estimationPMatrix; 
}

/**
 * This function calculates  RMSE.
 *
 * @param measurements Vector of GPS measurements
 * @param estimateStates Vector of estimated measurements
 *
 * @return  RMSE
 */
Eigen::VectorXd KF::calculateRMSE(Eigen::MatrixXd estimateStates, Eigen::VectorXd measurements)
{
    // A vector was created for RMSE values.
    Eigen::VectorXd RMSEs = Eigen::VectorXd::Zero(2);
    // The initial value is assigned to the sum of the differences between sensor measurements and estimated values.
    double sumSquaredDiffLong = 0.0;
    double sumSquaredDiffLat = 0.0;
    

    // Calculate the sum of the differences between sensor measurements and estimated values.
    for (int i = 0; i <= m_temp; ++i) 
    {

        double diffLong = measurements[0]- estimateStates(0,i);
        double diffLat = measurements[1]- estimateStates(1,i);
        sumSquaredDiffLong += pow(diffLong,2);
        sumSquaredDiffLat += pow(diffLat,2);
       
    }
    
    // The RMSE value was calculated by finding the square root of the mean sum of differences.
    double meanSquaredDiffLong = sumSquaredDiffLong / m_temp;
    double rmseLong = std::sqrt(meanSquaredDiffLong);
    double meanSquaredDiffLat = sumSquaredDiffLat / m_temp;
    double rmseLat = std::sqrt(meanSquaredDiffLat);
  

    // The calculated RMSE values were assigned to the vector.
    RMSEs[0] = rmseLong;
    RMSEs[1] = rmseLat;
 
    
    // Debugging the RMSE values.
    if (m_temp==1000 && debug)
    {    
        std::cout << "RMSEs: " << RMSEs << "\n";
        std::cout << "--------------------------------------------------------------\n";

    }

    m_temp+=1;

    if (m_temp == 1001)
    {
        m_temp=0;
    }

    return RMSEs;
}

/**
 * This function publishes results and RMSE.
 *
 */
void KF::publishResults()
{

    int temp = 0;

    gps_common::GPSFix processed_msg;
    gps_common::GPSFix rmse;

    processed_msg.header = m_gpsHeader;
    processed_msg.latitude = m_currentSystemState[0];
    processed_msg.longitude = m_currentSystemState[3];

    rmse.header = m_gpsHeader;
    rmse.longitude = m_errorVector(0,temp);
    rmse.latitude = m_errorVector(1,temp);

    gpsPub.publish(processed_msg);
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
void KF::debugging()
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
        std::cout << "Process Noise Matrix: \n" << m_QMatrix << "\n";
        std::cout << "--------------------------------------------------------------\n";
        std::cout << "RMSE: \n" << m_errorVector << "\n";
        std::cout << "--------------------------------------------------------------\n";
     
    }

}