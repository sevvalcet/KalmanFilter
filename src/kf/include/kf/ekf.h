#pragma once 

#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <vector>
#include <Eigen/Dense>
#include <gps_common/GPSFix.h>
#include <sensor_msgs/Imu.h>
#include "kf/EkfMsg.h"
#include <std_msgs/Header.h>
#include <numeric>


class EKF
{
    public:
    
        EKF(int argc, char **argv);  // Constructor
        EKF();  // Default Constructor

    private:

        // Functions 
        Eigen::VectorXd  systemStatesEquation(Eigen::VectorXd);
        Eigen::MatrixXd  covarianceExtrapolationEquation(Eigen::MatrixXd);
        Eigen::MatrixXd  KalmanGainCalculation(Eigen::MatrixXd);
        Eigen::VectorXd  updateCurrentState(Eigen::VectorXd); 
        Eigen::MatrixXd  updateCurrentEstimateUncertainty(Eigen::MatrixXd);
        Eigen::VectorXd calculateRMSE(std::vector<gps_common::GPSFix>, std::vector<sensor_msgs::Imu> , Eigen::MatrixXd);

        void gpsCallback(const gps_common::GPSFix::ConstPtr&);
        void imuCallback(const sensor_msgs::Imu::ConstPtr&);

        void matricesInitializer();
        void publishResults();
        void debugging();
        void predict();
        void update();
        void measurementGps(const gps_common::GPSFix::ConstPtr&);
        void measurementImu(const sensor_msgs::Imu::ConstPtr&);
        void measurement();

        // ROS
        ros::NodeHandle* n;
        ros::Subscriber gpsSub;
        ros::Subscriber imuSub;
        ros::Publisher ekfPub; 
        ros::Publisher rmsePub;

        // Common Header
        std_msgs::Header m_gpsHeader;

        // Delta t
        double m_dt;

        // Procces Noises
        double m_gpsNoiseP = 0.1; // X and Y
        double m_phiNoiseP = 1.0*m_dt; // Yaw
        double m_velocityNoiseP = 8.8*m_dt; // Velocity
        double m_phiDotNoiseP = 0.1*m_dt; // Yaw rate
        double m_accNoiseP = 0.5; // Accelearation


        // Debugging check
        bool debug;

        // Error vector
        Eigen::VectorXd m_RMSE = Eigen::VectorXd::Zero(6);

        // System State vectors - Xn,n
        Eigen::VectorXd m_currentSystemState =  Eigen::VectorXd::Zero(6);
        Eigen::VectorXd m_predictionSystemState =  Eigen::VectorXd::Zero(6);
        Eigen::MatrixXd m_predictionSystemStateArray =  Eigen::MatrixXd::Zero(6,1005);
        Eigen::VectorXd m_estimationSystemState =  Eigen::VectorXd::Zero(6);
        
        // Covariance matrices - Pn,n
        Eigen::MatrixXd m_currentPMatrix = Eigen::MatrixXd::Zero(6, 6);
        Eigen::MatrixXd m_predictPMatrix = Eigen::MatrixXd::Zero(6, 6);
        Eigen::MatrixXd m_estimationPMatrix = Eigen::MatrixXd::Zero(6, 6);


        // Procces noise matrix - Q
        Eigen::MatrixXd m_QMatrix = Eigen::MatrixXd::Zero(6, 6);
        
        // State transition matrix - F
        Eigen::MatrixXd m_FMatrix = Eigen::MatrixXd::Zero(6,6);  

        // Kalman Gain Calculation - K
        Eigen::MatrixXd m_KGain = Eigen::MatrixXd::Zero(6,6); 

        // Observation Matrix - H
        Eigen::MatrixXd m_HMatrix = Eigen::MatrixXd::Zero(6,6); 

        // Measurement Covariance Matrix - R
        Eigen::MatrixXd m_RMatrix = Eigen::MatrixXd::Zero(6,6); 
    
        // Measurement of GPS
        Eigen::VectorXd m_gpsMeasurement =  Eigen::VectorXd::Zero(6);

        // Measurement of IMU
        Eigen::VectorXd m_imuMeasurement =  Eigen::VectorXd::Zero(6);

        // Measurement values
        Eigen::VectorXd m_ZMatrix =  Eigen::VectorXd::Zero(6);

        // Identity Matrix - I
        Eigen::MatrixXd m_I = Eigen::MatrixXd::Identity(6, 6);

        // IMU' and GPS' data vector for calculating the RMSE
        std::vector<sensor_msgs::Imu> m_imuVec;
        std::vector<gps_common::GPSFix> m_gpsVec;

        // Data limit for RMSE calculation
        int m_temp;


};
