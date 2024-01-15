#pragma once 

#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <vector>
#include <Eigen/Dense>
#include <gps_common/GPSFix.h>
#include <sensor_msgs/Imu.h>
#include "kf/EkfMsg.h"


class EKF
{
    public:
        EKF(int argc, char **argv);
        EKF();

    private:

        Eigen::VectorXd  systemStatesEquation(Eigen::VectorXd);
        Eigen::MatrixXd  covarianceExtrapolationEquation(Eigen::MatrixXd);
        Eigen::MatrixXd  KalmanGainCalculation(Eigen::MatrixXd);
        Eigen::VectorXd  updateCurrentState(Eigen::VectorXd); 
        Eigen::MatrixXd  updateCurrentEstimateUncertainty(Eigen::MatrixXd);


        void gpsCallback(const gps_common::GPSFix::ConstPtr&);
        void imuCallback(const sensor_msgs::Imu::ConstPtr&);

        void matricesInitializer();
        void publishPosition();
        void debugging();
        Eigen::VectorXd calculateError(Eigen::VectorXd, Eigen::VectorXd);
        void predict();
        void update();
        void measurementGps(const gps_common::GPSFix::ConstPtr&);
        void measurementImu(const sensor_msgs::Imu::ConstPtr&);
        void measurement();

        ros::NodeHandle* n;
        ros::Subscriber gpsSub;
        ros::Subscriber imuSub;
        ros::Publisher ekfPub; 

        // Delta t
        double m_dt = 0.01;

        // Procces Noises
        // double m_gpsNoiseP = 0.5*8.8*pow(m_dt, 2); // X and Y
        double m_gpsNoiseP = 0.1; // X and Y
        double m_phiNoiseP = 1.0*m_dt; // Yaw
        double m_velocityNoiseP = 8.8*m_dt; // Velocity
        double m_phiDotNoiseP = 0.1*m_dt; // Yaw rate
        double m_accNoiseP = 0.5; // Accelearation


        // Debugging check
        bool debug;

        // Error vector
        Eigen::VectorXd m_errorVector = Eigen::VectorXd::Zero(6);

        // System State vectors - Xn,n
        Eigen::VectorXd m_currentSystemState =  Eigen::VectorXd::Zero(6);
        Eigen::VectorXd m_predictionSystemState =  Eigen::VectorXd::Zero(6);
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












 



};
