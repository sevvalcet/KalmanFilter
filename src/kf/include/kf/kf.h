#pragma once 

#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <vector>
#include <Eigen/Dense>
#include <gps_common/GPSFix.h>

class KF
{
    public:
        KF(int argc, char **argv);
        KF();

    private:

        Eigen::VectorXd  systemStatesEquation(Eigen::VectorXd);
        Eigen::MatrixXd  covarianceExtrapolationEquation(Eigen::MatrixXd);
        Eigen::MatrixXd  KalmanGainCalculation(Eigen::MatrixXd);
        Eigen::VectorXd  updateCurrentState(Eigen::VectorXd); 
        Eigen::MatrixXd  updateCurrentEstimateUncertainty(Eigen::MatrixXd);



        void matricesInitializer();
        void gpsCallback(const gps_common::GPSFix::ConstPtr& msg );
        void publishPosition();
        void debugging();
        Eigen::VectorXd calculateError(Eigen::VectorXd, Eigen::VectorXd);
        void predict();
        void update();
        void measurement(const gps_common::GPSFix::ConstPtr& msg);

        ros::NodeHandle* n;
        ros::Subscriber gpsSub;
        ros::Publisher gpsPub; 

        // Delta t
        double m_dt = 0.01;

        // Variance of acceleration
        double m_accVariance = 0.04;

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
        Eigen::MatrixXd m_KGain = Eigen::MatrixXd::Zero(6,2); 

        // Observation Matrix - H
        Eigen::MatrixXd m_HMatrix = Eigen::MatrixXd::Zero(2,6); 

        // Measurement Covariance Matrix - R
        Eigen::MatrixXd m_RMatrix = Eigen::MatrixXd::Zero(2,2); 
        
        // Measurement values
        Eigen::VectorXd m_ZMatrix =  Eigen::Vector2d::Zero(2);

        // Identity Matrix - I
        Eigen::MatrixXd m_I = Eigen::MatrixXd::Identity(6, 6);












 



};
