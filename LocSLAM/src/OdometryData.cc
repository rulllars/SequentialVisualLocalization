/**
* This file is part of LocSLAM.
*
* Copyright (C) 2021 Lars Hammarstrand <lars.hammarstrand at chalmers dot se> (Chalmers University of Technology)
* For more information see <https://github.com/rulllars/SequentialLocalization>
*
* LocSLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LocSLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with locSLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include "OdometryData.h"
#include "Converter.h"
#include <thread>

#include <unsupported/Eigen/MatrixFunctions>

using namespace std;

namespace LocSLAM
{

    OdometryData::OdometryData()
    {
        mTimeStamp = 0.0;
        mPredictionTime = 0.0;

        mTransVelocity = cv::Mat::zeros(3, 1, CV_32F);
        mRotVelocity = cv::Mat::zeros(3, 1, CV_32F);

        mT21 = cv::Mat::eye(4, 4, CV_32F);
        mT21.colRange(0, 3).rowRange(0, 3).copyTo(mR21);
        mT21.col(3).rowRange(0, 3).copyTo(mt21);

        mSigmaOmegaX = 0.02;
        mSigmaOmegaY = 0.02;
        mSigmaOmegaZ = 0.02;
        mSigmaTransX = .08;
        mSigmaTransY = .08;
        mSigmaTransZ = .08;

        mCovarianceMatrix = Eigen::MatrixXd::Zero(6, 6);
    }

    void OdometryData::Clear()
    {
        mTimeStamp = 0.0;
        mPredictionTime = 0.0;
        mTransVelocity = cv::Mat::zeros(3, 1, CV_32F);
        mRotVelocity = cv::Mat::zeros(3, 1, CV_32F);
        mT21 = cv::Mat::eye(4, 4, CV_32F);

        mCovarianceMatrix = Eigen::MatrixXd::Zero(6, 6);
    }

    OdometryData::OdometryData(double time, double dt,
                               float v_x, float v_y, float v_z,
                               float w_x, float w_y, float w_z,
                               double sigmaAngle, double sigmaTrans)
        : mTimeStamp(time), mSigmaOmegaX(sigmaAngle), mSigmaOmegaY(sigmaAngle), mSigmaOmegaZ(sigmaAngle),
          mSigmaTransX(sigmaTrans), mSigmaTransY(sigmaTrans), mSigmaTransZ(sigmaTrans)
    {

        // Check that prediction time is non-zero
        if (dt < 0)
        {
            cout << "OdometryData: Negative prediction time, " << dt << "!" << endl;
            mPredictionTime = 1e-6;
        }
        else
            mPredictionTime = dt;

        // Check if we are moving forward
        const double sigmaSpeed = sqrt(pow(mSigmaOmegaX, 2) + pow(mSigmaOmegaX, 2) + pow(mSigmaOmegaX, 2));
        double speed = sqrt(pow(v_x, 2) + pow(v_y, 2) + pow(v_z, 2));
        if (speed > 2 * sigmaSpeed)
        {
            // Store translational velocities
            mTransVelocity = cv::Mat::zeros(3, 1, CV_32F);
            mTransVelocity.at<float>(0) = v_x;
            mTransVelocity.at<float>(1) = v_y;
            mTransVelocity.at<float>(2) = v_z;

            // Translation vector t12 (1: previous frame, 2: current frame)
            cv::Mat t12 = dt * mTransVelocity;

            // Store rotational velocities
            mRotVelocity = cv::Mat::zeros(3, 1, CV_32F);
            mRotVelocity.at<float>(0) = w_x;
            mRotVelocity.at<float>(1) = w_y;
            mRotVelocity.at<float>(2) = w_z;

            // Rotation matrix R12 (1: previous frame, 2: current frame)
            Eigen::Matrix3d rotVel = Eigen::Matrix3d::Zero(3, 3);

            rotVel(0, 1) = -dt * w_z;
            rotVel(0, 2) = dt * w_y;
            rotVel(1, 2) = -dt * w_x;

            Eigen::Matrix3d expmat = rotVel - rotVel.transpose();

            expmat = expmat.exp();
            cv::Mat R12 = Converter::toCvMat(expmat);

            // Transformation matrix T12 (1: previous frame, 2: current frame)
            cv::Mat T12 = cv::Mat::eye(4, 4, CV_32F);
            R12.copyTo(T12.colRange(0, 3).rowRange(0, 3));
            t12.copyTo(T12.col(3).rowRange(0, 3));

            // Transformation matrix T21 (1: previous frame, 2: current frame)
            mT21 = T12.inv();
            mR21 = mT21.colRange(0, 3).rowRange(0, 3);
            mt21 = mT21.col(3).rowRange(0, 3);

            mCovarianceMatrix = Eigen::MatrixXd::Zero(6, 6);
            double sigmaTime = fmax(mPredictionTime, 1e-9);
            mCovarianceMatrix(0, 0) = sigmaTime * mSigmaTransX * mSigmaTransX;
            mCovarianceMatrix(1, 1) = sigmaTime * mSigmaTransY * mSigmaTransY;
            mCovarianceMatrix(2, 2) = sigmaTime * mSigmaTransZ * mSigmaTransZ;
            mCovarianceMatrix(3, 3) = sigmaTime * mSigmaOmegaX * mSigmaOmegaX;
            mCovarianceMatrix(4, 4) = sigmaTime * mSigmaOmegaY * mSigmaOmegaY;
            mCovarianceMatrix(5, 5) = sigmaTime * mSigmaOmegaZ * mSigmaOmegaZ;
        }
        else
        {
            //cout << "OdoCopy; Near zero odometry detected! -> Assume stationary!" << endl;

            // Store translational velocities
            mTransVelocity = cv::Mat::zeros(3, 1, CV_32F);

            // Store rotational velocities
            mRotVelocity = cv::Mat::zeros(3, 1, CV_32F);

            // Rotation matrix R12 (1: previous frame, 2: current frame)
            Eigen::Matrix3d rotVel = Eigen::Matrix3d::Zero(3, 3);

            // Transformation matrix T21 (1: previous frame, 2: current frame)
            mT21 = cv::Mat::eye(4, 4, CV_32F);
            mR21 = mT21.colRange(0, 3).rowRange(0, 3);
            mt21 = mT21.col(3).rowRange(0, 3);

            mCovarianceMatrix = Eigen::MatrixXd::Zero(6, 6);
            double sigmaTime = fmax(mPredictionTime, 1e-9);
            float scaleFactor = 1.5;
            mCovarianceMatrix(0, 0) = scaleFactor * sigmaTime * mSigmaTransX * mSigmaTransX;
            mCovarianceMatrix(1, 1) = scaleFactor * sigmaTime * mSigmaTransY * mSigmaTransY;
            mCovarianceMatrix(2, 2) = scaleFactor * sigmaTime * mSigmaTransZ * mSigmaTransZ;
            mCovarianceMatrix(3, 3) = scaleFactor * sigmaTime * mSigmaOmegaX * mSigmaOmegaX;
            mCovarianceMatrix(4, 4) = scaleFactor * sigmaTime * mSigmaOmegaY * mSigmaOmegaY;
            mCovarianceMatrix(5, 5) = scaleFactor * sigmaTime * mSigmaOmegaZ * mSigmaOmegaZ;
        }

        CheckCovariance();
    }

    OdometryData::OdometryData(const OdometryData &odo)
        : mTimeStamp(odo.mTimeStamp), mPredictionTime(odo.mPredictionTime), mR21(odo.mR21.clone()),
          mt21(odo.mt21.clone()), mT21(odo.mT21.clone()), mTransVelocity(odo.mTransVelocity.clone()),
          mRotVelocity(odo.mRotVelocity.clone()), mCovarianceMatrix(odo.mCovarianceMatrix),
          mSigmaOmegaX(odo.mSigmaOmegaX), mSigmaOmegaY(odo.mSigmaOmegaY), mSigmaOmegaZ(odo.mSigmaOmegaZ),
          mSigmaTransX(odo.mSigmaTransX), mSigmaTransY(odo.mSigmaTransY), mSigmaTransZ(odo.mSigmaTransZ)
    {
        //Ensure mCovarianceMatrix is positive definite
        CheckCovariance();
    }

    void OdometryData::Print()
    {
        cout << "Odometry:" << endl
             << mT21 << endl;
    }

    OdometryData OdometryData::Integrate(const OdometryData &first, const OdometryData &last)
    {
        OdometryData integratedOdo;

        // Set time stamp to last object
        integratedOdo.mTimeStamp = last.mTimeStamp;

        // Sum prediction length
        integratedOdo.mPredictionTime = first.mPredictionTime + last.mPredictionTime;
        float dt = integratedOdo.mPredictionTime;

        // Calculate cascaded transformation
        integratedOdo.mT21 = last.mT21 * first.mT21;

        // Update translation and rotational velocities
        integratedOdo.mT21.rowRange(0, 3).col(3).copyTo(integratedOdo.mt21);
        mTransVelocity = integratedOdo.mt21 / dt;

        Eigen::Matrix3d R21 = Converter::toMatrix3d(integratedOdo.mT21.rowRange(0, 3).colRange(0, 3));
        Eigen::Matrix3d logmat = R21.log();

        integratedOdo.mRotVelocity.at<float>(0) = -logmat(1, 2) / dt; // w_x
        integratedOdo.mRotVelocity.at<float>(1) = logmat(0, 2) / dt;  // w_y
        integratedOdo.mRotVelocity.at<float>(2) = -logmat(0, 1) / dt; // w_z

        // Update Covariance matrix

        // Should use Jacobians (J*C*J^T) but using simple method (I*C*I^T) instead. Probably little effect on the end result.
        integratedOdo.mCovarianceMatrix = first.mCovarianceMatrix + last.mCovarianceMatrix;

        // Check to ensure mCovarianceMatrix is positive definite
        CheckCovariance();

        return integratedOdo;
    }

    // ****************************************************************
    // Unverified code to generate jacobians for the prediction model.
    // Needs to be checked and probably corrected before use,
    // ****************************************************************
    /*
    Eigen::Matrix<double, 6, 6> OdometryData::PoseJacobian(cv::Mat _R, cv::Mat _t) const
    {
        Eigen::Matrix<double, 6, 6> J;

        // Get linearization point of in for of rotation and translation
        Eigen::Matrix3d R = Converter::toMatrix3d(_R);
        Eigen::Vector3d t = Converter::toVector3d(_t);

        // Get odometry rotation and translation
        Eigen::Matrix3d R21 = Converter::toMatrix3d(mR21);
        Eigen::Vector3d t21 = Converter::toVector3d(mt21);

        // Construct jacobian

        // Rotation in pose 2 by rotation in pose 1
        Eigen::Matrix3d J_dw2dw1 = R21 * R * Converter::skew(Eigen::Vector3d::Ones());

        // Translation in pose 2 by rotation in pose 1
        Eigen::Matrix3d J_dw2dt1 = Eigen::Matrix3d::Zero();

        // Translation in pose 2 by rotation in pose 1
        Eigen::Matrix3d J_dt2dw1 = Eigen::Matrix3d::Zero();

        // Translation in pose 2 by translation in pose 1
        Eigen::Matrix3d J_dt2dt1 = R21;

        // Construct jacobian
        J.block<3, 3>(0, 0) = J_dw2dw1;
        J.block<3, 3>(3, 0) = J_dt2dw1;
        J.block<3, 3>(0, 3) = J_dw2dt1;
        J.block<3, 3>(3, 3) = J_dt2dt1;

        return J;
    }
    
    Eigen::Matrix<double, 6, 6> OdometryData::NoiseJacobian(cv::Mat _R, cv::Mat _t) const
    {
        Eigen::Matrix<double, 6, 6> J;

        // Get linearization point of in for of rotation and translation
        Eigen::Matrix3d R = Converter::toMatrix3d(_R);
        Eigen::Vector3d t = Converter::toVector3d(_t);

        // Get odometry rotation and translation
        Eigen::Matrix3d R21 = Converter::toMatrix3d(mR21);
        Eigen::Vector3d t21 = Converter::toVector3d(mt21);

        // Construct jacobian

        // Rotation in pose 2 by rotation in pose 1
        Eigen::Matrix3d J_dw2dqw = R21 * Converter::skew(Eigen::Vector3d::Ones()) * R;

        // Translation in pose 2 by rotation in pose 1
        Eigen::Matrix3d J_dw2dqv = Eigen::Matrix3d::Zero();

        // Translation in pose 2 by rotation in pose 1
        Eigen::Matrix3d J_dt2dqw = -R21 * Converter::skew(t + t21);

        // Translation in pose 2 by translation in pose 1
        Eigen::Matrix3d J_dt2dqv = -R21;

        // Construct jacobian
        J.block<3, 3>(0, 0) = J_dw2dqw;
        J.block<3, 3>(3, 0) = J_dw2dqv;
        J.block<3, 3>(0, 3) = J_dt2dqw;
        J.block<3, 3>(3, 3) = J_dt2dqv;

        return J;
    }
    */

    void OdometryData::Integrate(const OdometryData &last)
    {

        // Set time stamp to last object
        mTimeStamp = last.mTimeStamp;

        // Sum prediction length
        mPredictionTime = mPredictionTime + last.mPredictionTime;
        double dt = mPredictionTime;

        mSigmaOmegaX = last.mSigmaOmegaX;
        mSigmaOmegaY = last.mSigmaOmegaY;
        mSigmaOmegaZ = last.mSigmaOmegaZ;
        mSigmaTransX = last.mSigmaTransX;
        mSigmaTransY = last.mSigmaTransY;
        mSigmaTransZ = last.mSigmaTransZ;

        // Check if vehicle is moving
        // Calculate cascaded transformation
        mT21 = last.mT21 * mT21;

        // Update translation and rotational velocities
        mT21.rowRange(0, 3).col(3).copyTo(mt21);
        mTransVelocity = -mt21 / dt;

        mT21.rowRange(0, 3).colRange(0, 3).copyTo(mR21);
        Eigen::Matrix3d R21 = Converter::toMatrix3d(mR21);
        R21.transposeInPlace();
        Eigen::Matrix3d logmat = R21.log();

        mRotVelocity.at<float>(0) = -logmat(1, 2) / dt; // w_x
        mRotVelocity.at<float>(1) = logmat(0, 2) / dt;  // w_y
        mRotVelocity.at<float>(2) = -logmat(0, 1) / dt; // w_z

        // Update Covariance matrix using simple (I*C*I) instead of proper linearized model
        mCovarianceMatrix = mCovarianceMatrix + last.mCovarianceMatrix;

        //Ensure mCovarianceMatrix is positive definite
        CheckCovariance();
    }

    void OdometryData::RemoveFirst(const OdometryData &odo)
    {

        // Set time stamp to last object
        mTimeStamp = mTimeStamp;

        // Sum prediction length
        mPredictionTime = mPredictionTime - odo.mPredictionTime;

        if (mPredictionTime < 0)
        {
            cout << "OdoRemoveFirst: Negative prediction time; " << mPredictionTime << "!" << endl;
            mPredictionTime = 1e-6;
            return;
        }
        double dt = mPredictionTime;

        // Calculate cascaded transformation
        mT21 = mT21 * odo.mT21.inv();

        // Update translation and rotational velocities
        mT21.rowRange(0, 3).col(3).copyTo(mt21);
        mTransVelocity = -mt21 / dt;

        mT21.rowRange(0, 3).colRange(0, 3).copyTo(mR21);
        Eigen::Matrix3d R21 = Converter::toMatrix3d(mR21);
        R21.transposeInPlace();
        Eigen::Matrix3d logmat = R21.log();

        mRotVelocity.at<float>(0) = -logmat(1, 2) / dt; // w_x
        mRotVelocity.at<float>(1) = logmat(0, 2) / dt;  // w_y
        mRotVelocity.at<float>(2) = -logmat(0, 1) / dt; // w_z

        mCovarianceMatrix = mCovarianceMatrix - odo.mCovarianceMatrix;

        //Ensure mCovarianceMatrix is positive definite
        CheckCovariance();
    }

    void OdometryData::RemoveLast(const OdometryData &odo)
    {

        // Set time stamp to last object
        mTimeStamp = mTimeStamp - odo.mPredictionTime;

        // Sum prediction length
        mPredictionTime = mPredictionTime - odo.mPredictionTime;
        if (mPredictionTime < 0)
        {
            cout << "ÓdoRemoveLast: Negative prediction time; " << mPredictionTime << "!" << endl;
            mPredictionTime = 1e-6;
        }
        double dt = mPredictionTime;

        // Calculate cascaded transformation
        mT21 = odo.mT21.inv() * mT21;

        // Update translation and rotational velocities
        mT21.rowRange(0, 3).col(3).copyTo(mt21);
        mTransVelocity = -mt21 / dt;

        mT21.rowRange(0, 3).colRange(0, 3).copyTo(mR21);
        Eigen::Matrix3d R21 = Converter::toMatrix3d(mR21);
        R21.transposeInPlace();
        Eigen::Matrix3d logmat = R21.log();

        mRotVelocity.at<float>(0) = -logmat(1, 2) / dt; // w_x
        mRotVelocity.at<float>(1) = logmat(0, 2) / dt;  // w_y
        mRotVelocity.at<float>(2) = -logmat(0, 1) / dt; // w_z

        mCovarianceMatrix = mCovarianceMatrix - odo.mCovarianceMatrix;

        //Ensure mCovarianceMatrix is positive definite
        CheckCovariance();
    }

    bool OdometryData::CheckCovariance()
    {

        double eigenThreshold = 1e-12;

        //Ensure mCovarianceMatrix is positive definite
        Eigen::EigenSolver<Eigen::Matrix<double, 6, 6>> eigensolver(mCovarianceMatrix);
        double minEigenVal = eigensolver.eigenvalues().real().minCoeff();

        bool initialCondition = minEigenVal < eigenThreshold;

        while (minEigenVal < eigenThreshold)
        {
            mCovarianceMatrix = mCovarianceMatrix + Eigen::MatrixXd::Identity(6, 6) * fmax(eigenThreshold, -minEigenVal);
            Eigen::EigenSolver<Eigen::Matrix<double, 6, 6>> eigensol(mCovarianceMatrix);
            minEigenVal = eigensol.eigenvalues().real().minCoeff();
        }

        //if (initialCondition)
        //    cout << "OdoCheckCov; Covariance badly conditioned! Adjusted!" << endl;

        return initialCondition;
    }

} // namespace LocSLAM
