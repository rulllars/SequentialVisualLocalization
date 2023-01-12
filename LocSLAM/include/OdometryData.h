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

#ifndef ODOMETRYDATA_H
#define ODOMETRYDATA_H

#include <opencv2/core/core.hpp>
#include <Eigen/Dense>

#include "Converter.h"

namespace LocSLAM
{

    // Odometry information class
    class OdometryData
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        // Empty constructor
        OdometryData();

        // Clear data
        void Clear();

        // Data constructor
        OdometryData(double time, double dt,
                     float v_x, float v_y, float v_z,
                     float w_x, float w_y, float w_z,
                     double sigmaAngle, double sigmaTrans);

        // Copy constructor
        OdometryData(const OdometryData &odo);

        // Print to stdio
        void Print();

        // Return odometry transformation SE(3)
        cv::Mat GetTransformation() const { return mT21.clone(); };
        //g2o::SE3Quat GetSE3QuatTransformation() const { return Converter::toSE3Quat(mT21); };

        // Return covariance for odometry transformation SE(3)
        // described as a 6D vector representing axis angle and translation
        Eigen::Matrix<double, 6, 6> GetCovariance() const { return mCovarianceMatrix; };

        // Predict a SE(3) pose using odometry
        cv::Mat Predict(cv::Mat Tbw) { return mT21 * Tbw; };

        // Jacobians
        //Eigen::Matrix<double, 6, 6> PoseJacobian(cv::Mat _R, cv::Mat _t) const;
        //Eigen::Matrix<double, 6, 6> NoiseJacobian(cv::Mat _R, cv::Mat _t) const;

        // Odometry (pre-)integration from first to last
        OdometryData Integrate(const OdometryData &first, const OdometryData &last);

        // Odometry integration with this class object
        void Integrate(const OdometryData &last);

        // Remove odometry
        void RemoveFirst(const OdometryData &odo);
        void RemoveLast(const OdometryData &odo);

        bool CheckCovariance();

        // time stamp of measurement
        double mTimeStamp;

        // Time since last measurement
        double mPredictionTime;

    private:
        // Translational velocity v_x v_y v_z
        cv::Mat mTransVelocity;

        // Rotational velocity around x, y, and z axis
        cv::Mat mRotVelocity;

        // Transformation matrix from previous to current body pose.
        cv::Mat mT21;

        // Translation vector
        cv::Mat mt21;

        // Rotation vector
        cv::Mat mR21;

        // Covariance matrix in axis angle and translation
        Eigen::Matrix<double, 6, 6> mCovarianceMatrix;

        // Default standard deviation values for 1s prediction
        double mSigmaOmegaX;
        double mSigmaOmegaY;
        double mSigmaOmegaZ;

        double mSigmaTransX;
        double mSigmaTransY;
        double mSigmaTransZ;
    };

} // namespace LocSLAM
#endif