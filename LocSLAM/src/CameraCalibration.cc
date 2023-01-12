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

#include "CameraCalibration.h"
#include "Tracking.h"

#include <mutex>

namespace LocSLAM
{

    CameraCalibration::CameraCalibration(Tracking *pTracker) : mpTracker(pTracker)
    {

        // Fixed map transformation
        mCalibrationState = NOT_INITIALIZED;

        // Extrinsic calibration
        mvCameraCalibration.resize(mpTracker->mnCameras);
        for (auto it = mvCameraCalibration.begin(); it != mvCameraCalibration.end(); ++it)
        {
            *it = cv::Mat::eye(4, 4, CV_32F);
        }

        mvCovariance.resize(mpTracker->mnCameras);

        Eigen::VectorXd sigma2(6);
        sigma2 << mSigma2Head, mSigma2Head, mSigma2Head, mSigma2Pos, mSigma2Pos, mSigma2Pos;
        for (auto it = mvCovariance.begin(); it != mvCovariance.end(); ++it)
        {
            *it = sigma2.asDiagonal();
        }
    }

    void CameraCalibration::FixateCalibration(bool bFixate)
    {
        if (bFixate && mCalibrationState >= INITIALIZED)
            mCalibrationState = FIXATED;
    }

    vector<cv::Mat> CameraCalibration::GetCalibration()
    {
        return mvCameraCalibration;
    }

    vector<Eigen::Matrix<double, 6, 6>> CameraCalibration::GetCovariance()
    {
        return mvCovariance;
    }

    void CameraCalibration::UpdateCalibration(vector<cv::Mat> newCalib, vector<Eigen::Matrix<double, 6, 6>> newCov)
    {
        int nCameras = mpTracker->mnCameras;

        if (mCalibrationState == NOT_INITIALIZED)
            mCalibrationState = INITIALIZED;

        // Update Extrinsic parameters
        if (mCalibrationState != FIXATED)
        {
            int i = 0;
            for (auto it = newCalib.begin(); it != newCalib.end(); ++it)
            {
                cout << "UpdateCalibration: Calibration for camera " << i << " :" << endl
                     << *it * mpTracker->GetExtrinsics(i) << endl;
                mpTracker->UpdateExtrinsics(i++, *it);
            }

            mvCovariance = newCov;

            // Check if we should fixate
            double meanCovarianceVolume = 0.0;
            for (auto it = mvCovariance.begin(); it != mvCovariance.end(); ++it)
            {
                meanCovarianceVolume = meanCovarianceVolume + it->determinant() / (double)nCameras;
            }

            cout << "Calibration Volume: " << meanCovarianceVolume << endl;
            FixateCalibration(pow(meanCovarianceVolume, 1.0 / 6.0) < fixateThreshold);
        }
    }

    void CameraCalibration::Reset()
    {
        for (auto it = mvCameraCalibration.begin(); it != mvCameraCalibration.end(); ++it)
        {
            *it = cv::Mat::eye(4, 4, CV_32F);
        }
    }

} // namespace LocSLAM
