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

#ifndef CAMERACALIBRATION_H
#define CAMERACALIBRATION_H

#include "Tracking.h"

#include <mutex>

namespace LocSLAM
{

    class Tracking;

    class CameraCalibration
    {

    public:
        CameraCalibration(Tracking *pTracker);

        bool IsFixated() { return (mCalibrationState == FIXATED); };

        void FixateCalibration(bool bFixate);

        vector<cv::Mat> GetCalibration();
        vector<Eigen::Matrix<double, 6, 6>> GetCovariance();

        void UpdateCalibration(vector<cv::Mat> vCalib, vector<Eigen::Matrix<double, 6, 6>> vCov);

        void Reset();

    protected:
        Tracking *mpTracker;

        // Status of calibration
        enum eCalibrationState
        {
            NOT_INITIALIZED = 0,
            INITIALIZED = 1,
            FIXATED = 2
        };
        eCalibrationState mCalibrationState;

        const float mSigma2Pos = pow(0.005, 2.0);
        const float mSigma2Head = pow(0.5 * CV_PI / 180.0, 2.0);

        // Camera extrinsic calibration
        vector<cv::Mat> mvCameraCalibration;
        vector<Eigen::Matrix<double, 6, 6>> mvCovariance;
        const double fixateThreshold = 1e-5;
    };

} // namespace LocSLAM

#endif // CAMERACALIBRATION_H
