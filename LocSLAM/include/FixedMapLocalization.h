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

#ifndef FIXEDMAPLOCALIZATION_H
#define FIXEDMAPLOCALIZATION_H

#include "KeyFrame.h"
#include "Map.h"
#include "CameraCalibration.h"

#include <mutex>

namespace LocSLAM
{
    class CameraCalibration;

    class FixedMapLocalization
    {

    public:
        FixedMapLocalization(Map *pFixedMap, Map *pLocalMap, CameraCalibration *pCameraCaliprater);

        bool CheckNewFixedMapMatches();
        void ProcessNewKeyFrame();
        void InsertNewKeyFrame(KeyFrame *pKF);
        bool CheckNewKeyFrames();

        bool CheckFixMapPose(KeyFrame *pKF);

        int KeyFramesInQue();
        bool IsKFQueFull();

        bool LargeDistanceLastKeyframe();
        double DistToLastKeyFrame();
        double DistToLastKeyFrame(KeyFrame *pKF);
        double DistFirstToLastKeyFrame();

        void MapPointCulling();
        void KeyFrameCulling();

        Map *GetMap();

        bool InitializeFixedMap();
        bool IsInitializing() { return (mFixedMapState == INITIALIZING); };
        bool IsInitialized() { return (mFixedMapState >= INITIALIZED); };
        void SetInitialized() { mFixedMapState = INITIALIZED; };

        bool IsFixedTransformation() { return (mFixedMapState == FIX_TRANSFORM); };

        void FixateTransformation(bool bFixate);

        cv::Mat GetTransformation();
        void UpdateTransformation(cv::Mat Tlf, Eigen::MatrixXd Cov);
        void SetTransformation(cv::Mat Tlf);
        void SetCovariance(Eigen::MatrixXd Cov);
        Eigen::MatrixXd GetInformation();
        Eigen::MatrixXd GetCovariance();

        void SetLastKeyFrame(KeyFrame *pKF) { mpLastKeyFrame = pKF; };
        KeyFrame *GetLastKeyFrame() { return mpLastKeyFrame; };

        void SetFirstKeyFrame(KeyFrame *pKF) { mpFirstKeyFrame = pKF; };
        KeyFrame *GetFirstKeyFrame() { return mpFirstKeyFrame; };

        list<KeyFrame *> GetKeyFrameWindow();

        void Reset();

        bool bAddedKeyFrames = false;
        int mnKeyFramesInQue = 0;

        CameraCalibration *mpCameraCalibrater;

    protected:
        bool mbResetRequested;
        std::recursive_mutex mMutexReset;

        Map *mpMap;
        Map *mpLocalMap;

        std::list<KeyFrame *> mlNewKeyFrames;

        KeyFrame *mpCurrentKeyFrame;
        KeyFrame *mpLastKeyFrame;
        KeyFrame *mpFirstKeyFrame;

        int mnQueSize;
        int mnWindowSize;
        list<KeyFrame *> mlCurrentKeyFrameWindow;

        enum eFixedMapState
        {
            NOT_INITIALIZED = 0,
            INITIALIZING = 1,
            INITIALIZED = 3,
            FIX_TRANSFORM = 4
        };

        eFixedMapState mFixedMapState;

        // Transfromation from fixed map point in local map frame
        cv::Mat mTlf = cv::Mat::eye(4, 4, CV_32F);

        Eigen::Matrix<double, 6, 6> mTlfCovariance;
        const float mSigmaPosition = 1;
        const float mSigmaHeading = 1.0 / 20.0;
    };

} // namespace LocSLAM

#endif // FIXEDMAPLOCALIZATION_h
