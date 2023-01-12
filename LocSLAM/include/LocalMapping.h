/**
* This file is part of LocSLAM.
*
* Copyright (C) 2020-2021 Lars Hammarstrand <lars.hammarstrand at chalmers dot se> (2021)
* For more information see <https://github.com/rulllars/SequentialLocalization>
*
* This file is a modified version of the file with the same name in ORB_SLAM2:
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
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
* along with LocSLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "KeyFrame.h"
#include "Map.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"
#include "CameraCalibration.h"
#include "FixedMapLocalization.h"

#include <mutex>

namespace LocSLAM
{

    class Tracking;
    class Map;
    class FixedMapLocalization;
    class CameraCalibration;

    class LocalMapping
    {
    public:
        LocalMapping(Map *pMap, Map *pFixedMap, CameraCalibration *pCameraCalibrater, const float bMonocular, const bool bUseFixedMap);

        void SetTracker(Tracking *pTracker);

        KeyFrame *GetLastKeyFrame() { return mpCurrentKeyFrame; }

        // Main function
        void Run();

        void UpdateLocalMap();

        void InsertKeyFrame(KeyFrame *pKF);

        // Thread Synch
        void RequestStop();
        void RequestReset();
        bool Stop();
        void Release();
        bool isStopped();
        bool stopRequested();
        bool AcceptKeyFrames();
        void SetAcceptKeyFrames(bool flag);
        bool SetNotStop(bool flag);

        void InterruptBA();

        void RequestFinish();
        bool isFinished();

        int KeyframesInQueue()
        {
            unique_lock<std::recursive_mutex> lock(mMutexNewKFs);
            return mlNewKeyFrames.size();
        }

        // Fixed map
        FixedMapLocalization *mpFixedMapLocalizer;

    protected:
        bool CheckNewKeyFrames();
        void ProcessNewKeyFrame();
        void CreateNewMapPoints();

        void MapPointCulling();
        void SearchInNeighbors();

        void KeyFrameCulling();

        void InitializeGlobalMap();

        cv::Mat ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2);

        cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

        bool mbMonocular;
        bool mbUseFixedMap;

        void ResetIfRequested();
        bool mbResetRequested;
        std::recursive_mutex mMutexReset;

        bool CheckFinish();
        void SetFinish();
        bool mbFinishRequested;
        bool mbFinished;
        std::recursive_mutex mMutexFinish;

        Map *mpLocalMap;
        Map *mpFixedMap;

        // Fixed map
        int mnFixMapLocalizations = 0;

        Tracking *mpTracker;

        std::list<KeyFrame *> mlNewKeyFrames;

        KeyFrame *mpCurrentKeyFrame;

        std::list<MapPoint *> mlpRecentAddedMapPoints;

        // Camera extrinsic calibration

        int mnFramesSinceCalibration;

        std::recursive_mutex mMutexNewKFs;

        bool mbAbortBA;

        bool mbStopped;
        bool mbStopRequested;
        bool mbNotStop;
        std::recursive_mutex mMutexStop;

        bool mbAcceptKeyFrames;
        std::recursive_mutex mMutexAccept;
    };

} // namespace LocSLAM

#endif // LOCALMAPPING_H
