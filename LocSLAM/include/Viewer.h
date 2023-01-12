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

#ifndef VIEWER_H
#define VIEWER_H

#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Tracking.h"
#include "KeyFrame.h"
#include "KeyFrame.h"
#include "System.h"

#include <mutex>

namespace LocSLAM
{

    class Tracking;
    class FrameDrawer;
    class MapDrawer;
    class System;

    class Viewer
    {
    public:
        Viewer(System *pSystem, vector<FrameDrawer *> vpFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, const string &strSettingPath);
        ~Viewer();

        // Main thread function. Draw points, keyframes, the current camera pose and the last processed
        // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
        void Run();

        void RequestFinish();

        void RequestStop();

        bool isFinished();

        bool isStopped();

        void Release();

    private:
        bool Stop();

        System *mpSystem;
        vector<FrameDrawer *> mvpFrameDrawer;
        MapDrawer *mpMapDrawer;
        Tracking *mpTracker;
        KeyFrame *mpCurrentKeyFrame;

        bool mbRecordVideo;
        string msCamVideoFileName;
        string msMapVideoFileName;
        cv::VideoWriter mVideoRec;

        // Number of cameras in a rig
        int mnCameras;
        vector<int> mvSupportedCameras;

        // 1/fps in ms
        double mT;
        float mImageWidth, mImageHeight;

        float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

        float mImageScale;

        bool CheckFinish();
        void SetFinish();
        bool mbFinishRequested;
        bool mbFinished;
        std::recursive_mutex mMutexFinish;

        bool mbStopped;
        bool mbStopRequested;
        std::recursive_mutex mMutexStop;
    };

} // namespace LocSLAM

#endif // VIEWER_H
