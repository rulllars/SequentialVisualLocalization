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

#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>

#include <mutex>

namespace LocSLAM
{

    class Map;
    class KeyFrame;

    class MapDrawer
    {
    public:
        MapDrawer(Map *pMap, Map *pFixedMap, const string &strSettingPath);

        Map *mpMap;
        Map *mpFixedMap;

        void DrawMapPoints();
        void DrawLocalMapPoints();
        void DrawFixedMapPoints();
        void DrawLocalFixedMapPoints();
        void DrawKeyFrames(const bool bDrawKF, const bool bDrawPriors, const bool bDrawGraph, const bool bDrawPose, const bool bDrawOdo);
        void DrawCurrentPose(const cv::Mat &Tbw);
        void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
        void DrawCurrentPredictedCamera(pangolin::OpenGlMatrix &Twc);
        void SetCurrentCameraPose(const cv::Mat &Tcw);
        void SetPredictedCameraPose(const cv::Mat &Tcw);
        void SetViewCameraPose(const cv::Mat &Tbw);
        void SetReferenceKeyFrame(KeyFrame *pKF);
        void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);
        void GetCurrentOpenGLPredictedCameraMatrix(pangolin::OpenGlMatrix &M);
        void GetCurrentOpenGLBodyMatrix(pangolin::OpenGlMatrix &M);
        void GetCurrentBodyMatrix(cv::Mat &Tbw);

        const pangolin::OpenGlMatrix CVMat2OpenGL(cv::Mat Twc);

        void UpdateKeyFrames();

        void SetCurrentKeyFrame(KeyFrame *pCurrentKF);
        void SetFixedMap(Map *pFixedMap) { mpFixedMap = pFixedMap; };

        bool mbUpdateCurrentCamera;

    private:
        float mKeyFrameSize;
        float mKeyFrameLineWidth;
        float mGraphLineWidth;
        float mPointSize;
        float mCameraSize;
        float mCameraLineWidth;

        KeyFrame *mpCurrentKeyFrame;

        vector<KeyFrame *> mvpKFs;

        vector<KeyFrame *> mvpFixedMapKFs;

        cv::Mat mCameraPose;
        cv::Mat mPredictedCameraPose;
        cv::Mat mBodyPose;

        // White background
        /*
    float vcPose[4] = { 0.3f, 0.3f, 0.8f, 1.0f };
    float vcOdo[4] = { 0.5f, 0.0f, 0.5f, .6f };
    float vcOdoConnection[4] = { .7f, 0.3f, 0.3f, .6f };
    
    float vcLocalMapPoints[4] = { 0.3f, 0.3f, 0.3f, 1.0f};
    float vcStaticMapPoints[4] = {0.3f, 0.3f, 0.3f, 0.7f };

    float vcPredictedCamera[4] = { 1.0f, 0.3f, 0.3f, 0.7f };
    float vcUpdatedCamera[4] = { 0.3f, 1.0f, 0.3f, 0.7f };
    
    float vcPosePrior[4] = { 1.0f, 0.3f, 1.0f, 0.8f };
    float vcFixedMapPoints[4] = { 1.0f, 0.3f, 1.0f, 0.8f };
    
    float vcGraph[4] = { 0.3f, 0.7f, .3f, 0.6f };
    */

        // Black background
        float vcPose[4] = {0.3f, 0.3f, 1.f, 1.0f};
        float vcOdo[4] = {0.5f, 0.0f, 0.5f, .6f};
        float vcOdoConnection[4] = {1.f, 0.3f, 0.3f, .6f};

        float vcLocalMapPoints[4] = {0.6f, 0.6f, 0.6f, 1.0f};
        float vcStaticMapPoints[4] = {0.6f, 0.6f, 0.6f, 0.7f};

        float vcPredictedCamera[4] = {1.0f, 0.3f, 0.3f, 0.7f};
        float vcUpdatedCamera[4] = {0.3f, 1.0f, 0.3f, 0.7f};

        float vcPosePrior[4] = {1.0f, 0.3f, 1.0f, 0.8f};
        float vcPosePriorOutlier[4] = {1.0f, 0.2f, 0.2f, 0.8f};
        float vcFixedMapPoints[4] = {1.0f, 0.3f, 1.0f, 0.4f};

        float vcGraph[4] = {0.3f, 1.0f, .3f, 0.6f};

        std::recursive_mutex mMutexCamera;
    };

} // namespace LocSLAM

#endif // MAPDRAWER_H
