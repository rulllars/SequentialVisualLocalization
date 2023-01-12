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

#ifndef TRACKING_H
#define TRACKING_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <Eigen/Dense>

#include "Viewer.h"
#include "FrameDrawer.h"
#include "Map.h"
#include "LocalMapping.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
#include "ORBextractor.h"
#include "Initializer.h"
#include "MapDrawer.h"
#include "System.h"
#include "SLAMDataTypes.h"

#include <mutex>

namespace LocSLAM
{

    class Viewer;
    class FrameDrawer;
    class Map;
    class LocalMapping;
    class System;
    class Initializer;

    class Tracking
    {

    public:
        Tracking(System *pSys, ORB_SLAM2::ORBVocabulary *pVoc, vector<FrameDrawer *> vpFrameDrawer, MapDrawer *pMapDrawer, Map *pMap,
                 Map *pFixedMap, KeyFrameDatabase *pKFDB, const string &strSettingPath, const int sensor, const bool bUseOdometry = false);

        // Preprocess the input and call Track(). Extract features and performs stereo matching.
        cv::Mat GrabImageMonocularOdo(const cv::Mat &im, const OdometryData &odometry,
                                      const FixedMapMatches &frameFixeMapMatches,
                                      const double &timestamp, const int &cameraID = 0, string strImgFileName = string());

        void SetLocalMapper(LocalMapping *pLocalMapper);
        void SetViewer(Viewer *pViewer);

        // Load new settings
        // The focal length should be similar or scale prediction will fail when projecting points
        void ChangeCalibration(const string &strSettingPath);

        cv::Mat GetExtrinsics(int cameraID);
        void UpdateExtrinsics(int cameraID, cv::Mat calib);

        double GetDistanceLastFixedKF();
        double GetDistanceLastFixedKF(int cameraID);

        void StoreCurrentPose();

    public:
        // Camera definitions
        struct sCameraCalibration
        {
            cv::Mat Tcb;         // Camera extrinsics (Active transformation from camera to body)
            cv::Mat K;           // Calibration matix
            cv::Mat DistCoef;    // OpenCV distortion parameters
            float fBaseLine = 0; // Base line for stereo system (called mbf in org ORB_SLAM)
            int fps;             // Frames per second
        };

        // Single camera tracking states
        enum eSingleTrackingState
        {
            NO_IMAGES_YET = 0,
            NOT_INITIALIZED = 1,
            OK = 2,
            LOST = 3
        };
        vector<eSingleTrackingState> mState;
        vector<eSingleTrackingState> mLastProcessedState;

        // tracking system states
        enum eSystemTrackingState
        {
            SYSTEM_NOT_READY = -1,
            NON_INITIALIZED = 0,
            INITIALIZING = 1,
            INITIALIZED = 2,
        };

        eSystemTrackingState mSystemState;

        // Input sensor
        int mSensor;
        int mnCameras;
        bool mbFixedMapInitialized;

        // Current Frame
        int mCurrentCameraID;
        vector<Frame> mvCurrentFrame;
        cv::Mat mImGray;

        // Initalization (only for monocular)
        vector<Initializer *> mvpInitializer;

        // Lists used to recover the full camera trajectory at the end of the execution.
        // Basically we store the reference keyframe for each frame and its relative transformation
        list<cv::Mat> mlRelativeFramePoses;
        list<int> mlCameraID;
        list<KeyFrame *> mlpReferences;
        list<string> mlstrFileNames;
        list<double> mlFrameTimes;
        list<bool> mlbLost;

        list<cv::Mat> mlTrackingPose;

        void Reset();

    protected:
        // Main tracking function. It is independent of the input sensor.
        void Track();

        // Map initialization for monocular
        void MonocularInitialization();
        void CreateInitialMapMonocular();

        void CheckReplacedInLastFrame();
        bool TrackReferenceKeyFrame();
        void UpdateLastFrame();
        bool TrackWithMotionModel();

        bool Relocalization();

        void UpdateLocalMap();
        void UpdateLocalPoints();
        void UpdateLocalKeyFrames();

        bool TrackLocalMap();
        void SearchLocalPoints();

        bool NeedNewKeyFrame();
        void CreateNewKeyFrame();
        void CreateNewFixedKeyFrame(Frame &newKFFrame);
        void CreateNewFixedKeyFrame(Initializer *pInitializer);

        // Check if any camera is tracking
        bool AnyCameraTracking();
        bool AnyCameraReseted();
        bool AnyCameraInitializing();

        //Other Thread Pointers
        LocalMapping *mpLocalMapper;

        //ORB
        ORB_SLAM2::ORBextractor *mpORBextractorLeft, *mpORBextractorRight;
        ORB_SLAM2::ORBextractor *mpIniORBextractor;

        //BoW
        ORB_SLAM2::ORBVocabulary *mpORBVocabulary;
        KeyFrameDatabase *mpKeyFrameDB;

        //Local Map
        vector<KeyFrame *> mvpReferenceKF;
        std::vector<KeyFrame *> mvpLocalKeyFrames;
        std::vector<MapPoint *> mvpLocalMapPoints;

        // System
        System *mpSystem;

        //Drawers
        Viewer *mpViewer;
        vector<FrameDrawer *> mvpFrameDrawer;
        MapDrawer *mpMapDrawer;

        //Map
        Map *mpMap;
        Map *mpFixedMap;

        // Intrinsic and extrinsic camera parameters
        vector<sCameraCalibration> msCameraCalibration;

        //New KeyFrame rules (according to fps)
        int mMinFrames;
        int mMaxFrames;

        // Fixed map localization
        int mnLastFixedMapFrameId = 0;
        vector<OdometryData> mOdoLastFixedKF;

        // Threshold close/far points
        // Points seen as close by the stereo/RGBD sensor are considered reliable
        // and inserted from just one frame. Far points requiere a match in two keyframes.
        float mThDepth;

        // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
        float mDepthMapFactor;

        //Current matches in frame
        int mnMatchesInliers;

        // Good tracking
        vector<bool> mvbGoodMapTracking;

        //Last Frame, KeyFrame and Relocalisation Info
        KeyFrame *mpLastKeyFrame;
        vector<Frame> mvPreviousFrames;
        int mLastCameraID;
        unsigned int mnLastKFFrameId;
        unsigned int mnLastRelocFrameId;

        // Indicate that we had a camera rest for a specific camera
        vector<bool> mbWasCameraReseted;

        //Motion Model
        cv::Mat mVelocity;

        // Aggregate odometry since last keyFrame. Should be cleared when new KF is created.
        OdometryData mOdometryLastKF;
        bool mbUseOdometry;
        bool mbTrackWithOdometry;

        // If set, we describe the motion of the camera in terms of velocites (x,y,z) and (yaw, picth, roll)
        // instead of as a SE(3) matrix. This to be able to compensate for uneven frame rates
        bool mbTrueVelocity;
        Eigen::Matrix<double, 3, 1> mvTranslationalVelocity;
        Eigen::Matrix<double, 3, 3> mRotationalVelocity;

        //Color order (true RGB, false BGR, ignored if grayscale)
        bool mbRGB;

        list<MapPoint *> mlpTemporalPoints;
    };

} // namespace LocSLAM

#endif // TRACKING_H
