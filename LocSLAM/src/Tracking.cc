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

#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/eigen.hpp>

#include <algorithm>

#include <unsupported/Eigen/MatrixFunctions>

#include "ORBmatcher.h"
#include "FrameDrawer.h"
#include "Converter.h"
#include "Map.h"
#include "Initializer.h"

#include "Optimizer.h"
#include "PnPsolver.h"

#include <iostream>

#include <mutex>

using namespace std;

namespace LocSLAM
{

    Tracking::Tracking(System *pSys, ORB_SLAM2::ORBVocabulary *pVoc, vector<FrameDrawer *> vpFrameDrawer,
                       MapDrawer *pMapDrawer, Map *pMap, Map *pFixedMap, KeyFrameDatabase *pKFDB, const string &strSettingPath,
                       const int sensor, const bool bUseOdometry)
        : mSensor(sensor), mpORBVocabulary(pVoc),
          //mbOnlyTracking(false), mbVO(false),
          mpKeyFrameDB(pKFDB), mpSystem(pSys), mpViewer(NULL),
          mvpFrameDrawer(vpFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap), mpFixedMap(pFixedMap), mnLastRelocFrameId(0), mbUseOdometry(bUseOdometry),
          mpLastKeyFrame(static_cast<KeyFrame *>(NULL))
    {
        // Load camera parameters from settings file
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        // Load number of cameras in rig
        mnCameras = fSettings["nCameras"];

        mSystemState = SYSTEM_NOT_READY;

        // Allocate vectors
        mState.resize(mnCameras);
        mLastProcessedState.resize(mnCameras);
        msCameraCalibration.resize(mnCameras);

        mOdoLastFixedKF.resize(mnCameras);

        mvbGoodMapTracking.resize(mnCameras);

        mvCurrentFrame.resize(mnCameras);
        mvPreviousFrames.resize(mnCameras);

        mvpReferenceKF.resize(mnCameras);
        mpLastKeyFrame = NULL;

        mLastCameraID = 0;
        mCurrentCameraID = 0;

        mbWasCameraReseted.resize(mnCameras);
        for (int i = 0; i < mnCameras; i++)
            mbWasCameraReseted[i] = false;

        // Per camera initializers
        mvpInitializer.resize(mnCameras);
        for (int i = 0; i < mnCameras; i++)
            mvpInitializer[i] = static_cast<Initializer *>(NULL);

        // Load intrinsic and extrinsic for each camera
        std::string s("Camera");
        for (int i = 0; i < mnCameras; i++)
        {
            std::string sCurrentCamera = "Camera" + std::to_string(i);

            float fx = fSettings[sCurrentCamera + ".fx"];
            float fy = fSettings[sCurrentCamera + ".fy"];
            float cx = fSettings[sCurrentCamera + ".cx"];
            float cy = fSettings[sCurrentCamera + ".cy"];

            cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
            K.at<float>(0, 0) = fx;
            K.at<float>(1, 1) = fy;
            K.at<float>(0, 2) = cx;
            K.at<float>(1, 2) = cy;
            K.copyTo(msCameraCalibration[i].K);

            cv::Mat DistCoef(4, 1, CV_32F);
            DistCoef.at<float>(0) = fSettings[sCurrentCamera + ".k1"];
            DistCoef.at<float>(1) = fSettings[sCurrentCamera + ".k2"];
            DistCoef.at<float>(2) = fSettings[sCurrentCamera + ".p1"];
            DistCoef.at<float>(3) = fSettings[sCurrentCamera + ".p2"];
            const float k3 = fSettings[sCurrentCamera + ".k3"];
            if (k3 != 0)
            {
                DistCoef.resize(5);
                DistCoef.at<float>(4) = k3;
            }
            DistCoef.copyTo(msCameraCalibration[i].DistCoef);

            // Camera extrinsic
            cv::Mat Tcb = cv::Mat::eye(4, 4, CV_32F);
            fSettings[sCurrentCamera + ".Tcb"] >> Tcb;

            Tcb.copyTo(msCameraCalibration[i].Tcb);

            msCameraCalibration[i].fBaseLine = fSettings[sCurrentCamera + ".bf"];

            float fps = fSettings[sCurrentCamera + ".fps"];
            if (fps == 0)
                fps = 30;

            msCameraCalibration[i].fps = fps;

            std::cout << endl
                      << "Camera Parameters: Camera " << i << endl;
            std::cout << "- fx: " << fx << endl;
            std::cout << "- fy: " << fy << endl;
            std::cout << "- cx: " << cx << endl;
            std::cout << "- cy: " << cy << endl;
            std::cout << "- k1: " << DistCoef.at<float>(0) << endl;
            std::cout << "- k2: " << DistCoef.at<float>(1) << endl;
            if (DistCoef.rows == 5)
                std::cout << "- k3: " << DistCoef.at<float>(4) << endl;
            std::cout << "- p1: " << DistCoef.at<float>(2) << endl;
            std::cout << "- p2: " << DistCoef.at<float>(3) << endl;
            std::cout << "- fps: " << fps << endl;

            cout << endl
                 << "Extrinsic: Tcb" << endl;
            cout << Tcb << endl;

            // Set tracking state
            mState[i] = NO_IMAGES_YET;
        }

        int nTrackWithOdometry = fSettings["Tracking.TrackWithOdometry"];
        mbTrackWithOdometry = (nTrackWithOdometry == 1);

        if (nTrackWithOdometry)
            std::cout << endl
                      << "Tracking: Tracking using odometry constraints." << endl;
        else
            std::cout << endl
                      << "Tracking: Tracking without using odometry constraints." << endl;

        // Max/Min Frames to insert keyframes and to check relocalization
        mMinFrames = 1;
        mMaxFrames = msCameraCalibration[0].fps;

        int nRGB = fSettings["Camera0.RGB"];
        mbRGB = nRGB;

        if (mbRGB)
            std::cout << "- color order: RGB (ignored if grayscale)" << endl;
        else
            std::cout << "- color order: BGR (ignored if grayscale)" << endl;

        // Load ORB parameters

        int nFeatures = fSettings["ORBextractor.nFeatures"];
        float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
        int nLevels = fSettings["ORBextractor.nLevels"];
        int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
        int fMinThFAST = fSettings["ORBextractor.minThFAST"];

        mpORBextractorLeft = new ORB_SLAM2::ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

        if (sensor == System::STEREO)
            mpORBextractorRight = new ORB_SLAM2::ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

        if (sensor == System::MONOCULAR)
            mpIniORBextractor = new ORB_SLAM2::ORBextractor(2 * nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

        std::cout << endl
                  << "ORB Extractor Parameters: " << endl;
        std::cout << "- Number of Features: " << nFeatures << endl;
        std::cout << "- Scale Levels: " << nLevels << endl;
        std::cout << "- Scale Factor: " << fScaleFactor << endl;
        std::cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
        std::cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

        if (sensor == System::STEREO || sensor == System::RGBD)
        {
            mThDepth = msCameraCalibration[0].fBaseLine * (float)fSettings["ThDepth"] / msCameraCalibration[0].K.at<float>(0, 0);
            std::cout << endl
                      << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
        }

        if (sensor == System::RGBD)
        {
            mDepthMapFactor = fSettings["DepthMapFactor"];
            if (fabs(mDepthMapFactor) < 1e-5)
                mDepthMapFactor = 1;
            else
                mDepthMapFactor = 1.0f / mDepthMapFactor;
        }

        mbFixedMapInitialized = false;
    }

    void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
    {
        mpLocalMapper = pLocalMapper;
    }

    void Tracking::SetViewer(Viewer *pViewer)
    {
        mpViewer = pViewer;
    }

    cv::Mat Tracking::GrabImageMonocularOdo(const cv::Mat &im, const OdometryData &odometry,
                                            const FixedMapMatches &frameFixedMapMatches,
                                            const double &timestamp, const int &cameraID, string strImgFileName)
    {
        mImGray = im;

        if (mImGray.channels() == 3)
        {
            if (mbRGB)
                cvtColor(mImGray, mImGray, CV_RGB2GRAY);
            else
                cvtColor(mImGray, mImGray, CV_BGR2GRAY);
        }
        else if (mImGray.channels() == 4)
        {
            if (mbRGB)
                cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
            else
                cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
        }

        // Get camera paramters for current frame
        cv::Mat Tcb = msCameraCalibration[cameraID].Tcb;
        cv::Mat K = msCameraCalibration[cameraID].K;
        cv::Mat distCoef = msCameraCalibration[cameraID].DistCoef;
        float baseLine = msCameraCalibration[cameraID].fBaseLine;

        mCurrentCameraID = cameraID;
        if (mState[cameraID] == NOT_INITIALIZED || mState[cameraID] == NO_IMAGES_YET)
            mvCurrentFrame[cameraID] = Frame(mImGray, odometry, frameFixedMapMatches, timestamp, mpIniORBextractor, mpORBVocabulary, cameraID, strImgFileName, Tcb, K, distCoef, baseLine, mThDepth);
        else
            mvCurrentFrame[cameraID] = Frame(mImGray, odometry, frameFixedMapMatches, timestamp, mpORBextractorLeft, mpORBVocabulary, cameraID, strImgFileName, Tcb, K, distCoef, baseLine, mThDepth);

        // Update odometry histroy since last KF
        mOdometryLastKF.Integrate(odometry);

        // Update Odometry since last fix map KF
        if (mpSystem->mbUseGlobalMap)
            for (int i = 0; i < mnCameras; i++)
                mOdoLastFixedKF[i].Integrate(odometry);

        Track();

        return mvCurrentFrame[cameraID].mTbw.clone();
    }

    void Tracking::Track()
    {
        // Setup frame stucture for current camera
        eSingleTrackingState &currentState = mState[mCurrentCameraID];
        Frame &currentFrame = mvCurrentFrame[mCurrentCameraID];
        Frame &previousCameraFrame = mvPreviousFrames[mCurrentCameraID];
        Frame &previousBodyFrame = mvPreviousFrames[mLastCameraID];

        if (currentState == NO_IMAGES_YET)
        {
            currentState = NOT_INITIALIZED;
        }

        if (mSystemState == SYSTEM_NOT_READY)
        {
            mSystemState = NON_INITIALIZED;
        }

        mLastProcessedState[mCurrentCameraID] = currentState;

        // Update initializer odometry
        for (int iCam = 0; iCam < mnCameras; iCam++)
        {
            if (mvpInitializer[iCam])
            {
                mvpInitializer[iCam]->mOdometryDelta.Integrate(currentFrame.mOdometry);
            }
        }

        // Update frame motion
        if (mSystemState > NON_INITIALIZED) // We have an initial frame?
        {

            if (mSystemState == INITIALIZED)
                UpdateLastFrame();
            else
                cout << "Tracking:\tNot initialized" << endl;

            if (mbUseOdometry)
            {
                if (mSystemState == INITIALIZING || (mSystemState == INITIALIZED && AnyCameraTracking()))
                {
                    currentFrame.PredictAndSetPose(previousBodyFrame.mTbw);
                }
                else if (mpLastKeyFrame)
                {
                    currentFrame.SetPose(mOdometryLastKF.Predict(mpLastKeyFrame->GetPose()));
                    currentFrame.mPoseCovariance = previousBodyFrame.mPoseCovariance + currentFrame.mOdometry.GetCovariance();
                }
            }
            else if (!mVelocity.empty())
            {
                currentFrame.SetPose(mVelocity * previousBodyFrame.mTbw);
            }

            mpMapDrawer->SetPredictedCameraPose(currentFrame.mTcb * currentFrame.mTbw);
            mpMapDrawer->mbUpdateCurrentCamera = true;
        }

        {

            // Get Map Mutex -> Map cannot be changed
            unique_lock<recursive_mutex> lock(mpMap->mMutexMapUpdate);

            if (currentState == NOT_INITIALIZED)
            {
                if (mSensor == System::MONOCULAR)
                    MonocularInitialization();
                else
                    cout << "Track:\tOnly MONOCULAR cameras supported" << endl;

                mvpFrameDrawer[mCurrentCameraID]->Update(this);
            }
            //else

            if (mSystemState == INITIALIZED)
            {

                // System is initialized. Track Frame.
                bool bOK;

                if (currentState == OK)
                {

                    // Local Mapping might have changed some MapPoints tracked in last frame
                    CheckReplacedInLastFrame();

                    if ((mVelocity.empty() && currentFrame.mOdometry.mPredictionTime == 0) || currentFrame.mnId < mnLastRelocFrameId + 2)
                    {
                        bOK = TrackReferenceKeyFrame();
                    }
                    else
                    {
                        bOK = TrackWithMotionModel();
                        if (!bOK)
                            bOK = TrackReferenceKeyFrame();
                    }
                }
                else
                {
                    bOK = Relocalization();
                }

                currentFrame.mpReferenceKF = mvpReferenceKF[mCurrentCameraID];

                if (bOK)
                {
                    bOK = TrackLocalMap();
                }

                if (bOK)
                    currentState = OK;
                else
                    currentState = LOST;

                mpMapDrawer->SetCurrentCameraPose(currentFrame.mTcb * currentFrame.mTbw);
                mpMapDrawer->SetViewCameraPose(currentFrame.mTbw);
                mpMapDrawer->mbUpdateCurrentCamera = true;

                // Update drawer
                mvpFrameDrawer[mCurrentCameraID]->Update(this);

                // If tracking were good, check if we insert a keyframe
                if (bOK)
                {
                    // Update motion model if prediction time is sufficiently large
                    double T = currentFrame.mTimeStamp - previousBodyFrame.mTimeStamp;

                    if (!previousBodyFrame.mTbw.empty())
                    {
                        // Check if we have odometry data
                        if (mbUseOdometry)
                        {
                            // Added for possible backward compatibility with ORB SLAM
                            mVelocity = currentFrame.mOdometry.GetTransformation();
                        }
                        else // No odometry data -> use constant velocity model
                        {
                            cv::Mat LastTwb = cv::Mat::eye(4, 4, CV_32F);
                            previousBodyFrame.GetPoseRotationInverse().copyTo(LastTwb.rowRange(0, 3).colRange(0, 3));
                            previousBodyFrame.GetBodyPosition().copyTo(LastTwb.rowRange(0, 3).col(3));
                            mVelocity = currentFrame.mTbw * LastTwb;
                        }
                    }
                    else
                        mVelocity = cv::Mat();

                    // Clean VO matches
                    for (int i = 0; i < currentFrame.N; i++)
                    {
                        MapPoint *pMP = currentFrame.mvpMapPoints[i];
                        if (pMP)
                            if (pMP->Observations() < 1)
                            {
                                currentFrame.mvbOutlier[i] = false;
                                currentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                            }
                    }

                    // Delete temporal MapPoints
                    for (list<MapPoint *>::iterator lit = mlpTemporalPoints.begin(), lend = mlpTemporalPoints.end(); lit != lend; lit++)
                    {
                        MapPoint *pMP = *lit;
                        delete pMP;
                    }
                    mlpTemporalPoints.clear();

                    // Release map to allow for map update from localBA
                    unique_lock<recursive_mutex> unlock(mpMap->mMutexMapUpdate);

                    // Check if we need to insert a new keyframe
                    if (NeedNewKeyFrame())
                    {
                        CreateNewKeyFrame();
                    }

                    // Get Map Mutex -> Map cannot be changed
                    unique_lock<recursive_mutex> lock(mpMap->mMutexMapUpdate);

                    // We allow points with high innovation (considered outliers by the Huber Function)
                    // pass to the new keyframe, so that bundle adjustment will finally decide
                    // if they are outliers or not. We don't want next frame to estimate its position
                    // with those points so we discard them in the frame.
                    for (int i = 0; i < currentFrame.N; i++)
                    {
                        if (currentFrame.mvpMapPoints[i] && currentFrame.mvbOutlier[i])
                            currentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                    }

                    // Enable local mapper again
                    mpLocalMapper->SetNotStop(false);
                }

                // Reset if the camera get lost soon after initialization
                if (currentState == LOST)
                {
                    // Reset current pose using odometry
                    if (currentFrame.mbUseOdometry)
                    {
                        if (mSystemState >= INITIALIZING && AnyCameraTracking())
                        {
                            currentFrame.PredictAndSetPose(previousBodyFrame.mTbw);
                        }
                        else if (mpLastKeyFrame)
                        {
                            currentFrame.SetPose(mOdometryLastKF.Predict(mpLastKeyFrame->GetPose()));
                        }

                        mpMapDrawer->SetCurrentCameraPose(currentFrame.mTcb * currentFrame.mTbw);
                        mpMapDrawer->SetViewCameraPose(currentFrame.mTbw);

                        mpMapDrawer->mbUpdateCurrentCamera = true;
                    }

                    Reset();
                }

                if (!currentFrame.mpReferenceKF)
                    currentFrame.mpReferenceKF = mvpReferenceKF[mCurrentCameraID];
            }

            // Store frame pose information to retrieve the complete camera trajectory afterwards.

            if (mpLastKeyFrame != NULL) // Check that we have at least one key frame
            {
                if (!currentFrame.mTbw.empty())
                {
                    currentFrame.mpLastKeyFrame = mpLastKeyFrame;

                    cv::Mat Tbr = currentFrame.mTbw * currentFrame.mpLastKeyFrame->GetPoseInverse(); // currentFrame.mpReferenceKF->GetPoseInverse();
                    currentFrame.mTbr = Tbr;
                }
            }

            // Update last frame vector
            currentFrame.mpLastKeyFrame = mpLastKeyFrame;
            mvPreviousFrames[mCurrentCameraID] = Frame(currentFrame);
            mLastCameraID = mCurrentCameraID;
        }
    }
    void Tracking::StoreCurrentPose()
    {
        Frame &currentFrame = mvCurrentFrame[mCurrentCameraID];
        eSingleTrackingState currentState = mState[mCurrentCameraID];

        // Store frame pose information to retrieve the complete camera trajectory afterwards.
        if (mpLastKeyFrame != NULL) // Check that we have at least one key frame
        {
            if (!currentFrame.mTbw.empty())
            {
                cv::Mat Tbr = currentFrame.mTbw * currentFrame.mpLastKeyFrame->GetPoseInverse(); // currentFrame.mpReferenceKF->GetPoseInverse();
                cv::Mat Tbw = Tbr * currentFrame.mpLastKeyFrame->GetPose();

                mlRelativeFramePoses.push_back(Tbr);
                mlCameraID.push_back(mCurrentCameraID);
                mlpReferences.push_back(mpLastKeyFrame); //push_back(mvpReferenceKF[mCurrentCameraID]);
                mlstrFileNames.push_back(currentFrame.mstrImgFileName);
                mlFrameTimes.push_back(currentFrame.mTimeStamp);
                mlbLost.push_back((currentState) == LOST);

                mlTrackingPose.push_back(Tbw);
            }
            else
            {
                // This can happen if tracking is lost
                mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
                mlCameraID.push_back(mCurrentCameraID);
                mlpReferences.push_back(mlpReferences.back());
                mlstrFileNames.push_back(mlstrFileNames.back());
                mlFrameTimes.push_back(mlFrameTimes.back());
                mlbLost.push_back((currentState) == LOST);

                mlTrackingPose.push_back(cv::Mat::eye(4, 4, CV_32F));
            }
        }
    }

    void Tracking::MonocularInitialization()
    {

        // Setup frame stucture for current camera
        eSingleTrackingState *pState = &mState[mCurrentCameraID];
        Frame &currentFrame = mvCurrentFrame[mCurrentCameraID];
        Frame &previousCameraFrame = mvPreviousFrames[mCurrentCameraID];
        Frame &previousBodyFrame = mvPreviousFrames[mLastCameraID];

        if (mvpInitializer[mCurrentCameraID] == NULL)
        {
            bool init = false;
            if (mbUseOdometry && currentFrame.mvKeys.size() > 50)
                init = true;
            else if (currentFrame.mvKeys.size() > 100)
                init = true;

            if (init)
            {
                // Set Reference Frame
                if (mSystemState == NON_INITIALIZED)
                {
                    // No camera is initalized...

                    // Set first camera at the origin
                    cv::Mat Tcw = cv::Mat::eye(4, 4, CV_32F);
                    cv::Mat Tbc = currentFrame.mTcb.inv();
                    currentFrame.SetPose(Tbc * Tcw);

                    // Update Frames

                    mSystemState = INITIALIZING;
                }
                else // We have initialized and lost track or started initialization for at least one other camera
                {

                    if (mvpInitializer[mCurrentCameraID])
                        delete mvpInitializer[mCurrentCameraID];

                    mvpInitializer[mCurrentCameraID] = new Initializer(currentFrame, mpLastKeyFrame, mOdometryLastKF, 3.0, 200);
                    mState[mCurrentCameraID] = NOT_INITIALIZED;

                    // Set current frame as initial frame for current camera ID
                    mvPreviousFrames[mCurrentCameraID] = Frame(currentFrame);
                    mLastCameraID = mCurrentCameraID;

                    return;
                }
            }
            else // not enough local points to initialize
            {
                // If currentFrame frame has fix map matches add it as a key frame anyway
                if (mSystemState == INITIALIZED && currentFrame.mFixedMapMatches.nMatches > 5 && GetDistanceLastFixedKF(mCurrentCameraID) > 5)
                    CreateNewFixedKeyFrame(currentFrame);
                else
                    cout << "Init:\tNo fixed map points in frame!" << endl;
            }
        }
        else // We have initial frame -> try to form inital map with current frame
        {

            mvpInitializer[mCurrentCameraID]->UpdateReferenceFrame();
            Frame referenceFrame = mvpInitializer[mCurrentCameraID]->GetReferenceFrame();

            // Find correspondences
            ORBmatcher matcher(0.9, true);
            int nmatches = matcher.SearchForInitialization(referenceFrame, currentFrame,
                                                           mvpInitializer[mCurrentCameraID]->mvbPrevMatched,
                                                           mvpInitializer[mCurrentCameraID]->mvIniMatches, 100);

            // If too few matches... Increase search region
            if (mbUseOdometry && nmatches < 20)
            {
                nmatches = matcher.SearchForInitialization(referenceFrame, currentFrame,
                                                           mvpInitializer[mCurrentCameraID]->mvbPrevMatched,
                                                           mvpInitializer[mCurrentCameraID]->mvIniMatches, 400);
            }

            // Check if there are enough correspondences
            bool enoughMatches = false;
            if (mbUseOdometry && nmatches >= 30)
                enoughMatches = true;
            else if (nmatches >= 100)
                enoughMatches = true;

            // Do we get enough matches?
            if (!enoughMatches)
            {

                // Check if initial frame contains fixed map matches before discarding
                //mvpInitializer[mCurrentCameraID]->UpdateReferenceFrame();
                Frame initFrame = mvpInitializer[mCurrentCameraID]->mReferenceFrame;

                // Clean VO matches
                for (int i = 0; i < initFrame.N; i++)
                {
                    MapPoint *pMP = initFrame.mvpMapPoints[i];
                    if (pMP)
                        if (pMP->Observations() < 1)
                        {
                            initFrame.mvbOutlier[i] = false;
                            initFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                        }
                }

                // If reference frame has fix map matches add it as a key frame anyway
                if (mSystemState == INITIALIZED && initFrame.mFixedMapMatches.nMatches > 5 && GetDistanceLastFixedKF(mCurrentCameraID) > 5)
                    CreateNewFixedKeyFrame(mvpInitializer[mCurrentCameraID]);

                mvpInitializer[mCurrentCameraID] = new Initializer(currentFrame, mpLastKeyFrame, mOdometryLastKF, 3.0, 200);
                mState[mCurrentCameraID] = NOT_INITIALIZED;

                cout << "Init:\tTrying to initialize but not enough matches (" << nmatches << " found)." << endl;

                return;
            }

            // We have enough matches!

            cv::Mat Rcw;                 // Current Camera Rotation
            cv::Mat tcw;                 // Current Camera Translation
            vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

            // Try to initialize
            if (mvpInitializer[mCurrentCameraID]->Initialize(currentFrame, mvpInitializer[mCurrentCameraID]->mvIniMatches, Rcw, tcw, vbTriangulated))
            {
                vector<int> &vMatches = mvpInitializer[mCurrentCameraID]->mvIniMatches;

                for (size_t i = 0, iend = vMatches.size(); i < iend; i++)
                {
                    if (vMatches[i] >= 0 && !vbTriangulated[i])
                    {
                        vMatches[i] = -1;
                        nmatches--;
                    }
                }

                // Set Frame Poses
                Frame &initailFrame = mvpInitializer[mCurrentCameraID]->mReferenceFrame;
                cv::Mat Tbw1 = initailFrame.mTbw;
                cv::Mat Tcw1 = initailFrame.mTcb * Tbw1;

                // Initial camera is rotated and translated according to initial estimate
                cv::Mat T21 = cv::Mat::eye(4, 4, CV_32F);
                Rcw.copyTo(T21.rowRange(0, 3).colRange(0, 3));
                tcw.copyTo(T21.rowRange(0, 3).col(3));

                cv::Mat Tbw2 = msCameraCalibration[mCurrentCameraID].Tcb.inv() * T21 * Tcw1;

                currentFrame.SetPose(Tbw2);

                CreateInitialMapMonocular();
            }
            else // Initialization failed! Better luck next time...
            {
                // Check if initial frame contains fixed map matches before discarding
                mvpInitializer[mCurrentCameraID]->UpdateReferenceFrame();
                Frame initFrame = mvpInitializer[mCurrentCameraID]->mReferenceFrame;

                // Clean VO matches
                for (int i = 0; i < initFrame.N; i++)
                {
                    MapPoint *pMP = initFrame.mvpMapPoints[i];
                    if (pMP)
                        if (pMP->Observations() < 1)
                        {
                            initFrame.mvbOutlier[i] = false;
                            initFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                        }
                }

                // If reference frame has fix map matches add it as a key frame anyway
                if (mSystemState == INITIALIZED && initFrame.mFixedMapMatches.nMatches > 5 && GetDistanceLastFixedKF(mCurrentCameraID) > 5)
                    CreateNewFixedKeyFrame(mvpInitializer[mCurrentCameraID]);

                mvpInitializer[mCurrentCameraID] = new Initializer(currentFrame, mpLastKeyFrame, mOdometryLastKF, 3.0, 200);
                mState[mCurrentCameraID] = NOT_INITIALIZED;
            }
        }
    }

    void Tracking::CreateInitialMapMonocular()
    {
        // Set set to current camera
        eSingleTrackingState *pState = &mState[mCurrentCameraID];
        Frame &currentFrame = mvCurrentFrame[mCurrentCameraID];
        Frame &referenceFrame = mvpInitializer[mCurrentCameraID]->mReferenceFrame;
        Frame &previousBodyFrame = mvPreviousFrames[mLastCameraID];

        OdometryData previousOdo;

        // Wait for local mapping to finish before inserting new KeyFrames
        while (!mpLocalMapper->AcceptKeyFrames())
        {
            usleep(300);
        }

        KeyFrame *pKFini, *pKFcur;

        KeyFrame *firstKF = mvpInitializer[mCurrentCameraID]->mpLastKF;
        OdometryData deltaOdo(mvpInitializer[mCurrentCameraID]->mOdometryDelta);
        OdometryData initOdo(mvpInitializer[mCurrentCameraID]->mOdoLastKF2RefFrame);

        if (mSystemState != INITIALIZED)
        {
            // We are not tracking in any camera. Create initial key frames.
            pKFini = new KeyFrame(referenceFrame, mpMap, mpKeyFrameDB, referenceFrame.mOdometry, NULL);
            pKFcur = new KeyFrame(currentFrame, mpMap, mpKeyFrameDB, deltaOdo, pKFini);
        }
        else
        {

            // find where to insert initial frame
            KeyFrame *pNextKF = firstKF->GetNextKeyFrame();
            KeyFrame *pPrevKF = firstKF;
            while (pNextKF && pNextKF->mTimeStamp < referenceFrame.mTimeStamp)
            {

                initOdo.RemoveFirst(pNextKF->mOdometry);
                pPrevKF = pNextKF;
                pNextKF = pNextKF->GetNextKeyFrame();
            }

            pKFini = new KeyFrame(referenceFrame, mpMap, mpKeyFrameDB, initOdo, pPrevKF);

            // find where to insert current frame
            pNextKF = pKFini->GetNextKeyFrame();
            pPrevKF = pKFini;
            while (pNextKF && pNextKF->mTimeStamp < currentFrame.mTimeStamp)
            {
                deltaOdo.RemoveFirst(pNextKF->mOdometry);
                pPrevKF = pNextKF;
                pNextKF = pNextKF->GetNextKeyFrame();
            }

            pKFcur = new KeyFrame(currentFrame, mpMap, mpKeyFrameDB, deltaOdo, pPrevKF);
        }

        pKFini->ComputeBoW();
        pKFcur->ComputeBoW();

        // If available, use odometry to scale the map
        cv::Mat Tbw1 = pKFini->GetPose();
        cv::Mat Tbw2 = pKFcur->GetPose();

        // Get scale from odometry
        float scale = 1;
        if (mbUseOdometry)
        {
            cv::Mat Tcb = currentFrame.mTcb;

            // Expected position of camera 2 from odometry
            cv::Mat Tcw2 = Tcb * mvpInitializer[mCurrentCameraID]->mOdometryDelta.Predict(Tbw1);

            // Position of camera 1
            cv::Mat Tcw1 = Tcb * Tbw1;

            // Expected baseline between cameras from odometry
            cv::Mat odo_T21 = Tcw2 * Tcw1.inv();
            cv::Mat odo_t21;
            odo_T21.col(3).rowRange(0, 3).copyTo(odo_t21);

            float odoBaseline = cv::norm(odo_t21);

            // Camera baseline after initialization
            Tcw1 = Tcb * Tbw1;
            Tcw2 = Tcb * Tbw2;

            cv::Mat T21 = Tcw2 * Tcw1.inv();
            cv::Mat t21;
            T21.col(3).rowRange(0, 3).copyTo(t21);

            float baseline = cv::norm(t21);

            // Calculate scale
            scale = odoBaseline / baseline;

            // Rescale camera 2 in relation to camera 1
            t21 = t21 * scale;
            t21.copyTo(T21.col(3).rowRange(0, 3));
            Tcw2 = T21 * Tcw1;

            // Update camera 2
            pKFcur->SetPose(Tcb.inv() * Tcw2);
        }

        // Insert KFs in the map
        mpMap->AddKeyFrame(pKFini);
        mpMap->AddKeyFrame(pKFcur);

        // Get initial map points and matches
        vector<int> vIniMatches = mvpInitializer[mCurrentCameraID]->mvIniMatches;
        vector<cv::Point3f> v3P = mvpInitializer[mCurrentCameraID]->mvIniP3D;

        // Create MapPoints and associated to keyframes
        int nNewMapPoints = 0;

        // Get camera transformation
        cv::Mat Tcw = referenceFrame.mTcb * referenceFrame.mTbw;
        cv::Mat Twc = Tcw.inv();
        cv::Mat Rwc = Twc.rowRange(0, 3).colRange(0, 3);
        cv::Mat twc = Twc.rowRange(0, 3).col(3);

        for (size_t i = 0; i < vIniMatches.size(); i++)
        {
            if (vIniMatches[i] < 0)
                continue;

            //Create MapPoint.
            cv::Mat cameraPos(v3P[i]);
            cameraPos *= scale;

            cv::Mat worldPos = Rwc * cameraPos + twc;

            MapPoint *pMP = new MapPoint(worldPos, pKFcur, mpMap);

            pKFini->AddMapPoint(pMP, i);
            pKFcur->AddMapPoint(pMP, vIniMatches[i]);

            pMP->AddObservation(pKFini, i);
            pMP->AddObservation(pKFcur, vIniMatches[i]);

            pMP->ComputeDistinctiveDescriptors();
            pMP->UpdateNormalAndDepth();

            // Fill Current Frame structure
            currentFrame.mvpMapPoints[vIniMatches[i]] = pMP;
            currentFrame.mvbOutlier[vIniMatches[i]] = false;

            // Add to Map
            mpMap->AddMapPoint(pMP);
            nNewMapPoints++;
        }

        // Update Connections
        pKFini->UpdateConnections();
        pKFcur->UpdateConnections();

        // Bundle Adjustment
        std::cout << "Tracking:\tInitiated map for camera " << mCurrentCameraID << " with " << nNewMapPoints << " points" << endl;

        // Calculate odometry between init frames
        OdometryData Odo;

        KeyFrame *pKF = pKFini;
        while (pKF->GetNextKeyFrame() != NULL && pKF != pKFcur)
        {
            pKF = pKF->GetNextKeyFrame();
            Odo.Integrate(pKF->mOdometry);
        }

        vector<KeyFrame *> pInitKFs;
        pInitKFs.push_back(pKFini);
        pInitKFs.push_back(pKFcur);

        Optimizer::InitialBundleAdjustment(pInitKFs, Odo, mpMap);

        // Check initial map
        float medianDepth = pKFini->ComputeSceneMedianDepth(2);
        if (medianDepth < 0 || pKFcur->TrackedMapPoints(1) < 20)
        {
            std::cout << "Init:\tToo few tracked points (" << pKFcur->TrackedMapPoints(1) << "), resetting..." << endl;

            // Cleanup
            if (mbUseOdometry)
            {
                if (mSystemState >= INITIALIZING && AnyCameraTracking())
                {
                    currentFrame.PredictAndSetPose(previousBodyFrame.mTbw);
                }
                else if (mpLastKeyFrame != NULL)
                {
                    cv::Mat Tbw1 = mpLastKeyFrame->GetPose();
                    cv::Mat Tbw2 = mOdometryLastKF.Predict(Tbw1);
                    currentFrame.SetPose(Tbw2);
                }

                mpMapDrawer->SetPredictedCameraPose(currentFrame.mTcb * currentFrame.mTbw);
                mpMapDrawer->mbUpdateCurrentCamera = true;
            }

            // Correct KeyFrame pointers
            if (AnyCameraTracking() || AnyCameraReseted())
            {
                pKFini->SetBadFlag();
                pKFcur->SetBadFlag();
            }
            mpMap->EraseKeyFrame(pKFini);
            mpMap->EraseKeyFrame(pKFcur);

            // Check if init frame contain fixed map matches
            Frame referenceFrame = mvpInitializer[mCurrentCameraID]->GetReferenceFrame();
            if (!referenceFrame.isKeyFrame && referenceFrame.mFixedMapMatches.nMatches > 5 && GetDistanceLastFixedKF(mCurrentCameraID) > 5)
                CreateNewFixedKeyFrame(mvpInitializer[mCurrentCameraID]);

            Reset();
            return;
        }

        // If no odometry, use median depth to scale
        float invMedianDepth = 1.0f / medianDepth;
        if (!mbUseOdometry)
        {
            // Scale initial baseline
            cv::Mat Tbw1 = pKFini->GetPose();
            Tbw1.col(3).rowRange(0, 3) = Tbw1.col(3).rowRange(0, 3) * invMedianDepth;
            pKFini->SetPose(Tbw1);

            cv::Mat Tbw2 = pKFcur->GetPose();
            Tbw2.col(3).rowRange(0, 3) = Tbw2.col(3).rowRange(0, 3) * invMedianDepth;
            pKFcur->SetPose(Tbw2);

            vector<MapPoint *> vpAllMapPoints = pKFini->GetMapPointMatches();
            for (size_t iMP = 0; iMP < vpAllMapPoints.size(); iMP++)
            {
                if (vpAllMapPoints[iMP])
                {
                    MapPoint *pMP = vpAllMapPoints[iMP];
                    pMP->SetWorldPos(pMP->GetWorldPos() * invMedianDepth);
                }
            }
        }

        mpLocalMapper->InsertKeyFrame(pKFini);
        mpLocalMapper->InsertKeyFrame(pKFcur);

        currentFrame.SetPose(pKFcur->GetPose());
        mnLastKFFrameId = currentFrame.mnId;

        // Update last KF if current KF has no children.
        mvpReferenceKF[pKF->mCameraID] = pKF;
        if (pKFcur->GetNextKeyFrame() == NULL)
        {

            // Check that times are consistent
            if (mpLastKeyFrame && pKFcur->mTimeStamp < mpLastKeyFrame->mTimeStamp)
            {
                throw std::runtime_error("Init: Trying to update mpLastKeyFrame with a older key frame!");
            }

            mpLastKeyFrame = pKFcur;
            mOdometryLastKF.Clear();
        }

        mpMapDrawer->SetCurrentCameraPose(currentFrame.mTcb * currentFrame.mTbw);
        mpMapDrawer->SetViewCameraPose(currentFrame.mTbw);
        mpMapDrawer->mbUpdateCurrentCamera = true;

        mpMapDrawer->SetCurrentKeyFrame(mpLastKeyFrame);

        // Update key frame pointers in the initializers for the other cameras
        for (int i = 0; i < mnCameras; i++)
        {
            if (i != mCurrentCameraID && mvpInitializer[i] != NULL && mState[i] != OK)
            {
                // Update key point references
                if (!mvpInitializer[i]->mpLastKF)
                {
                    mvpInitializer[i]->mpLastKF = pKFini;
                    mvpInitializer[i]->mOdoLastKF2RefFrame.RemoveFirst(initOdo);
                }
            }
        }
        delete mvpInitializer[mCurrentCameraID];
        mvpInitializer[mCurrentCameraID] = static_cast<Initializer *>(NULL);

        mvpLocalKeyFrames.push_back(pKFcur);
        mvpLocalKeyFrames.push_back(pKFini);

        UpdateLocalKeyFrames();

        //mvpLocalMapPoints = mpMap->GetAllMapPoints();

        if (mvpReferenceKF[mCurrentCameraID])
            currentFrame.mpReferenceKF = mvpReferenceKF[mCurrentCameraID];
        else
            currentFrame.mpReferenceKF = pKFcur;

        currentFrame.mpLastKeyFrame = pKFcur;
        currentFrame.isKeyFrame = true;

        mvPreviousFrames[mCurrentCameraID] = Frame(currentFrame);
        mLastCameraID = mCurrentCameraID;

        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

        mpMapDrawer->SetCurrentCameraPose(pKFcur->GetCameraPose());
        mpMapDrawer->SetViewCameraPose(pKFcur->GetPose());
        mpMapDrawer->mbUpdateCurrentCamera = true;

        mpMap->mvpKeyFrameOrigins.push_back(pKFini);

        (*pState) = OK;
        mSystemState = INITIALIZED;
    }

    void Tracking::CheckReplacedInLastFrame()
    {
        Frame &previousCameraFrame = mvPreviousFrames[mCurrentCameraID];

        for (int i = 0; i < previousCameraFrame.N; i++)
        {
            MapPoint *pMP = previousCameraFrame.mvpMapPoints[i];

            if (pMP)
            {
                MapPoint *pRep = pMP->GetReplaced();
                if (pRep)
                {
                    previousCameraFrame.mvpMapPoints[i] = pRep;
                }
            }
        }
    }

    bool Tracking::TrackReferenceKeyFrame()
    {
        // Set set to current camera
        Frame &currentFrame = mvCurrentFrame[mCurrentCameraID];
        Frame &previousBodyFrame = mvPreviousFrames[mLastCameraID];

        // Compute Bag of Words vector
        currentFrame.ComputeBoW();

        // We perform first an ORB matching with the reference keyframe
        // If enough matches are found we setup a PnP solver
        ORBmatcher matcher(0.7, true);
        vector<MapPoint *> vpMapPointMatches;

        int nmatches = matcher.SearchByBoW(mvpReferenceKF[mCurrentCameraID], currentFrame, vpMapPointMatches);

        if (nmatches < 15)
        {
            return false;
        }

        currentFrame.mvpMapPoints = vpMapPointMatches;

        // Resting current frame to prediction
        cout << "TrackRefKF:\t";
        if (mbUseOdometry)
        {
            currentFrame.PredictAndSetPose(previousBodyFrame.mTbw);
        }

        Optimizer::PoseOptimization(&currentFrame, mbTrackWithOdometry);

        // Discard outliers
        int nmatchesMap = 0;
        for (int i = 0; i < currentFrame.N; i++)
        {
            if (currentFrame.mvpMapPoints[i])
            {
                if (currentFrame.mvbOutlier[i])
                {
                    MapPoint *pMP = currentFrame.mvpMapPoints[i];

                    currentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                    currentFrame.mvbOutlier[i] = false;
                    pMP->mbTrackInView = false;
                    pMP->mnLastFrameSeen = currentFrame.mnId;
                    nmatches--;
                }
                else if (currentFrame.mvpMapPoints[i]->Observations() > 0)
                    nmatchesMap++;
            }
        }

        if (nmatchesMap < 10)
            cout << "TrackWithRef:\tToo few map matches (" << nmatchesMap << ")" << endl;

        return nmatchesMap >= 10;
    }

    void Tracking::UpdateLastFrame()
    {

        for (int i = 0; i < mnCameras; i++)
        {

            Frame &previousCameraFrame = mvPreviousFrames[i];

            // Update pose according to reference keyframe
            KeyFrame *pRef = previousCameraFrame.mpLastKeyFrame;
            cv::Mat Tbr = previousCameraFrame.mTbr;

            if (!Tbr.empty())
            {
                previousCameraFrame.SetPose(Tbr * pRef->GetPose());
            }
        }
    }

    bool Tracking::TrackWithMotionModel()
    {

        // Set set to current camera and the previous camera
        Frame &currentFrame = mvCurrentFrame[mCurrentCameraID];
        Frame &previousCameraFrame = mvPreviousFrames[mCurrentCameraID];
        Frame &previousBodyFrame = mvPreviousFrames[mLastCameraID];

        // Construct matcher
        ORBmatcher matcher(0.9, true);

        // Update last frame pose according to its reference keyframe
        // Create "visual odometry" points if in Localization Mode
        UpdateLastFrame();

        fill(currentFrame.mvpMapPoints.begin(), currentFrame.mvpMapPoints.end(), static_cast<MapPoint *>(NULL));

        // Project points seen in previous frame
        int th;
        if (mSensor != System::STEREO)
            //th=15;
            if (mbTrackWithOdometry)
                th = 7; // Use tighter threshold if we use odometry to predict?
            else
                th = 15;
        else
            th = 7;
        int nmatches = matcher.SearchByProjection(currentFrame, previousCameraFrame, th, mSensor == System::MONOCULAR);

        // If few matches, uses a wider window search
        if (nmatches < 20)
        {

            fill(currentFrame.mvpMapPoints.begin(), currentFrame.mvpMapPoints.end(), static_cast<MapPoint *>(NULL));
            nmatches = matcher.SearchByProjection(currentFrame, previousCameraFrame, 2 * th, mSensor == System::MONOCULAR);
        }

        if (mbUseOdometry)
        {
            // Optimize frame pose with all matches
            cout << "Tracking:\t";
            Optimizer::PoseOptimization(&currentFrame, true);
        }
        else
        {
            // Check if enough matches before optimizing pose without odometry constraint.
            if (nmatches < 20)
            {
                return false;
            }
            Optimizer::PoseOptimization(&currentFrame);
        }

        // Discard outliers
        int nmatchesMap = 0;
        for (int i = 0; i < currentFrame.N; i++)
        {
            if (currentFrame.mvpMapPoints[i])
            {
                if (currentFrame.mvbOutlier[i])
                {
                    MapPoint *pMP = currentFrame.mvpMapPoints[i];

                    currentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                    currentFrame.mvbOutlier[i] = false;
                    pMP->mbTrackInView = false;
                    pMP->mnLastFrameSeen = currentFrame.mnId;
                    nmatches--;
                }
                else if (currentFrame.mvpMapPoints[i]->Observations() > 0)
                    nmatchesMap++;
            }
        }

        // Update visualization
        mpMapDrawer->SetCurrentCameraPose(currentFrame.mTcb * currentFrame.mTbw);
        mpMapDrawer->SetViewCameraPose(currentFrame.mTbw);
        mpMapDrawer->mbUpdateCurrentCamera = true;

        return nmatchesMap >= 10;
    }

    bool Tracking::TrackLocalMap()
    {

        // Set set to current camera and the previous camera
        Frame &currentFrame = mvCurrentFrame[mCurrentCameraID];

        // We have an estimation of the camera pose and some map points tracked in the frame.
        // We retrieve the local map and try to find matches to points in the local map.
        UpdateLocalMap();

        SearchLocalPoints();

        // Optimize Pose
        cout << "TrackLocalMap:\t";
        Optimizer::PoseOptimization(&currentFrame, mbTrackWithOdometry);

        mnMatchesInliers = 0;

        // Update MapPoints Statistics
        int nTotalMatches = 0;
        for (int i = 0; i < currentFrame.N; i++)
        {
            if (currentFrame.mvpMapPoints[i])
            {
                nTotalMatches++;
                if (!currentFrame.mvbOutlier[i])
                {
                    //if (!mbOnlyTracking)
                    //{
                    if (currentFrame.mvpMapPoints[i]->Observations() > 0)
                        mnMatchesInliers++;
                    //}
                    //else
                    //    mnMatchesInliers++;
                }
                else if (mSensor == System::STEREO)
                    currentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
            }
        }

        // Indicate weak tracking to insert new KF
        if (mnMatchesInliers < 40)
            mvbGoodMapTracking[mCurrentCameraID] = false;
        else
            mvbGoodMapTracking[mCurrentCameraID] = true;

        // Decide if the tracking was successful
        // More restrictive if there was a relocalization recently

        // Use lower threshold of odometry information is present
        if (mbUseOdometry)
        {

            if (currentFrame.mnId < mnLastRelocFrameId + mMaxFrames && (mnMatchesInliers < 20 && (float)mnMatchesInliers / (float)nTotalMatches < 0.7))
            {
                cout << "TrackLocalMap:\tToo few matched inliers (" << mnMatchesInliers << ") for camera "
                     << mCurrentCameraID << ". Resetting..." << endl;
                return false;
            }
            else if (mnMatchesInliers < 10)
            {
                cout << "TrackLocalMap:\tToo few matched inliers (" << mnMatchesInliers << ") for camera "
                     << mCurrentCameraID << ". Resetting..." << endl;
                return false;
            }
            else
                return true;
        }
        else
        {
            if (currentFrame.mnId < mnLastRelocFrameId + mMaxFrames && mnMatchesInliers < 50)
            {
                cout << "TrackLocalMap:\tToo few matched inliers (" << mnMatchesInliers << ") for camera "
                     << mCurrentCameraID << ". Resetting..." << endl;
                return false;
            }

            if (mnMatchesInliers < 30)
            {
                cout << endl
                     << "TrackLocalMap: Too few matched inliers (" << mnMatchesInliers << ") for camera "
                     << mCurrentCameraID << ". Resetting..." << endl;
                return false;
            }
            else
                return true;
        }
    }

    bool Tracking::NeedNewKeyFrame()
    {
        // Get Map Mutex -> Map cannot be changed
        unique_lock<recursive_mutex> lock(mpMap->mMutexMapUpdate);

        // Set set to current camera
        Frame &currentFrame = mvCurrentFrame[mCurrentCameraID];

        // LocSLAM is currenty not real-time and LocalMapping is run in the Tracking thread
        // If Local Mapping is freezed by a Loop Closure do not insert keyframes
        //if (mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        //    return false;

        const int nKFs = mpMap->KeyFramesInMapForCamera(mCurrentCameraID);

        // Do not insert keyframes if not enough frames have passed from last relocalization
        if (currentFrame.mnId < mnLastRelocFrameId + mMaxFrames && nKFs > mMaxFrames)
            return false;

        // Tracked MapPoints in the reference keyframe
        int nMinObs = 3;
        if (nKFs <= 2 || mbWasCameraReseted[mCurrentCameraID])
            nMinObs = 2;
        int nRefMatches = mvpReferenceKF[mCurrentCameraID]->TrackedMapPoints(nMinObs);

        // Local Mapping accept keyframes?
        bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

        // Check how many "close" points are being tracked and how many could be potentially created.
        int nNonTrackedClose = 0;
        int nTrackedClose = 0;
        if (mSensor != System::MONOCULAR)
        {
            for (int i = 0; i < currentFrame.N; i++)
            {
                if (currentFrame.mvDepth[i] > 0 && currentFrame.mvDepth[i] < mThDepth)
                {
                    if (currentFrame.mvpMapPoints[i] && !currentFrame.mvbOutlier[i])
                        nTrackedClose++;
                    else
                        nNonTrackedClose++;
                }
            }
        }

        bool bNeedToInsertClose = (nTrackedClose < 100) && (nNonTrackedClose > 70);

        // Check if fixed map matches in frame
        bool bFixedMapMatches = currentFrame.mFixedMapMatches.nMatches > 5;

        double covarianceSpread = pow(mOdometryLastKF.GetCovariance().determinant(), 1.0 / 6.0);
        const double covThreshold = pow(0.1 * 0.1 * CV_PI / 180.0, 1.0 / 2.0);

        bool bUncertainOdometry = (covarianceSpread > covThreshold);

        // If to scale, check distance to last key frame
        const double transThreshold = 2;
        double transDist = transThreshold;
        const double rotThreshold = 5.0 * CV_PI / 180.0;
        double rotDist = rotThreshold;
        if (mbUseOdometry)
        {
            g2o::SE3Quat T21 = Converter::toSE3Quat(currentFrame.mTbw * mpLastKeyFrame->GetPoseInverse());

            transDist = T21.translation().norm();
            rotDist = T21.rotation().vec().norm();
        }

        // Thresholds
        float thRefRatio = 0.75f;
        if (nKFs < 2)
            thRefRatio = 0.4f;

        if (mSensor == System::MONOCULAR)
            if (transDist >= transThreshold || rotDist >= rotThreshold)
                thRefRatio = 0.75f;
            else
                thRefRatio = 0.6f;

        int nTrackingCameras = 0;
        for (int i = 0; i < mnCameras; i++)
            if (mState[i] == OK)
                nTrackingCameras++;

        //int currMinFrames = mMinFrames * nTrackingCameras;
        int currMinFrames = 3 * nTrackingCameras;

        // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
        //const bool c1a = currentFrame.mnId >= mnLastKFFrameId + mMaxFrames;
        const bool c1a = false;
        // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
        //const bool c1b = (currentFrame.mnId >= mnLastKFFrameId + currMinFrames && false); // bLocalMappingIdle);
        const bool c1b = false;
        // Condition 1c: tracking is weak
        //const bool c1c = mSensor != System::MONOCULAR && (mnMatchesInliers < nRefMatches * thRefRatio || bNeedToInsertClose);
        const bool c1c = nRefMatches < 20 || mnMatchesInliers < nRefMatches * thRefRatio || bNeedToInsertClose;

        // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
        //const bool c2a = ((mnMatchesInliers < nRefMatches * thRefRatio || bNeedToInsertClose) && mnMatchesInliers > 15);
        const bool c2a = mnMatchesInliers > 10;
        // Condition 2b: Too few map points in reference KF

        // Condition 3: Need to be sufficient distance or heading change since last KF
        //const bool c3 = !mvbGoodMapTracking[mCurrentCameraID] && (transDist >= transThreshold || rotDist >= rotThreshold); //nRefMatches < 25;
        const bool c3 = false;
        // Condition 4: Enough fixed map matches
        const bool c4 = bFixedMapMatches && GetDistanceLastFixedKF(mCurrentCameraID) > 10;

        // Condition 5: Too uncertain odometry.
        const bool c5 = bUncertainOdometry && (mnMatchesInliers > 10);

        if (((c1a || c1b || c1c) && c2a) || c3 || c4 || c5)
        {
            // If the mapping accepts keyframes, insert keyframe.
            // Otherwise send a signal to interrupt BA
            if (bLocalMappingIdle) // this is always true as long as LocalMapping and Tracking are runnning in the same thread.
            {
                cout << "Tracking:\tAdding new KF on cam" << mCurrentCameraID << " (reason(s): ";

                if (((c1a || c1b || c1c) && c2a))
                    cout << "Track ";
                if (c3)
                    cout << "Dist ";
                if (c4)
                    cout << "FixDist ";
                if (c5)
                    cout << "Uncert ";

                cout << ")" << endl;

                return true;
            }
            else
            {

                // LocSLAM is currenty not real-time and LocalMapping is run in the Tracking thread
                /*
                // Release map update lock to allow local bundle to finish.
                lock.unlock();

                // Instead of interupt wait for local bundle to finish
                while (!mpLocalMapper->AcceptKeyFrames())
                {
                    usleep(300);
                }

                mpLocalMapper->InterruptBA();
                */

                if (mSensor != System::MONOCULAR)
                {
                    if (mpLocalMapper->KeyframesInQueue() < 3)
                    {
                        return true;
                    }
                    else
                    {
                        return false;
                    }
                }
                else
                {
                    return false;
                }
            }
        }
        else
        {
            return false;
        }
    }

    void Tracking::CreateNewKeyFrame()
    {
        // Get Map Mutex -> Map cannot be changed
        unique_lock<recursive_mutex> lock(mpMap->mMutexMapUpdate);

        // Set set to current camera
        Frame &currentFrame = mvCurrentFrame[mCurrentCameraID];

        //if (!mpLocalMapper->SetNotStop(true))
        //    return;

        // Add key frame in all tracking cameras
        vector<pair<double, Frame *>> vNewFrames;

        vNewFrames.push_back(make_pair(currentFrame.mTimeStamp, &currentFrame));

        // Uncomment code if to add KFs for all cameras in a rig
        /*
        for (int i = 0; i < mnCameras; i++)
        {
            if (mState[i] == OK && i != mCurrentCameraID && !mvPreviousFrames[i].isKeyFrame)
            {
                vNewFrames.push_back(make_pair(mvPreviousFrames[i].mTimeStamp, &mvPreviousFrames[i]));
            }
        }
        */

        // Sort frames by time stamp
        sort(vNewFrames.begin(), vNewFrames.end());

        // Add key frames
        vector<KeyFrame *> vpNewKFs;
        for (int i = 0; i < vNewFrames.size(); i++)
        {

            // Handle first frame differently
            if (i == 0)
            {
                // Integate odometry
                OdometryData odo;
                for (int j = i + 1; j < vNewFrames.size(); j++)
                    odo.Integrate(vNewFrames[j].second->mOdometry);

                // Remove odomtry of subsequent frames
                mOdometryLastKF.RemoveLast(odo);

                // Add key frame
                KeyFrame *pKF = new KeyFrame((*vNewFrames[i].second), mpMap, mpKeyFrameDB, mOdometryLastKF, mpLastKeyFrame);
                vpNewKFs.push_back(pKF);

                mvpReferenceKF[pKF->mCameraID] = pKF;
                vNewFrames[i].second->mpReferenceKF = pKF;
                vNewFrames[i].second->mpLastKeyFrame = pKF;
                vNewFrames[i].second->mTbr = cv::Mat::eye(4, 4, CV_32F);
                vNewFrames[i].second->isKeyFrame = true;

                // Update frame trajectory (to get correct results)
                list<KeyFrame *>::iterator lRit = mlpReferences.end();
                list<string>::iterator lFileName = mlstrFileNames.end();
                for (list<cv::Mat>::iterator lit = mlRelativeFramePoses.end(), lbegin = mlRelativeFramePoses.begin(); lit != lbegin; lit--, lRit--, lFileName--)
                {
                    if ((*lFileName) == vNewFrames[i].second->mstrImgFileName)
                    {
                        (*lit) = cv::Mat::eye(4, 4, CV_32F);
                        (*lRit) = pKF;
                        break;
                    }
                }
            }
            else
            {
                KeyFrame *pKF = new KeyFrame((*vNewFrames[i].second), mpMap, mpKeyFrameDB, vNewFrames[i].second->mOdometry, vpNewKFs.back());
                vpNewKFs.push_back(pKF);

                mvpReferenceKF[pKF->mCameraID] = pKF;
                vNewFrames[i].second->mpReferenceKF = pKF;
                vNewFrames[i].second->mpLastKeyFrame = pKF;
                vNewFrames[i].second->mTbr = cv::Mat::eye(4, 4, CV_32F);
                vNewFrames[i].second->isKeyFrame = true;

                // Update frame trajectory (to get correct results)
                list<KeyFrame *>::iterator lRit = mlpReferences.end();
                list<string>::iterator lFileName = mlstrFileNames.end();
                for (list<cv::Mat>::iterator lit = mlRelativeFramePoses.end(), lbegin = mlRelativeFramePoses.begin(); lit != lbegin; lit--, lRit--, lFileName--)
                {
                    if ((*lFileName) == vNewFrames[i].second->mstrImgFileName)
                    {
                        (*lit) = cv::Mat::eye(4, 4, CV_32F);
                        (*lRit) = pKF;
                        break;
                    }
                }
            }
        }

        // Clear odometry history
        mOdometryLastKF.Clear();

        // **************************************************************
        // Legacy ORB SLAM code for stereo camera not currently supported
        // **************************************************************
        /*
        if (mSensor != System::MONOCULAR)
        {
            currentFrame.UpdatePoseMatrices();

            // We sort points by the measured depth by the stereo/RGBD sensor.
            // We create all those MapPoints whose depth < mThDepth.
            // If there are less than 100 close points we create the 100 closest.
            vector<pair<float, int>> vDepthIdx;
            vDepthIdx.reserve(currentFrame.N);
            for (int i = 0; i < currentFrame.N; i++)
            {
                float z = currentFrame.mvDepth[i];
                if (z > 0)
                {
                    vDepthIdx.push_back(make_pair(z, i));
                }
            }

            if (!vDepthIdx.empty())
            {
                std::sort(vDepthIdx.begin(), vDepthIdx.end());

                int nPoints = 0;
                for (size_t j = 0; j < vDepthIdx.size(); j++)
                {
                    int i = vDepthIdx[j].second;

                    bool bCreateNew = false;

                    MapPoint *pMP = currentFrame.mvpMapPoints[i];
                    if (!pMP)
                        bCreateNew = true;
                    else if (pMP->Observations() < 1)
                    {
                        bCreateNew = true;
                        currentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                    }

                    if (bCreateNew)
                    {
                        cv::Mat x3D = currentFrame.UnprojectStereo(i);
                        MapPoint *pNewMP = new MapPoint(x3D, pKF, mpMap);
                        pNewMP->AddObservation(pKF, i);
                        pKF->AddMapPoint(pNewMP, i);
                        pNewMP->ComputeDistinctiveDescriptors();
                        pNewMP->UpdateNormalAndDepth();
                        mpMap->AddMapPoint(pNewMP);

                        currentFrame.mvpMapPoints[i] = pNewMP;
                        nPoints++;
                    }
                    else
                    {
                        nPoints++;
                    }

                    if (vDepthIdx[j].first > mThDepth && nPoints > 100)
                        break;
                }
            }
        }
        */

        for (int i = 0; i < vpNewKFs.size(); i++)
        {
            mpLocalMapper->InsertKeyFrame(vpNewKFs[i]);
        }

        // New keyframe inserted -> camera no longer in restet mode
        mbWasCameraReseted[mCurrentCameraID] = false;

        // Check if there are fixed map matches in KeyFrames -> insert in fixed map
        for (int i = 0; i < vpNewKFs.size(); i++)
            if (vpNewKFs[i]->mFixedMapMatches.nMatches > 5 && mnLastFixedMapFrameId < vpNewKFs[i]->mnFrameId)
            {
                mnLastFixedMapFrameId = vpNewKFs[i]->mnFrameId;
                mOdoLastFixedKF[vpNewKFs[i]->mCameraID].Clear();
            }

        // LocSLAM is currenty not real-time and LocalMapping is run in the Tracking thread
        //mpLocalMapper->SetNotStop(false);

        mnLastKFFrameId = currentFrame.mnId;
        mpLastKeyFrame = vpNewKFs.back();

        mpMapDrawer->SetCurrentKeyFrame(mpLastKeyFrame);
    }

    void Tracking::CreateNewFixedKeyFrame(Initializer *pInitializer)
    {
        // Get Map Mutex -> Map cannot be changed
        unique_lock<recursive_mutex> lock(mpMap->mMutexMapUpdate);

        KeyFrame *firstKF = pInitializer->mpLastKF;
        Frame referenceFrame = pInitializer->GetReferenceFrame();

        OdometryData initOdo(pInitializer->mOdoLastKF2RefFrame);

        // find where to insert current frame
        KeyFrame *pNextKF = firstKF->GetNextKeyFrame();
        KeyFrame *pPrevKF = firstKF;
        while (pNextKF && pNextKF->mTimeStamp < referenceFrame.mTimeStamp)
        {
            initOdo.RemoveFirst(pNextKF->mOdometry);
            pPrevKF = pNextKF;
            pNextKF = pNextKF->GetNextKeyFrame();
        }

        KeyFrame *pKF;
        pKF = new KeyFrame(referenceFrame, mpMap, mpKeyFrameDB, initOdo, pPrevKF);

        // Update key frame pointers in the initializers for the other cameras
        for (int i = 0; i < mnCameras; i++)
        {
            if (i != mCurrentCameraID && mvpInitializer[i] != NULL && mState[i] != OK)
            {
                // Update key frame references
                if (!mvpInitializer[i]->mpLastKF)
                {
                    mvpInitializer[i]->mpLastKF = pKF;
                    mvpInitializer[i]->mOdoLastKF2RefFrame.RemoveFirst(initOdo);
                }
            }
        }

        referenceFrame.mpReferenceKF = pKF;
        referenceFrame.mpLastKeyFrame = pKF;
        referenceFrame.mTbr = cv::Mat::eye(4, 4, CV_32F);
        referenceFrame.isKeyFrame = true;

        // Update frame trajectory (to get correct results)
        list<KeyFrame *>::iterator lRit = mlpReferences.end();
        list<string>::iterator lFileName = mlstrFileNames.end();
        for (list<cv::Mat>::iterator lit = mlRelativeFramePoses.end(), lbegin = mlRelativeFramePoses.begin(); lit != lbegin; lit--, lRit--, lFileName--)
        {
            if ((*lFileName) == referenceFrame.mstrImgFileName)
            {
                (*lit) = cv::Mat::eye(4, 4, CV_32F);
                (*lRit) = pKF;
                break;
            }
        }

        if (pKF->mTimeStamp > mpLastKeyFrame->mTimeStamp)
        {

            for (int i = 0; i < mnCameras; i++)
            {
                if (mvPreviousFrames[i].mnId == referenceFrame.mnId)
                {
                    mvPreviousFrames[i].mpLastKeyFrame = pKF;
                    mvPreviousFrames[i].mTbr = cv::Mat::eye(4, 4, CV_32F);
                }
                else if (mvPreviousFrames[i].mTimeStamp >= pKF->mTimeStamp && mvPreviousFrames[i].mpLastKeyFrame->mnId == mpLastKeyFrame->mnId)
                {
                    mvPreviousFrames[i].mpLastKeyFrame = pKF;
                    mvPreviousFrames[i].mTbr = mvPreviousFrames[i].mTbr * initOdo.GetTransformation().inv(); // remove first...
                }
            }

            mpLastKeyFrame = pKF;
            mOdometryLastKF.RemoveFirst(initOdo);
        }

        pKF->UpdateConnections();
        mpLocalMapper->InsertKeyFrame(pKF);

        mnLastFixedMapFrameId = pKF->mnFrameId;
        mOdoLastFixedKF[pKF->mCameraID].Clear();

        mpMapDrawer->SetCurrentKeyFrame(mpLastKeyFrame);
    }

    void Tracking::CreateNewFixedKeyFrame(Frame &newKFFrame)
    {
        // Get Map Mutex -> Map cannot be changed
        unique_lock<recursive_mutex> lock(mpMap->mMutexMapUpdate);

        // Add key frame
        KeyFrame *pKF = new KeyFrame(newKFFrame, mpMap, mpKeyFrameDB, mOdometryLastKF, mpLastKeyFrame);

        mnLastFixedMapFrameId = pKF->mnFrameId;
        mOdoLastFixedKF[pKF->mCameraID].Clear();

        newKFFrame.mpReferenceKF = pKF;
        newKFFrame.mpLastKeyFrame = pKF;
        newKFFrame.mTbr = cv::Mat::eye(4, 4, CV_32F);
        newKFFrame.isKeyFrame = true;

        // Update frame trajectory (to get correct results)
        list<KeyFrame *>::iterator lRit = mlpReferences.end();
        list<string>::iterator lFileName = mlstrFileNames.end();
        for (list<cv::Mat>::iterator lit = mlRelativeFramePoses.end(), lbegin = mlRelativeFramePoses.begin(); lit != lbegin; lit--, lRit--, lFileName--)
        {
            if ((*lFileName) == newKFFrame.mstrImgFileName)
            {
                (*lit) = cv::Mat::eye(4, 4, CV_32F);
                (*lRit) = pKF;
                break;
            }
        }

        mpLocalMapper->InsertKeyFrame(pKF);

        if (pKF->mTimeStamp > mpLastKeyFrame->mTimeStamp)
        {
            mpLastKeyFrame = pKF;
            mnLastKFFrameId = pKF->mnFrameId;
            mOdometryLastKF.Clear();
        }

        mpMapDrawer->SetCurrentKeyFrame(mpLastKeyFrame);
    }

    void Tracking::SearchLocalPoints()
    {
        // Set current camera frame
        Frame &currentFrame = mvCurrentFrame[mCurrentCameraID];

        // Do not search map points already matched
        for (vector<MapPoint *>::iterator vit = currentFrame.mvpMapPoints.begin(), vend = currentFrame.mvpMapPoints.end(); vit != vend; vit++)
        {
            MapPoint *pMP = *vit;
            if (pMP)
            {
                if (pMP->isBad())
                {
                    *vit = static_cast<MapPoint *>(NULL);
                }
                else
                {
                    pMP->IncreaseVisible();
                    pMP->mnLastFrameSeen = currentFrame.mnId;
                    pMP->mbTrackInView = false;
                }
            }
        }

        int nToMatch = 0;

        // Project points in frame and check its visibility
        for (vector<MapPoint *>::iterator vit = mvpLocalMapPoints.begin(), vend = mvpLocalMapPoints.end(); vit != vend; vit++)
        {
            MapPoint *pMP = *vit;
            if (pMP->mnLastFrameSeen == currentFrame.mnId)
                continue;
            if (pMP->isBad())
                continue;
            // Project (this fills MapPoint variables for matching)
            if (currentFrame.isInFrustum(pMP, 0.5))
            {
                pMP->IncreaseVisible();
                nToMatch++;
            }
        }

        if (nToMatch > 0)
        {
            ORBmatcher matcher(0.8);

            int th = 1;

            // If we use odometry to get initial pose, performe a coarser search?
            if (mnCameras > 1)
                th = 1;

            if (mSensor == System::RGBD)
                th = 3;
            // If the camera has been relocalized recently, perform a coarser search
            if (currentFrame.mnId < mnLastRelocFrameId + 2)
                th = 5;
            matcher.SearchByProjection(currentFrame, mvpLocalMapPoints, th);
        }
    }

    void Tracking::UpdateLocalMap()
    {
        // This is for visualization
        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

        // Update (wait for localBA to finish?)
        while (!mpLocalMapper->AcceptKeyFrames())
        {
            usleep(3000);
        }

        unique_lock<recursive_mutex> lock(mpMap->mMutexMapUpdate);

        UpdateLocalKeyFrames();
        UpdateLocalPoints();
    }

    void Tracking::UpdateLocalPoints()
    {
        // Set current camera frame
        Frame &currentFrame = mvCurrentFrame[mCurrentCameraID];

        mvpLocalMapPoints.clear();

        for (vector<KeyFrame *>::const_iterator itKF = mvpLocalKeyFrames.begin(), itEndKF = mvpLocalKeyFrames.end(); itKF != itEndKF; itKF++)
        {
            KeyFrame *pKF = *itKF;
            const vector<MapPoint *> vpMPs = pKF->GetMapPointMatches();

            for (vector<MapPoint *>::const_iterator itMP = vpMPs.begin(), itEndMP = vpMPs.end(); itMP != itEndMP; itMP++)
            {
                MapPoint *pMP = *itMP;
                if (!pMP)
                    continue;
                if (pMP->mnTrackReferenceForFrame == currentFrame.mnId)
                    continue;
                if (!pMP->isBad())
                {
                    mvpLocalMapPoints.push_back(pMP);
                    pMP->mnTrackReferenceForFrame = currentFrame.mnId;
                }
            }
        }
    }

    void Tracking::UpdateLocalKeyFrames()
    {
        // Set current camera
        Frame &currentFrame = mvCurrentFrame[mCurrentCameraID];

        // Each map point vote for the keyframes in which it has been observed
        map<KeyFrame *, int> keyframeCounter;
        for (int i = 0; i < currentFrame.N; i++)
        {
            if (currentFrame.mvpMapPoints[i])
            {
                MapPoint *pMP = currentFrame.mvpMapPoints[i];
                if (!pMP->isBad())
                {
                    const map<KeyFrame *, size_t> observations = pMP->GetObservations();
                    for (map<KeyFrame *, size_t>::const_iterator it = observations.begin(), itend = observations.end(); it != itend; it++)
                        keyframeCounter[it->first]++;
                }
                else
                {
                    currentFrame.mvpMapPoints[i] = NULL;
                }
            }
        }

        if (keyframeCounter.empty())
            return;

        int max = 0;
        KeyFrame *pKFmax = static_cast<KeyFrame *>(NULL);

        mvpLocalKeyFrames.clear();
        mvpLocalKeyFrames.reserve(3 * keyframeCounter.size());

        // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
        for (map<KeyFrame *, int>::const_iterator it = keyframeCounter.begin(), itEnd = keyframeCounter.end(); it != itEnd; it++)
        {
            KeyFrame *pKF = it->first;

            if (pKF->isBad())
                continue;

            if (it->second > max)
            {
                max = it->second;
                pKFmax = pKF;
            }

            mvpLocalKeyFrames.push_back(it->first);
            pKF->mnTrackReferenceForFrame = currentFrame.mnId;
        }

        // Include also some not-already-included keyframes that are neighbors to already-included keyframes
        for (vector<KeyFrame *>::const_iterator itKF = mvpLocalKeyFrames.begin(), itEndKF = mvpLocalKeyFrames.end(); itKF != itEndKF; itKF++)
        {
            // Limit the number of keyframes
            if (mvpLocalKeyFrames.size() > 80)
                break;

            KeyFrame *pKF = *itKF;

            const vector<KeyFrame *> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

            for (vector<KeyFrame *>::const_iterator itNeighKF = vNeighs.begin(), itEndNeighKF = vNeighs.end(); itNeighKF != itEndNeighKF; itNeighKF++)
            {
                KeyFrame *pNeighKF = *itNeighKF;
                if (!pNeighKF->isBad())
                {
                    if (pNeighKF->mnTrackReferenceForFrame != currentFrame.mnId)
                    {
                        mvpLocalKeyFrames.push_back(pNeighKF);
                        pNeighKF->mnTrackReferenceForFrame = currentFrame.mnId;
                        break;
                    }
                }
            }

            const set<KeyFrame *> spChilds = pKF->GetChildren();
            for (set<KeyFrame *>::const_iterator sit = spChilds.begin(), send = spChilds.end(); sit != send; sit++)
            {
                KeyFrame *pChildKF = *sit;
                if (!pChildKF->isBad())
                {
                    if (pChildKF->mnTrackReferenceForFrame != currentFrame.mnId)
                    {
                        mvpLocalKeyFrames.push_back(pChildKF);
                        pChildKF->mnTrackReferenceForFrame = currentFrame.mnId;
                        break;
                    }
                }
            }

            KeyFrame *pParent = pKF->GetParent();
            if (pParent)
            {
                if (pParent->mnTrackReferenceForFrame != currentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pParent);
                    pParent->mnTrackReferenceForFrame = currentFrame.mnId;
                    break;
                }
            }
        }

        if (pKFmax)
        {
            mvpReferenceKF[mCurrentCameraID] = pKFmax;
            currentFrame.mpReferenceKF = mvpReferenceKF[mCurrentCameraID];
        }
    }

    bool Tracking::Relocalization()
    {
        // Set set to current camera frame
        Frame &currentFrame = mvCurrentFrame[mCurrentCameraID];

        // Compute Bag of Words Vector
        currentFrame.ComputeBoW();

        // Relocalization is performed when tracking is lost
        // Track Lost: Query KeyFrame Database for keyframe candidates for relocalization
        vector<KeyFrame *> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&currentFrame);

        if (vpCandidateKFs.empty())
            return false;

        const int nKFs = vpCandidateKFs.size();

        // We perform first an ORB matching with each candidate
        // If enough matches are found we setup a PnP solver
        ORBmatcher matcher(0.75, true);

        vector<PnPsolver *> vpPnPsolvers;
        vpPnPsolvers.resize(nKFs);

        vector<vector<MapPoint *>> vvpMapPointMatches;
        vvpMapPointMatches.resize(nKFs);

        vector<bool> vbDiscarded;
        vbDiscarded.resize(nKFs);

        int nCandidates = 0;

        for (int i = 0; i < nKFs; i++)
        {
            KeyFrame *pKF = vpCandidateKFs[i];
            if (pKF->isBad())
                vbDiscarded[i] = true;
            else
            {
                int nmatches = matcher.SearchByBoW(pKF, (currentFrame), vvpMapPointMatches[i]);
                if (nmatches < 15)
                {
                    vbDiscarded[i] = true;
                    continue;
                }
                else
                {
                    PnPsolver *pSolver = new PnPsolver((currentFrame), vvpMapPointMatches[i]);
                    pSolver->SetRansacParameters(0.99, 10, 300, 4, 0.5, 5.991);
                    vpPnPsolvers[i] = pSolver;
                    nCandidates++;
                }
            }
        }

        // Alternatively perform some iterations of P4P RANSAC
        // Until we found a camera pose supported by enough inliers
        bool bMatch = false;
        ORBmatcher matcher2(0.9, true);

        while (nCandidates > 0 && !bMatch)
        {
            for (int i = 0; i < nKFs; i++)
            {
                if (vbDiscarded[i])
                    continue;

                // Perform 5 Ransac Iterations
                vector<bool> vbInliers;
                int nInliers;
                bool bNoMore;

                PnPsolver *pSolver = vpPnPsolvers[i];
                cv::Mat Tbw = pSolver->iterate(5, bNoMore, vbInliers, nInliers);

                // If Ransac reachs max. iterations discard keyframe
                if (bNoMore)
                {
                    vbDiscarded[i] = true;
                    nCandidates--;
                }

                // If a Camera Pose is computed, optimize
                if (!Tbw.empty())
                {
                    Tbw.copyTo(currentFrame.mTbw);

                    set<MapPoint *> sFound;

                    const int np = vbInliers.size();

                    for (int j = 0; j < np; j++)
                    {
                        if (vbInliers[j])
                        {
                            currentFrame.mvpMapPoints[j] = vvpMapPointMatches[i][j];
                            sFound.insert(vvpMapPointMatches[i][j]);
                        }
                        else
                            currentFrame.mvpMapPoints[j] = NULL;
                    }

                    int nGood = Optimizer::PoseOptimization(&currentFrame);

                    if (nGood < 10)
                        continue;

                    for (int io = 0; io < currentFrame.N; io++)
                        if (currentFrame.mvbOutlier[io])
                            currentFrame.mvpMapPoints[io] = static_cast<MapPoint *>(NULL);

                    // If few inliers, search by projection in a coarse window and optimize again
                    if (nGood < 50)
                    {
                        int nadditional = matcher2.SearchByProjection((currentFrame), vpCandidateKFs[i], sFound, 10, 100);

                        if (nadditional + nGood >= 50)
                        {
                            nGood = Optimizer::PoseOptimization(&currentFrame);

                            // If many inliers but still not enough, search by projection again in a narrower window
                            // the camera has been already optimized with many points
                            if (nGood > 30 && nGood < 50)
                            {
                                sFound.clear();
                                for (int ip = 0; ip < currentFrame.N; ip++)
                                    if (currentFrame.mvpMapPoints[ip])
                                        sFound.insert(currentFrame.mvpMapPoints[ip]);
                                nadditional = matcher2.SearchByProjection((currentFrame), vpCandidateKFs[i], sFound, 3, 64);

                                // Final optimization
                                if (nGood + nadditional >= 50)
                                {
                                    nGood = Optimizer::PoseOptimization(&currentFrame);

                                    for (int io = 0; io < currentFrame.N; io++)
                                        if (currentFrame.mvbOutlier[io])
                                            currentFrame.mvpMapPoints[io] = NULL;
                                }
                            }
                        }
                    }

                    // If the pose is supported by enough inliers stop ransacs and continue
                    if (nGood >= 50)
                    {
                        bMatch = true;
                        break;
                    }
                }
            }
        }

        if (!bMatch)
        {
            return false;
        }
        else
        {
            mnLastRelocFrameId = currentFrame.mnId;
            return true;
        }
    }

    void Tracking::Reset()
    {

        const bool partialReset = true;

        if (partialReset)
        {
            cout << "Tracking:̣\tCamera " << mCurrentCameraID << " reseted!" << endl;
            mvpInitializer[mCurrentCameraID] = new Initializer(mvCurrentFrame[mCurrentCameraID], mpLastKeyFrame, mOdometryLastKF, 3.0, 200);
            mState[mCurrentCameraID] = NOT_INITIALIZED;

            if (mSystemState == INITIALIZED)
                mbWasCameraReseted[mCurrentCameraID] = true;
        }
        else
        {
            std::cout << "Complete system Reset..." << endl;

            if (mpViewer)
            {
                mpViewer->RequestStop();
                while (!mpViewer->isStopped())
                    usleep(3000);
            }

            // Reset Local Mapping
            mpLocalMapper->RequestReset();

            // Clear BoW Database
            mpKeyFrameDB->clear();

            // Clear Map (this erase MapPoints and KeyFrames)
            mpMap->clear();

            KeyFrame::nNextId = 0;
            Frame::nNextId = 0;

            // Resetting status of all cameras
            for (int i = 0; i < mnCameras; i++)
            {
                mState[i] = NO_IMAGES_YET;
                if (mvpInitializer[i])
                {
                    delete mvpInitializer[i];
                    mvpInitializer[i] = static_cast<Initializer *>(NULL);
                }
            }

            mlRelativeFramePoses.clear();
            mlpReferences.clear();
            mlFrameTimes.clear();
            mlbLost.clear();

            if (mpViewer)
                mpViewer->Release();

            mSystemState = NON_INITIALIZED;
        }
    }

    void Tracking::ChangeCalibration(const string &strSettingPath)
    {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        // Load intrinsic and extrinsics for each camera
        std::string s("Camera");
        for (int i = 0; i < msCameraCalibration.size(); i++)
        {
            std::string sCurrentCamera = "Camera" + std::to_string(i);

            float fx = fSettings[sCurrentCamera + ".fx"];
            float fy = fSettings[sCurrentCamera + ".fy"];
            float cx = fSettings[sCurrentCamera + ".cx"];
            float cy = fSettings[sCurrentCamera + ".cy"];

            cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
            K.at<float>(0, 0) = fx;
            K.at<float>(1, 1) = fy;
            K.at<float>(0, 2) = cx;
            K.at<float>(1, 2) = cy;
            K.copyTo(msCameraCalibration[i].K);

            cv::Mat DistCoef(4, 1, CV_32F);
            DistCoef.at<float>(0) = fSettings[sCurrentCamera + ".k1"];
            DistCoef.at<float>(1) = fSettings[sCurrentCamera + ".k2"];
            DistCoef.at<float>(2) = fSettings[sCurrentCamera + ".p1"];
            DistCoef.at<float>(3) = fSettings[sCurrentCamera + ".p2"];
            const float k3 = fSettings[sCurrentCamera + ".k3"];
            if (k3 != 0)
            {
                DistCoef.resize(5);
                DistCoef.at<float>(4) = k3;
            }
            DistCoef.copyTo(msCameraCalibration[i].DistCoef);

            msCameraCalibration[i].fBaseLine = fSettings[sCurrentCamera + ".bf"];

            std::cout << endl
                      << "Camera Parameters: Camera " << i << endl;
            std::cout << "- fx: " << fx << endl;
            std::cout << "- fy: " << fy << endl;
            std::cout << "- cx: " << cx << endl;
            std::cout << "- cy: " << cy << endl;
            std::cout << "- k1: " << DistCoef.at<float>(0) << endl;
            std::cout << "- k2: " << DistCoef.at<float>(1) << endl;
            if (DistCoef.rows == 5)
                std::cout << "- k3: " << DistCoef.at<float>(4) << endl;
            std::cout << "- p1: " << DistCoef.at<float>(2) << endl;
            std::cout << "- p2: " << DistCoef.at<float>(3) << endl;
        }
    }

    cv::Mat Tracking::GetExtrinsics(int cameraID)
    {
        return msCameraCalibration[cameraID].Tcb.clone();
    }

    void Tracking::UpdateExtrinsics(int cameraID, cv::Mat calib)
    {

        // Calculate calibrated camera extrinsics
        cv::Mat updatedTcb = calib * msCameraCalibration[cameraID].Tcb;

        // Update camera calibration structure for new frames
        msCameraCalibration[cameraID].Tcb = updatedTcb;

        // Update all key frames
        vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
        for (size_t i = 0, iend = vpKFs.size(); i < iend; i++)
        {
            KeyFrame *pKFi = vpKFs[i];

            if (pKFi->mCameraID == cameraID)
                pKFi->mTcb = updatedTcb;
        }

        // Update current and last tracked frames
        mvPreviousFrames[cameraID].mTcb = updatedTcb;
        mvCurrentFrame[cameraID].mTcb = updatedTcb;
    }

    double Tracking::GetDistanceLastFixedKF()
    {
        double d = 0;
        for (auto it = mOdoLastFixedKF.begin(); it != mOdoLastFixedKF.end(); ++it)
        {
            double tmp = cv::norm((*it).GetTransformation().col(3).rowRange(0, 3));
            if (d < tmp)
                d = tmp;
        }

        return d;
    }
    double Tracking::GetDistanceLastFixedKF(int cameraID)
    {
        if (mbFixedMapInitialized)
        {
            double d = cv::norm(mOdoLastFixedKF[cameraID].GetTransformation().col(3).rowRange(0, 3));

            return d;
        }
        else
        {
            return DBL_MAX;
        }
    }

    bool Tracking::AnyCameraTracking()
    {
        bool anyOK = false;
        for (int iCamera = 0; iCamera < mnCameras; iCamera++)
        {
            anyOK |= (mState[iCamera] == OK);
        }
        return anyOK;
    }

    bool Tracking::AnyCameraReseted()
    {
        bool anyReseted = false;
        for (int iCamera = 0; iCamera < mnCameras; iCamera++)
        {
            anyReseted |= (mbWasCameraReseted[iCamera]);
        }
        return anyReseted;
    }

    bool Tracking::AnyCameraInitializing()
    {
        bool anyInit = false;
        for (int iCamera = 0; iCamera < mnCameras; iCamera++)
        {
            anyInit |= (mState[iCamera] == NOT_INITIALIZED);
        }
        return anyInit;
    }

} // namespace LocSLAM
