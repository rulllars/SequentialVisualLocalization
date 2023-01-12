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

#include "Viewer.h"
#include <pangolin/pangolin.h>

#include <mutex>

namespace LocSLAM
{

    Viewer::Viewer(System *pSystem, vector<FrameDrawer *> vpFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, const string &strSettingPath)
        : mpSystem(pSystem), mvpFrameDrawer(vpFrameDrawer), mpMapDrawer(pMapDrawer), mpTracker(pTracking),
          mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false), mpCurrentKeyFrame(static_cast<KeyFrame *>(NULL))
    {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        mnCameras = fSettings["nCameras"];

        cv::Mat mat = cv::Mat::eye(mnCameras, 1, CV_32SC1);
        fSettings["supportedCameras"] >> mat;
        mvSupportedCameras = mat;

        float fps = fSettings["Camera.fps"];
        if (fps < 1)
            fps = 30;
        mT = 1e3 / fps;

        mImageWidth = fSettings["Camera.width"];
        mImageHeight = fSettings["Camera.height"];
        if (mImageWidth < 1 || mImageHeight < 1)
        {
            mImageWidth = 640;
            mImageHeight = 480;
        }

        mViewpointX = fSettings["Viewer.ViewpointX"];
        mViewpointY = fSettings["Viewer.ViewpointY"];
        mViewpointZ = fSettings["Viewer.ViewpointZ"];
        mViewpointF = fSettings["Viewer.ViewpointF"];

        mImageScale = fSettings["Viewer.ImageScale"];

        mpCurrentKeyFrame = NULL;

        mbRecordVideo = ((int)fSettings["Viewer.RecordVideo"] == 1);

        string sCamVideoSuffix = fSettings["Viewer.RecordVideoCameraSuffix"];
        string sMapVideoSuffix = fSettings["Viewer.RecordVideoMapSuffix"];
        msCamVideoFileName = strSettingPath + sCamVideoSuffix;
        msMapVideoFileName = strSettingPath + sMapVideoSuffix;

        if (mbRecordVideo == 1)
        {
            mVideoRec = cv::VideoWriter(msCamVideoFileName, CV_FOURCC('H', '2', '6', '4'), 10, cv::Size(1434, 552), true);
        }
    }

    Viewer::~Viewer()
    {
        // release video
        mVideoRec.release();
    }

    void Viewer::Run()
    {
        mbFinished = false;
        mbStopped = false;

        int UI_WIDTH = 175;
        int w = 1920;
        int h = 1080;
        pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer", w + UI_WIDTH, h);

        // 3D Mouse handler requires depth testing to be enabled
        glEnable(GL_DEPTH_TEST);

        // Issue specific OpenGl we might need
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175));
        pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", true, true);
        pangolin::Var<string> menuMap("menu.Map Settings:");
        pangolin::Var<bool> menuShowPoints("menu.All Local Points", true, true);
        pangolin::Var<bool> menuShowFixedPoints("menu.All Fixed Points", false, true);
        pangolin::Var<string> menuKeyFrames("menu.KeyFrames: ");
        pangolin::Var<bool> menuShowGraph("menu.Show Graph", true, true);
        pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames", true, true);
        pangolin::Var<bool> menuShowPose("menu.Show Pose", true, true);
        pangolin::Var<bool> menuShowFixPriors("menu.Show Fix Priors", true, true);
        pangolin::Var<bool> menuShowOdo("menu.Show Odometry", false, true);
        pangolin::Var<bool> menuReset("menu.Reset GUI", false, false);

        // Define Camera Render Object (for view / scene browsing)
        pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(w, h, mViewpointF, mViewpointF, w / 2, h / 2, 0.1, 2000),
            pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, 0.0, 1.0));

        // Add named OpenGL viewport to window and provide 3D Handler
        pangolin::View &d_cam = pangolin::CreateDisplay()
                                    .SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0, (float)w / (float)h)
                                    .SetHandler(new pangolin::Handler3D(s_cam));

        // Record map video
        if (mbRecordVideo)
            d_cam.RecordOnRender("ffmpeg:[fps=10,flip=true,c:v=libx264,profile=slow,crf=20,pix_fmt=yuv420p]//" + msMapVideoFileName);

        pangolin::OpenGlMatrix Twc;
        Twc.SetIdentity();

        pangolin::OpenGlMatrix Twb;
        Twb.SetIdentity();

        pangolin::OpenGlMatrix predictedTwc;
        predictedTwc.SetIdentity();

        cv::namedWindow("ORB-SLAM2: Current Frame(s)");

        bool bFollow = true;

        while (1)
        {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            if (mpMapDrawer->mbUpdateCurrentCamera)
            {
                mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);
                mpMapDrawer->GetCurrentOpenGLBodyMatrix(Twb);
                mpMapDrawer->GetCurrentOpenGLPredictedCameraMatrix(predictedTwc);

                mpMapDrawer->mbUpdateCurrentCamera = false;
            }
            if (menuFollowCamera && bFollow)
            {
                s_cam.Follow(Twb);
            }
            else if (menuFollowCamera && !bFollow)
            {
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, 0.0, 1.0));
                s_cam.Follow(Twb);
                bFollow = true;
            }
            else if (!menuFollowCamera && bFollow)
            {
                bFollow = false;
            }

            d_cam.Activate(s_cam);
            glClearColor(.2f, .2f, .2f, 1.0f);
            cv::Mat Tbw;
            mpMapDrawer->GetCurrentBodyMatrix(Tbw);
            if (!Tbw.empty())
                for (int i = 0; i < mnCameras; i++)
                {
                    cv::Mat Tcw = mpTracker->GetExtrinsics(i) * Tbw;
                    pangolin::OpenGlMatrix _Twc = mpMapDrawer->CVMat2OpenGL(Tcw);
                    mpMapDrawer->DrawCurrentCamera(_Twc);
                }

            mpMapDrawer->UpdateKeyFrames();

            if (menuShowKeyFrames || menuShowGraph || menuShowPose || menuShowOdo)
                mpMapDrawer->DrawKeyFrames(menuShowKeyFrames, menuShowFixPriors, menuShowGraph, menuShowPose, menuShowOdo);

            if (menuShowPoints)
                mpMapDrawer->DrawMapPoints();
            else
                mpMapDrawer->DrawLocalMapPoints();

            if (menuShowFixedPoints)
                mpMapDrawer->DrawFixedMapPoints();
            else
                mpMapDrawer->DrawLocalFixedMapPoints();

            pangolin::FinishFrame();

            cv::Mat concatIm;
            bool firstImage = true;
            for (int i = 0; i < mnCameras; i++)
            {
                if (mvSupportedCameras[i])
                {
                    cv::Mat im = mvpFrameDrawer[i]->DrawFrame();

                    // Scaled image for viewing
                    cv::Mat scaledIm;

                    cv::resize(im, scaledIm, cv::Size(), mImageScale, mImageScale);

                    if (firstImage)
                    {
                        concatIm = scaledIm;
                        firstImage = false;
                    }
                    else if (scaledIm.rows == concatIm.rows)
                    {
                        cv::hconcat(concatIm, scaledIm, concatIm);
                    }
                }
            }
            cv::imshow("ORB-SLAM2: Current Frame(s)", concatIm);
            cv::waitKey(mT);

            if (mbRecordVideo)
                mVideoRec.write(concatIm);

            if (menuReset)
            {
                menuShowGraph = true;
                menuShowPose = true;
                menuShowKeyFrames = true;
                menuShowPoints = true;
                bFollow = true;
                menuFollowCamera = true;
                mpSystem->Reset();
                menuReset = false;
            }

            if (Stop())
            {
                while (isStopped())
                {
                    usleep(3000);
                }
            }

            if (CheckFinish())
                break;
        }

        SetFinish();
    }

    void Viewer::RequestFinish()
    {
        unique_lock<recursive_mutex> lock(mMutexFinish);
        mbFinishRequested = true;
    }

    bool Viewer::CheckFinish()
    {
        unique_lock<recursive_mutex> lock(mMutexFinish);
        return mbFinishRequested;
    }

    void Viewer::SetFinish()
    {
        unique_lock<recursive_mutex> lock(mMutexFinish);
        mbFinished = true;
    }

    bool Viewer::isFinished()
    {
        unique_lock<recursive_mutex> lock(mMutexFinish);
        return mbFinished;
    }

    void Viewer::RequestStop()
    {
        unique_lock<recursive_mutex> lock(mMutexStop);
        if (!mbStopped)
            mbStopRequested = true;
    }

    bool Viewer::isStopped()
    {
        unique_lock<recursive_mutex> lock(mMutexStop);
        return mbStopped;
    }

    bool Viewer::Stop()
    {
        unique_lock<recursive_mutex> lock(mMutexStop);
        unique_lock<recursive_mutex> lock2(mMutexFinish);

        if (mbFinishRequested)
            return false;
        else if (mbStopRequested)
        {
            mbStopped = true;
            mbStopRequested = false;
            return true;
        }

        return false;
    }

    void Viewer::Release()
    {
        unique_lock<recursive_mutex> lock(mMutexStop);
        mbStopped = false;
    }

} // namespace LocSLAM
