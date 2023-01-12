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

#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>

namespace LocSLAM
{

    System::~System()
    {
        delete mpVocabulary;
        delete mpKeyFrameDatabase;
        delete mpMap;
        delete mpFixedMap;
        for (int i = 0; i < mvpFrameDrawer.size(); i++)
        {
            if (mvpFrameDrawer[i])
            {
                delete mvpFrameDrawer[i];
                mvpFrameDrawer[i] = NULL;
            }
        }
        if (mpMapDrawer)
        {
            delete mpMapDrawer;
            mpMapDrawer = NULL;
        }
        delete mpTracker;
        delete mpLocalMapper;
        delete mpCameraCalibrator;
        if (mpViewer)
        {
            mpViewer->RequestFinish();
            mpViewer->RequestStop();
        }
        if (mptViewer)
        {
            mptViewer->join();
        }
        if (mpViewer)
        {
            delete mpViewer;
            mpViewer = NULL;
        }
        if (mptViewer)
        {
            delete mptViewer;
            mptViewer = NULL;
        }
    }

    System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
                   const bool bUseViewer, const bool bUseOdometry, const bool bUseGlobalMap) : mSensor(sensor), mbUseOdometry(bUseOdometry), mbUseGlobalMap(bUseGlobalMap),
                                                                                               mpViewer(static_cast<Viewer *>(NULL)), mbReset(false), mbActivateLocalizationMode(false),
                                                                                               mbDeactivateLocalizationMode(false)
    {
        // Output welcome message
        cout << endl
             << "LocSLAM Copyright (C) 2020 Lars Hammarstrand, Chalmers University of Technology." << endl
             << "This program comes with ABSOLUTELY NO WARRANTY;" << endl
             << "This is free software, and you are welcome to redistribute it" << endl
             << "under certain conditions. See LICENSE.txt." << endl
             << endl;

        cout << "Input sensor was set to: ";

        if (mSensor == MONOCULAR)
            cout << "Monocular" << endl;
        else if (mSensor == STEREO)
        {
            cout << "Stereo: NOT CURRENTLY SUPPORTED! Terminating!" << endl;
            return;
        }
        else if (mSensor == RGBD)
        {
            cout << "RGB-D: NOT CURRENTLY SUPPORTED! Terminating!" << endl;
            return;
        }

        //Check settings file
        cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
        if (!fsSettings.isOpened())
        {
            cerr << "Failed to open settings file at: " << strSettingsFile << endl;
            exit(-1);
        }

        //Load ORB Vocabulary
        cout << endl
             << "Loading ORB Vocabulary. This could take a while..." << endl;

        mpVocabulary = new ORB_SLAM2::ORBVocabulary();
        bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
        if (!bVocLoad)
        {
            cerr << "Wrong path to vocabulary. " << endl;
            cerr << "Falied to open at: " << strVocFile << endl;
            exit(-1);
        }
        cout << "Vocabulary loaded!" << endl
             << endl;

        //Create KeyFrame Database
        mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

        //Create tracking Map
        mpMap = new Map(false);

        //Create fixed map
        mpFixedMap = new Map(true);

        //Create Drawers. These are used by the Viewer

        // Load number of cameras in rig
        int nCameras = fsSettings["nCameras"];
        mvpFrameDrawer.resize(nCameras);

        for (int i = 0; i < nCameras; i++)
            mvpFrameDrawer[i] = new FrameDrawer(mpMap);

        mpMapDrawer = new MapDrawer(mpMap, mpFixedMap, strSettingsFile);

        //Initialize the Tracking thread
        //(it will live in the main thread of execution, the one that called this constructor)
        mpTracker = new Tracking(this, mpVocabulary, mvpFrameDrawer, mpMapDrawer,
                                 mpMap, mpFixedMap, mpKeyFrameDatabase, strSettingsFile, mSensor, mbUseOdometry);

        mpCameraCalibrator = new CameraCalibration(mpTracker);

        //Initialize the Local Mapping thread and launch
        mpLocalMapper = new LocalMapping(mpMap, mpFixedMap, mpCameraCalibrator, mSensor == MONOCULAR, mbUseGlobalMap);

        // LocSLAM is currently not real-time. LocalMapping is run in tracking thread.
        //mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run, mpLocalMapper);

        //Initialize the Viewer thread and launch
        if (bUseViewer)
        {
            mpViewer = new Viewer(this, mvpFrameDrawer, mpMapDrawer, mpTracker, strSettingsFile);
            mptViewer = new thread(&Viewer::Run, mpViewer);
            mpTracker->SetViewer(mpViewer);
        }

        //Set pointers between threads
        mpTracker->SetLocalMapper(mpLocalMapper);
        mpLocalMapper->SetTracker(mpTracker);
    }

    cv::Mat System::TrackMonocularOdo(const cv::Mat &im, const OdometryData &odometry,
                                      const FixedMapMatches &frameFixedMapMatches, const double &timestamp,
                                      const int &cameraID, string strImgFileName)
    {
        if (mSensor != MONOCULAR)
        {
            cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
            exit(-1);
        }

        // Check reset
        {
            unique_lock<recursive_mutex> lock(mMutexReset);
            if (mbReset)
            {
                mpTracker->Reset();
                mbReset = false;
            }
        }

        // Track new image
        cv::Mat Tbw = mpTracker->GrabImageMonocularOdo(im, odometry, frameFixedMapMatches, timestamp, cameraID, strImgFileName);

        unique_lock<recursive_mutex> lock2(mMutexState);
        mTrackingState = mpTracker->mState[cameraID];
        mTrackedMapPoints = mpTracker->mvCurrentFrame[cameraID].mvpMapPoints;
        mTrackedKeyPointsUn = mpTracker->mvCurrentFrame[cameraID].mvKeysUn;

        // Run local bundle if new tracker has added key frames
        mpLocalMapper->UpdateLocalMap();

        mpMapDrawer->SetCurrentCameraPose(mpTracker->mvCurrentFrame[cameraID].mTcb * mpTracker->mvCurrentFrame[cameraID].mTbw);
        mpMapDrawer->SetViewCameraPose(mpTracker->mvCurrentFrame[cameraID].mTbw);
        mpMapDrawer->mbUpdateCurrentCamera = true;

        mpTracker->StoreCurrentPose();

        return Tbw;
    }

    bool System::MapChanged()
    {
        static int n = 0;
        int curn = mpMap->GetLastBigChangeIdx();
        if (n < curn)
        {
            n = curn;
            return true;
        }
        else
            return false;
    }

    void System::Reset()
    {
        unique_lock<recursive_mutex> lock(mMutexReset);
        mbReset = true;
    }

    void System::Shutdown()
    {
        mpLocalMapper->RequestFinish();
        if (mpViewer)
        {
            mpViewer->RequestFinish();
            while (!mpViewer->isFinished())
                usleep(5000);
        }

        // Wait until all thread have effectively stopped
        while (!mpLocalMapper->isFinished())
        {
            usleep(5000);
        }

        if (mpViewer)
            pangolin::BindToContext("ORB-SLAM2: Map Viewer");
    }

    void System::SaveKeyFrameTrajectoryCMU(const string &filename, const bool benchmarkFormat)
    {
        cout << endl
             << "Saving keyframe trajectory to " << filename << " ..." << endl;

        vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lTime);

        // Transform all keyframes so that the KeyFrames are in fixed world coordinates.
        cv::Mat Tfl = mpLocalMapper->mpFixedMapLocalizer->GetTransformation().inv();

        cv::Mat Rfl = Tfl.colRange(0, 3).rowRange(0, 3);
        Eigen::Quaterniond q_fl(Converter::toMatrix3d(Rfl));

        cv::Mat t_fl = Tfl.rowRange(0, 3).col(3);

        ofstream f_fixedMapTransform;
        string fixedMapFileName = "fixedMapTransform_" + filename;
        f_fixedMapTransform.open(fixedMapFileName.c_str());
        f_fixedMapTransform << fixed;

        f_fixedMapTransform << setprecision(7) << q_fl.w() << " " << q_fl.x() << " " << q_fl.y() << " " << q_fl.z() << " " << t_fl.at<float>(0) << " " << t_fl.at<float>(1) << " " << t_fl.at<float>(2)
                            << endl;

        f_fixedMapTransform.close();

        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame *pKF = vpKFs[i];

            if (pKF->isBad())
                continue;

            cv::Mat Trw = pKF->GetPose();

            cv::Mat Tprint;
            if (benchmarkFormat)
            {
                Tprint = pKF->mTcb * Trw * Tfl.inv(); //Tcf
            }
            else
            {
                Tprint = Tfl * Trw.inv(); //Tfb
            }
            cv::Mat Rprint = Tprint.colRange(0, 3).rowRange(0, 3);
            Eigen::Quaterniond q_print(Converter::toMatrix3d(Rprint));

            cv::Mat t_print = Tprint.rowRange(0, 3).col(3);

            string fileName = pKF->mstrImgFileName;
            size_t fn_pos = 0;
            size_t left_pos = fileName.rfind("left/");
            size_t rear_pos = fileName.rfind("rear/");
            size_t right_pos = fileName.rfind("right/");
            size_t cam_pos = min({left_pos, rear_pos, right_pos});
            size_t slash_pos = fileName.rfind("/");
            if (cam_pos < slash_pos)
            {
                fn_pos = cam_pos;
            }
            else if (slash_pos != std::string::npos)
            {
                fn_pos = slash_pos + 1;
            }
            string fn_print = fileName.substr(fn_pos);

            f << fn_print << setprecision(7) << " "
              << q_print.w() << " " << q_print.x() << " " << q_print.y() << " " << q_print.z() << " "
              << t_print.at<float>(0) << " " << t_print.at<float>(1) << " " << t_print.at<float>(2);

            f << endl;
        }

        f.close();
        cout << endl
             << "trajectory saved!" << endl;
    }

    void System::SaveTrajectoryCMU(const string &filename, const bool benchmarkFormat)
    {
        cout << endl
             << "Saving keyframe trajectory to " << filename << " ..." << endl;

        vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lTime);

        // Transform all keyframes so that the KeyFrames are in fixed world coordinates.
        cv::Mat Tfl = mpLocalMapper->mpFixedMapLocalizer->GetTransformation().inv();

        cv::Mat Rfl = Tfl.colRange(0, 3).rowRange(0, 3);
        Eigen::Quaterniond q_fl(Converter::toMatrix3d(Rfl));

        cv::Mat t_fl = Tfl.rowRange(0, 3).col(3);

        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
        // We need to get first the keyframe pose and then concatenate the relative transformation.
        // Frames not localized (tracking failure) are not saved.

        // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
        // which is true when tracking failed (lbL).
        list<KeyFrame *>::iterator lRit = mpTracker->mlpReferences.begin();
        list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
        list<string>::iterator lFileName = mpTracker->mlstrFileNames.begin();
        list<bool>::iterator lLost = mpTracker->mlbLost.begin();
        list<int>::iterator lCameraID = mpTracker->mlCameraID.begin();
        for (list<cv::Mat>::iterator lit = mpTracker->mlRelativeFramePoses.begin(), lend = mpTracker->mlRelativeFramePoses.end(); lit != lend; lit++, lRit++, lT++, lFileName++, lLost++, lCameraID++)
        {
            KeyFrame *pKF = *lRit;

            cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

            while (pKF->isBad())
            {
                //  cout << "bad parent" << endl;
                Trw = Trw * pKF->mTbp;
                pKF = pKF->GetParent();
            }

            Trw = Trw * pKF->GetPose();

            // Interpolate frames between 2 Keyframes
            g2o::SE3Quat Tbr;
            cv::Mat Tbw;

            // find next kf
            KeyFrame *pKFi = pKF->GetNextKeyFrame();
            while (pKFi && pKFi->GetNextKeyFrame() && pKFi->mTimeStamp < (*lT))
                pKFi = pKFi->GetNextKeyFrame();

            if (pKFi)
            {
                // Calculate relative transform
                g2o::SE3Quat T21 = Converter::toSE3Quat(pKFi->GetPose() * pKF->GetPoseInverse());

                double w;
                if (pKFi->mTimeStamp - pKF->mTimeStamp > 1e-5)
                    w = ((*lT) - pKF->mTimeStamp) / (pKFi->mTimeStamp - pKF->mTimeStamp);
                else
                    w = 0;

                if (w > 1)
                    w = 1;

                Eigen::Quaterniond qa = Eigen::Quaterniond::Identity();

                Tbr.setTranslation(w * T21.translation());

                Eigen::Quaterniond qb = qa.slerp(w, T21.rotation());
                Tbr.setRotation(qb);

                //cv::Mat Tbw = (*lit) * Trw;
                Tbw = Converter::toCvMat(Tbr) * Trw;
            }
            else
            {
                Tbw = (*lit) * Trw;
            }

            cv::Mat Tprint;
            if (benchmarkFormat)
            {
                cv::Mat Tcb = mpTracker->GetExtrinsics((*lCameraID));
                Tprint = Tcb * Tbw * Tfl.inv(); //Tcf
            }
            else
            {
                Tprint = Tfl * Tbw.inv(); //Tfb
            }

            size_t fn_pos = 0;
            size_t left_pos = (*lFileName).rfind("left/");
            size_t rear_pos = (*lFileName).rfind("rear/");
            size_t right_pos = (*lFileName).rfind("right/");
            size_t cam_pos = min({left_pos, rear_pos, right_pos});
            size_t slash_pos = (*lFileName).rfind("/");
            if (cam_pos < slash_pos)
            {
                fn_pos = cam_pos;
            }
            else if (slash_pos != std::string::npos)
            {
                fn_pos = slash_pos + 1;
            }
            string fn_print = (*lFileName).substr(fn_pos);

            cv::Mat Rprint = Tprint.colRange(0, 3).rowRange(0, 3);
            Eigen::Quaterniond q_print(Converter::toMatrix3d(Rprint));

            cv::Mat t_print = Tprint.rowRange(0, 3).col(3);

            f << fn_print << setprecision(7) << " "
              << q_print.w() << " " << q_print.x() << " " << q_print.y() << " " << q_print.z() << " "
              << t_print.at<float>(0) << " " << t_print.at<float>(1) << " " << t_print.at<float>(2);
            if (!benchmarkFormat)
            {
                f << " " << (*lLost);
            }
            f << endl;
        }

        f.close();
        cout << endl
             << "trajectory saved!" << endl;
    }

    void System::SaveTrackingTrajectoryCMU(const string &filename, const bool benchmarkFormat)
    {
        cout << endl
             << "Saving tracking trajectory to " << filename << " ..." << endl;

        // Transform all keyframes so that the KeyFrames are in fixed world coordinates.
        cv::Mat Tfl = mpLocalMapper->mpFixedMapLocalizer->GetTransformation().inv();

        cv::Mat Rfl = Tfl.colRange(0, 3).rowRange(0, 3);
        Eigen::Quaterniond q_fl(Converter::toMatrix3d(Rfl));

        cv::Mat t_fl = Tfl.rowRange(0, 3).col(3);

        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
        // We need to get first the keyframe pose and then concatenate the relative transformation.
        // Frames not localized (tracking failure) are not saved.

        // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
        // which is true when tracking failed (lbL).
        list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
        list<string>::iterator lFileName = mpTracker->mlstrFileNames.begin();
        list<bool>::iterator lLost = mpTracker->mlbLost.begin();
        list<int>::iterator lCameraID = mpTracker->mlCameraID.begin();
        for (list<cv::Mat>::iterator lit = mpTracker->mlTrackingPose.begin(), lend = mpTracker->mlTrackingPose.end(); lit != lend; lit++, lT++, lFileName++, lLost++, lCameraID++)
        {

            cv::Mat Tbw = (*lit);

            cv::Mat Tprint;
            size_t fn_pos = 0;
            if (benchmarkFormat)
            {
                cv::Mat Tcb = mpTracker->GetExtrinsics((*lCameraID));
                Tprint = Tcb * Tbw * Tfl.inv(); //Tcf
            }
            else
            {
                Tprint = Tfl * Tbw.inv(); //Tfb
            }

            size_t left_pos = (*lFileName).rfind("left/");
            size_t rear_pos = (*lFileName).rfind("rear/");
            size_t right_pos = (*lFileName).rfind("right/");
            size_t cam_pos = min({left_pos, rear_pos, right_pos});
            size_t slash_pos = (*lFileName).rfind("/");
            if (cam_pos < slash_pos)
            {
                fn_pos = cam_pos;
            }
            else if (slash_pos != std::string::npos)
            {
                fn_pos = slash_pos + 1;
            }
            string fn_print = (*lFileName).substr(fn_pos);

            cv::Mat Rprint = Tprint.colRange(0, 3).rowRange(0, 3);
            Eigen::Quaterniond q_print(Converter::toMatrix3d(Rprint));

            cv::Mat t_print = Tprint.rowRange(0, 3).col(3);

            f << fn_print << setprecision(7) << " "
              << q_print.w() << " " << q_print.x() << " " << q_print.y() << " " << q_print.z() << " "
              << t_print.at<float>(0) << " " << t_print.at<float>(1) << " " << t_print.at<float>(2);
            if (!benchmarkFormat)
            {
                f << " " << (*lLost);
            }
            f << endl;
        }

        f.close();
        cout << endl
             << "trajectory saved!" << endl;
    }

    int System::GetTrackingState()
    {
        unique_lock<recursive_mutex> lock(mMutexState);
        return mTrackingState;
    }

    vector<MapPoint *> System::GetTrackedMapPoints()
    {
        unique_lock<recursive_mutex> lock(mMutexState);
        return mTrackedMapPoints;
    }

    vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
    {
        unique_lock<recursive_mutex> lock(mMutexState);
        return mTrackedKeyPointsUn;
    }

} // namespace LocSLAM
