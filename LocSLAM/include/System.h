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

#ifndef SYSTEM_H
#define SYSTEM_H

#include <string>
#include <thread>
#include <opencv2/core/core.hpp>
#include <unistd.h>

#include "Tracking.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Map.h"
#include "LocalMapping.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "Viewer.h"
#include "SLAMDataTypes.h"
#include "CameraCalibration.h"

#include "OdometryData.h"

namespace LocSLAM
{

    class Viewer;
    class FrameDrawer;
    class Map;
    class Tracking;
    class LocalMapping;
    class MapDrawer;
    class CameraCalibration;

    class System
    {
    public:
        // Input sensor
        enum eSensor
        {
            MONOCULAR = 0,
            STEREO = 1,
            RGBD = 2
        };

    public:
        // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
        System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer = true, const bool bUseOdometry = false, const bool bUseGlobalMap = false);

        ~System();

        // Proccess the given monocular frame with odometry
        // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
        // Returns the camera/body pose (empty if tracking fails).
        cv::Mat TrackMonocularOdo(const cv::Mat &im, const OdometryData &odometry,
                                  const FixedMapMatches &frameFixeMapMatches,
                                  const double &timestamp, const int &cameraID = 0, string strImgFileName = string());

        // Returns true if there have been a big map change (loop closure, global BA)
        // since last call to this function
        bool MapChanged();

        // Reset the system (clear map)
        void Reset();

        // All threads will be requested to finish.
        // It waits until all threads have finished.
        // This function must be called before saving the trajectory.
        void Shutdown();

        // Save camera trajectory in the CMU dataset format.
        // Only for monocular.
        // Call first Shutdown()
        // See format details at: http://www.visuallocalization.net
        void SaveTrajectoryCMU(const string &filename, const bool benchmarkFormat = false);
        void SaveTrackingTrajectoryCMU(const string &filename, const bool benchmarkFormat = false);

        void SaveKeyFrameTrajectoryCMU(const string &filename, const bool benchmarkFormat = false);

        // TODO: Save/Load functions
        // SaveMap(const string &filename);
        // LoadMap(const string &filename);

        // Information from most recent processed frame
        // You can call this right after TrackMonocular (or stereo or RGBD)
        int GetTrackingState();
        std::vector<MapPoint *> GetTrackedMapPoints();
        std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

        // Use global map
        bool mbUseGlobalMap;

    private:
        // Input sensor
        eSensor mSensor;

        int mnCameras;
        vector<int> mvSupportedCameras;

        // Use odometry
        bool mbUseOdometry;

        // ORB vocabulary used for place recognition and feature matching.
        ORB_SLAM2::ORBVocabulary *mpVocabulary = NULL;

        // KeyFrame database for place recognition (relocalization and loop detection).
        KeyFrameDatabase *mpKeyFrameDatabase = NULL;

        // Map structure that stores the pointers to all KeyFrames and MapPoints.
        Map *mpMap = NULL;
        Map *mpFixedMap = NULL;

        // Camera calibration. Stores information related to online calibration of camera extrinsic parameters
        CameraCalibration *mpCameraCalibrator;

        // Tracker. It receives a frame and computes the associated camera pose.
        // It also decides when to insert a new keyframe, create some new MapPoints and
        // performs relocalization if tracking fails.
        Tracking *mpTracker = NULL;

        // Local Mapper. It manages the local map and performs local bundle adjustment.
        LocalMapping *mpLocalMapper = NULL;

        // The viewer draws the map and the current camera pose. It uses Pangolin.
        Viewer *mpViewer = NULL;

        vector<FrameDrawer *> mvpFrameDrawer;
        MapDrawer *mpMapDrawer = NULL;

        // System threads: Local Mapping, Viewer. TODO: Make fix map localizer in separate thread.
        // The Tracking thread "lives" in the main execution thread that creates the System object.
        std::thread *mptLocalMapping = NULL;
        std::thread *mptViewer = NULL;

        // Reset flag
        std::recursive_mutex mMutexReset;
        bool mbReset;

        // Change mode flags
        std::recursive_mutex mMutexMode;
        bool mbActivateLocalizationMode;
        bool mbDeactivateLocalizationMode;

        // Tracking state
        int mTrackingState;
        std::vector<MapPoint *> mTrackedMapPoints;
        std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
        std::recursive_mutex mMutexState;
    };

} // namespace LocSLAM

#endif // SYSTEM_H
