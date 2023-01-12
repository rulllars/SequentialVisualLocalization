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

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "KeyFrame.h"
#include "Frame.h"
#include "Map.h"

#include <opencv2/core/core.hpp>
#include <mutex>

namespace LocSLAM
{

    class KeyFrame;
    class Map;
    class Frame;

    class MapPoint
    {
    public:
        MapPoint(const cv::Mat &Pos, KeyFrame *pRefKF, Map *pMap);
        MapPoint(const cv::Mat &Pos, Map *pMap, Frame *pFrame, const int &idxF);

        void SetWorldPos(const cv::Mat &Pos);
        cv::Mat GetWorldPos();

        cv::Mat GetNormal();
        KeyFrame *GetReferenceKeyFrame();

        std::map<KeyFrame *, size_t> GetObservations();
        int Observations();

        void AddObservation(KeyFrame *pKF, size_t idx);
        void EraseObservation(KeyFrame *pKF);

        void ReportOutlierObservation(KeyFrame *pKF);
        size_t GetNumOutlierObservation(KeyFrame *pKF);

        int GetIndexInKeyFrame(KeyFrame *pKF);
        bool IsInKeyFrame(KeyFrame *pKF);

        void SetBadFlag();
        bool isBad();

        void Replace(MapPoint *pMP);
        MapPoint *GetReplaced();

        void IncreaseVisible(int n = 1);
        void IncreaseFound(int n = 1);
        float GetFoundRatio();
        inline int GetFound()
        {
            return mnFound;
        }

        void ComputeDistinctiveDescriptors();

        cv::Mat GetDescriptor();

        void UpdateNormalAndDepth();

        float GetMinDistanceInvariance();
        float GetMaxDistanceInvariance();
        int PredictScale(const float &currentDist, KeyFrame *pKF);
        int PredictScale(const float &currentDist, Frame *pF);

    public:
        long unsigned int mnId;
        static long unsigned int nNextId;
        long int mnFirstKFid;
        long int mnFirstFrame;
        int nObs;

        // Variables used by the tracking
        float mTrackProjX;
        float mTrackProjY;
        float mTrackProjXR;
        bool mbTrackInView;
        int mnTrackScaleLevel;
        float mTrackViewCos;
        long unsigned int mnTrackReferenceForFrame;
        long unsigned int mnLastFrameSeen;

        // Variables used by local mapping
        long unsigned int mnBALocalForKF;
        long unsigned int mnFuseCandidateForKF;

        // Variables used by loop closing
        long unsigned int mnLoopPointForKF;
        long unsigned int mnCorrectedByKF;
        long unsigned int mnCorrectedReference;
        cv::Mat mPosGBA;
        long unsigned int mnBAGlobalForKF;

        static std::recursive_mutex mGlobalMutex;

    protected:
        // Position in absolute coordinates
        cv::Mat mWorldPos;

        // Keyframes observing the point and associated index in keyframe
        std::map<KeyFrame *, size_t> mObservations;
        std::map<KeyFrame *, size_t> mnOutliers;

        // Mean viewing direction
        cv::Mat mNormalVector;

        // Best descriptor to fast matching
        cv::Mat mDescriptor;

        // Reference KeyFrame
        KeyFrame *mpRefKF;

        // Tracking counters
        int mnVisible;
        int mnFound;

        // Bad flag (we do not currently erase MapPoint from memory)
        bool mbBad;
        MapPoint *mpReplaced;

        // Scale invariance distances
        float mfMinDistance;
        float mfMaxDistance;

        Map *mpMap;

        std::recursive_mutex mMutexPos;
        std::recursive_mutex mMutexFeatures;
    };

} // namespace LocSLAM

#endif // MAPPOINT_H
