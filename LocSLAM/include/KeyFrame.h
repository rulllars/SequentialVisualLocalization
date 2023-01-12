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

#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"
#include "OdometryData.h"
#include "SLAMDataTypes.h"
#include <Eigen/Dense>

#include <mutex>

namespace LocSLAM
{

    class Map;
    class MapPoint;
    class Frame;
    class KeyFrameDatabase;
    //struct Correspondence2d3d;

    class KeyFrame
    {
    public:
        KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB, const OdometryData &odo, KeyFrame *pPrevKF = NULL);

        // Pose functions
        void SetPose(const cv::Mat &Tbw);
        void SetPoseCovariance(Eigen::Matrix<double, 6, 6> cov);

        cv::Mat GetPose();
        Eigen::Matrix<double, 6, 6> GetPoseCovariance();

        cv::Mat GetPoseInverse();
        cv::Mat GetBodyPosition();
        cv::Mat GetBodyRotation();
        cv::Mat GetCameraPose();
        cv::Mat GetCameraPoseInverse();
        cv::Mat GetCameraCenter();
        cv::Mat GetStereoCenter();
        cv::Mat GetRotation();
        cv::Mat GetTranslation();
        cv::Mat GetCameraRotation();
        cv::Mat GetCameraTranslation();

        cv::Mat GetFixedMapPosePrior();

        // Bag of Words Representation
        void ComputeBoW();

        // Covisibility graph functions
        void AddConnection(KeyFrame *pKF, const int &weight);
        void EraseConnection(KeyFrame *pKF);
        void UpdateConnections();
        void UpdateBestCovisibles();
        std::set<KeyFrame *> GetConnectedKeyFrames();
        std::vector<KeyFrame *> GetVectorCovisibleKeyFrames();
        std::vector<KeyFrame *> GetBestCovisibilityKeyFrames(const int &N);
        std::vector<KeyFrame *> GetCovisiblesByWeight(const int &w);
        int GetWeight(KeyFrame *pKF);

        // Spanning tree functions
        void AddChild(KeyFrame *pKF);
        void EraseChild(KeyFrame *pKF);
        void ChangeParent(KeyFrame *pKF);
        std::set<KeyFrame *> GetChildren();
        KeyFrame *GetParent();
        bool hasChild(KeyFrame *pKF);

        // Odometry list functions
        KeyFrame *GetPrevKeyFrame(void);
        KeyFrame *GetNextKeyFrame(void);
        void InsertKeyFrameAfter(KeyFrame *pPrevKF, KeyFrame *pCurrKF, OdometryData odo);

        void InsertAfter(KeyFrame *pPrevKF, OdometryData odo);
        void Remove();

        void SetPrevKeyFrame(KeyFrame *pKF);
        void SetNextKeyFrame(KeyFrame *pKF);

        // Loop Edges
        void AddLoopEdge(KeyFrame *pKF);
        std::set<KeyFrame *> GetLoopEdges();

        // Local MapPoint observation functions
        void AddMapPoint(MapPoint *pMP, const size_t &idx);
        void EraseMapPointMatch(const size_t &idx);
        void EraseMapPointMatch(MapPoint *pMP);
        void ReplaceMapPointMatch(const size_t &idx, MapPoint *pMP);
        std::set<MapPoint *> GetMapPoints();
        std::vector<MapPoint *> GetMapPointMatches();
        int TrackedMapPoints(const int &minObs);
        MapPoint *GetMapPoint(const size_t &idx);

        // Fixed MapPoint observation functions
        void AddFixedMapPoint(MapPoint *pMP, const size_t &idx);
        void EraseFixedMapPointMatch(const size_t &idx);
        void EraseFixedMapPointMatch(MapPoint *pMP);
        //void ReplaceMapPointMatch(const size_t &idx, MapPoint *pMP);
        std::set<MapPoint *> GetFixedMapPoints();
        std::vector<MapPoint *> GetFixedMapPointMatches();
        int nFixedMapPoints();
        MapPoint *GetFixedMapPoint(const size_t &idx);

        std::vector<MapPoint *> &GetMapPointMatchesRef();

        // KeyPoint functions
        std::vector<size_t> GetFeaturesInArea(const float &x, const float &y, const float &r) const;
        cv::Mat UnprojectStereo(int i);

        // Image
        bool IsInImage(const float &x, const float &y) const;

        // Enable/Disable bad flag changes
        void SetNotErase();
        void SetErase();

        // Set/check bad flag
        void SetBadFlag();
        bool isBad();

        // Set/check fix map outlier flag
        void SetOutlierFlag(bool state);
        bool isOutlier();

        // Compute Scene Depth (q=2 median). Used in monocular.
        float ComputeSceneMedianDepth(const int q);

        static bool weightComp(int a, int b)
        {
            return a > b;
        }

        static bool lId(KeyFrame *pKF1, KeyFrame *pKF2)
        {
            return pKF1->mnId < pKF2->mnId;
        }

        static bool lTime(KeyFrame *pKF1, KeyFrame *pKF2)
        {
            return pKF1->mTimeStamp < pKF2->mTimeStamp;
        }

        // The following variables are accesed from only 1 thread or never change (no mutex needed).
    public:
        static long unsigned int nNextId;
        long unsigned int mnId;
        const long unsigned int mnFrameId;
        const int mCameraID;

        // File name for logging purposes
        string mstrImgFileName;

        const double mTimeStamp;

        // Grid (to speed up feature matching)
        const int mnGridCols;
        const int mnGridRows;
        const float mfGridElementWidthInv;
        const float mfGridElementHeightInv;

        // Variables used by the tracking
        long unsigned int mnTrackReferenceForFrame;
        long unsigned int mnFuseTargetForKF;

        // Variables used by the local mapping
        long unsigned int mnBALocalForKF;
        long unsigned int mnBAFixedForKF;

        // Variables used by the keyframe database
        long unsigned int mnLoopQuery;
        int mnLoopWords;
        float mLoopScore;
        long unsigned int mnRelocQuery;
        int mnRelocWords;
        float mRelocScore;

        // Variables used by loop closing
        cv::Mat mTcwGBA;
        cv::Mat mTcwBefGBA;
        long unsigned int mnBAGlobalForKF;

        // Calibration parameters
        const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;

        // Extrinsics
        cv::Mat mTcb;

        // Odometry
        OdometryData mOdometry;

        // Fixed global map
        FixedMapMatches mFixedMapMatches;

        // Number of KeyPoints
        const int N;

        // KeyPoints, stereo coordinate and descriptors (all associated by an index)
        const std::vector<cv::KeyPoint> mvKeys;
        const std::vector<cv::KeyPoint> mvKeysUn;
        const std::vector<float> mvuRight; // negative value for monocular points
        const std::vector<float> mvDepth;  // negative value for monocular points
        const cv::Mat mDescriptors;

        //BoW
        DBoW2::BowVector mBowVec;
        DBoW2::FeatureVector mFeatVec;

        // Pose relative to parent (this is computed when bad flag is activated)
        cv::Mat mTbp;

        // Scale
        const int mnScaleLevels;
        const float mfScaleFactor;
        const float mfLogScaleFactor;
        const std::vector<float> mvScaleFactors;
        const std::vector<float> mvLevelSigma2;
        const std::vector<float> mvInvLevelSigma2;

        // Image bounds and calibration
        const int mnMinX;
        const int mnMinY;
        const int mnMaxX;
        const int mnMaxY;
        const cv::Mat mK;

        // The following variables need to be accessed trough a mutex to be thread safe.
    protected:
        // Point to previous and next KeyFrame
        std::recursive_mutex mMutexPrevKF;
        std::recursive_mutex mMutexNextKF;
        KeyFrame *mpPrevKeyFrame;
        KeyFrame *mpNextKeyFrame;

        // SE3 Pose center
        cv::Mat Tbw;
        cv::Mat Twb;
        cv::Mat bodyPosition;

        Eigen::Matrix<double, 6, 6> mPoseCovariance;

        cv::Mat Cw; // Stereo middel point. Only for visualization

        // MapPoints associated to keypoints
        std::vector<MapPoint *> mvpMapPoints;

        // MapPoints associated to keypoints
        std::vector<MapPoint *> mvpFixedMapPoints;

        // BoW
        KeyFrameDatabase *mpKeyFrameDB;
        ORB_SLAM2::ORBVocabulary *mpORBvocabulary;

        // Grid over the image to speed up feature matching
        std::vector<std::vector<std::vector<size_t>>> mGrid;

        std::map<KeyFrame *, int> mConnectedKeyFrameWeights;
        std::vector<KeyFrame *> mvpOrderedConnectedKeyFrames;
        std::vector<int> mvOrderedWeights;

        // Spanning Tree and Loop Edges
        bool mbFirstConnection;
        KeyFrame *mpParent;
        std::set<KeyFrame *> mspChildrens;
        std::set<KeyFrame *> mspLoopEdges;

        // Bad flags
        bool mbNotErase;
        bool mbToBeErased;
        bool mbBad;
        bool mbFixMapOutlier;

        float mHalfBaseline; // Only for visualization

        Map *mpMap;

        std::recursive_mutex mMutexPose;
        std::recursive_mutex mMutexConnections;
        std::recursive_mutex mMutexFeatures;
    };

} // namespace LocSLAM

#endif // KEYFRAME_H
