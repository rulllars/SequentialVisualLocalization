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

#ifndef FRAME_H
#define FRAME_H

#include <vector>

#include "SLAMDataTypes.h"
#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "KeyFrame.h"
#include "ORBextractor.h"
#include "OdometryData.h"
#include <Eigen/Dense>

#include <opencv2/opencv.hpp>

namespace LocSLAM
{
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

    class MapPoint;
    class KeyFrame;
    //struct Correspondence2d3d;

    class Frame
    {
    public:
        Frame();

        // Copy constructor.
        Frame(const Frame &frame);

        // Constructor for stereo cameras.
        Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORB_SLAM2::ORBextractor *extractorLeft, ORB_SLAM2::ORBextractor *extractorRight, ORB_SLAM2::ORBVocabulary *voc, const int &cameraID, cv::Mat &Tcb, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

        // Constructor for RGB-D cameras.
        Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORB_SLAM2::ORBextractor *extractor, ORB_SLAM2::ORBVocabulary *voc, const int &cameraID, cv::Mat &Tcb, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

        // Constructor for Monocular cameras.
        Frame(const cv::Mat &imGray, const double &timeStamp, ORB_SLAM2::ORBextractor *extractor, ORB_SLAM2::ORBVocabulary *voc, const int &cameraID, cv::Mat &Tcb, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

        // Constructor for Monocular cameras with odometry.
        Frame(const cv::Mat &imGray, const OdometryData &odometry,
              const FixedMapMatches &frameFixeMapMatches,
              const double &timeStamp, ORB_SLAM2::ORBextractor *extractor, ORB_SLAM2::ORBVocabulary *voc,
              const int &cameraID, string strImgFileName, cv::Mat &Tcb, cv::Mat &K,
              cv::Mat &distCoef, const float &bf, const float &thDepth);

        // Extract ORB on the image. 0 for left image and 1 for right image.
        void ExtractORB(int flag, const cv::Mat &im);

        // Compute Bag of Words representation.
        void ComputeBoW();

        // Set the camera pose.
        void SetPose(cv::Mat Tbw);

        // Predict ose using odometry
        void PredictAndSetPose(cv::Mat T1bw);

        // Computes rotation, translation and body center matrices from the body pose.
        void UpdatePoseMatrices();

        // Returns the camera center.
        inline cv::Mat GetBodyPosition()
        {
            return mBodyPosition.clone();
        }

        // Returns inverse of rotation
        inline cv::Mat GetPoseRotationInverse()
        {
            return mRwb.clone();
        }

        // Returns the camera center.
        inline cv::Mat GetCameraCenter()
        {
            return mOw.clone();
        }

        // Returns inverse of rotation
        inline cv::Mat GetRotationInverse()
        {
            return mRwc.clone();
        }

        // Check if a MapPoint is in the frustum of the camera
        // and fill variables of the MapPoint to be used by the tracking
        bool isInFrustum(MapPoint *pMP, float viewingCosLimit);

        // Compute the cell of a keypoint (return false if outside the grid)
        bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

        vector<size_t> GetFeaturesInArea(const float &x, const float &y, const float &r, const int minLevel = -1, const int maxLevel = -1) const;

        // Search a match for each keypoint in the left image to a keypoint in the right image.
        // If there is a match, depth is computed and the right coordinate associated to the left keypoint is stored.
        void ComputeStereoMatches();

        // Associate a "right" coordinate to a keypoint if there is valid depth in the depthmap.
        void ComputeStereoFromRGBD(const cv::Mat &imDepth);

        // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
        cv::Mat UnprojectStereo(const int &i);

    public:
        // Vocabulary used for relocalization.
        ORB_SLAM2::ORBVocabulary *mpORBvocabulary;

        // Feature extractor. The right is used only in the stereo case.
        ORB_SLAM2::ORBextractor *mpORBextractorLeft, *mpORBextractorRight;

        // Frame timestamp.
        double mTimeStamp;

        // The ID of the camera generating the frame
        int mCameraID;

        // File name for logging purposes
        string mstrImgFileName;

        // Calibration matrix and OpenCV distortion parameters.
        cv::Mat mK;
        float mfx;
        float mfy;
        float mcx;
        float mcy;
        float minvfx;
        float minvfy;
        cv::Mat mDistCoef;

        // Stereo baseline multiplied by fx.
        float mbf;

        // Extrinsics as active transform from camera to body
        cv::Mat mTcb;

        // Relative transform to last key frame
        cv::Mat mTbr;

        // Stereo baseline in meters.
        float mb;

        // Threshold close/far points. Close points are inserted from 1 view.
        // Far points are inserted as in the monocular case from 2 views.
        float mThDepth;

        // Number of KeyPoints.
        int N;

        // Odometry data
        bool mbUseOdometry;
        OdometryData mOdometry;

        // Fixed global map
        FixedMapMatches mFixedMapMatches;

        // Indicate that it has been added as a key frame
        bool isKeyFrame = false;

        // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
        // In the stereo case, mvKeysUn is redundant as images must be rectified.
        // In the RGB-D case, RGB images can be distorted.
        std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
        std::vector<cv::KeyPoint> mvKeysUn;

        // Corresponding stereo coordinate and depth for each keypoint.
        // "Monocular" keypoints have a negative value.
        std::vector<float> mvuRight;
        std::vector<float> mvDepth;

        // Bag of Words Vector structures.
        DBoW2::BowVector mBowVec;
        DBoW2::FeatureVector mFeatVec;

        // ORB descriptor, each row associated to a keypoint.
        cv::Mat mDescriptors, mDescriptorsRight;

        // MapPoints associated to keypoints, NULL pointer if no association.
        std::vector<MapPoint *> mvpMapPoints;

        // Flag to identify outlier associations.
        std::vector<bool> mvbOutlier;

        // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
        float mfGridElementWidthInv;
        float mfGridElementHeightInv;
        std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

        // Body pose. Active transformation from vehicle to world.
        cv::Mat mTbw;
        Eigen::Matrix<double, 6, 6> mPoseCovariance;

        // Current and Next Frame id.
        static long unsigned int nNextId;
        long unsigned int mnId;

        // Reference Keyframe.
        KeyFrame *mpReferenceKF;

        // Las KeyFrame
        KeyFrame *mpLastKeyFrame;

        // Scale pyramid info.
        int mnScaleLevels;
        float mfScaleFactor;
        float mfLogScaleFactor;
        vector<float> mvScaleFactors;
        vector<float> mvInvScaleFactors;
        vector<float> mvLevelSigma2;
        vector<float> mvInvLevelSigma2;

        // Undistorted Image Bounds (computed once for every frame).
        float mnMinX;
        float mnMaxX;
        float mnMinY;
        float mnMaxY;

    private:
        // Undistort keypoints given OpenCV distortion parameters.
        // Only for the RGB-D case. Stereo must be already rectified!
        // (called in the constructor).
        void UndistortKeyPoints();

        void UndistortFixedKeyPoints();

        // Computes image bounds for the undistorted image (called in the constructor).
        void ComputeImageBounds(const cv::Mat &imLeft);

        // Assign keypoints to the grid for speed up feature matching (called in the constructor).
        void AssignFeaturesToGrid();

        // Camera: Rotation, translation and camera center

        // Camera pose. Active transformation from camera to world.
        cv::Mat mTcw;

        // Passive rotation world to camera
        cv::Mat mRcw;

        // World center in camera coordinate frame.
        cv::Mat mtcw;

        // Active rotation world to camera
        cv::Mat mRwc;

        // Camera center in world coordinate frame
        cv::Mat mOw;

        // Body: Rotation, translation and camera center (same convention as camera)
        cv::Mat mRbw;
        cv::Mat mtbw;
        cv::Mat mRwb;
        cv::Mat mBodyPosition; // 3d postion of body frame in world coodinates
    };

} // namespace LocSLAM

#endif // FRAME_H
