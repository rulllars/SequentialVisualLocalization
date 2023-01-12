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

#ifndef INITIALIZER_H
#define INITIALIZER_H

#include <opencv2/opencv.hpp>
#include "Frame.h"

namespace LocSLAM
{

    //class Frame;

    // THIS IS THE INITIALIZER FOR MONOCULAR SLAM. NOT USED IN THE STEREO OR RGBD CASE.
    class Initializer
    {
        typedef pair<int, int> Match;

    public:
        // Fix the reference frame
        Initializer(const Frame &ReferenceFrame, KeyFrame *pLastKF, const OdometryData odoLastKF, float sigma = 1.0, int iterations = 200);

        // Computes in parallel a fundamental matrix and a homography
        // Selects a model and tries to recover the motion and the structure from motion
        bool Initialize(const Frame &CurrentFrame, const vector<int> &vMatches12,
                        cv::Mat &R21, cv::Mat &t21, vector<bool> &vbTriangulated);

        // Update reference frame if key frames have moved
        void UpdateReferenceFrame();
        Frame &GetReferenceFrame();

        // Initialization Variables (Monocular)
        std::vector<int> mvIniLastMatches;
        std::vector<cv::Point2f> mvbPrevMatched;
        std::vector<cv::Point3f> mvIniP3D;
        Frame mReferenceFrame;
        std::vector<int> mvIniMatches;

        // Integrated odometry from Last key frame to first initialization frame
        OdometryData mOdoLastKF2RefFrame;
        KeyFrame *mpLastKF;

        // Integrated odometry between initial frames
        OdometryData mOdometryDelta;

    private:
        void FindHomography(vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21);
        void FindFundamental(vector<bool> &vbInliers, float &score, cv::Mat &F21);

        cv::Mat ComputeH21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2);
        cv::Mat ComputeF21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2);

        float CheckHomography(const cv::Mat &H21, const cv::Mat &H12, vector<bool> &vbMatchesInliers, float sigma);

        float CheckFundamental(const cv::Mat &F21, vector<bool> &vbMatchesInliers, float sigma);

        // Try reconstructing using odometry data
        bool ReconstructOdo(vector<bool> &vbMatchesInliers, cv::Mat &R21, cv::Mat &t21, cv::Mat &K,
                            vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated);

        bool ReconstructF(vector<bool> &vbMatchesInliers, cv::Mat &F21, cv::Mat &K,
                          cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated);

        bool ReconstructH(vector<bool> &vbMatchesInliers, cv::Mat &H21, cv::Mat &K,
                          cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated);

        void Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D);

        void Normalize(const vector<cv::KeyPoint> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T);

        int CheckRT(const cv::Mat &R, const cv::Mat &t, const vector<cv::KeyPoint> &vKeys1, const vector<cv::KeyPoint> &vKeys2,
                    const vector<Match> &vMatches12, vector<bool> &vbInliers,
                    const cv::Mat &K, vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax);

        void DecomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t);

        // Keypoints from Reference Frame (Frame 1)
        vector<cv::KeyPoint> mvKeys1;

        // Keypoints from Current Frame (Frame 2)
        vector<cv::KeyPoint> mvKeys2;

        // Current Matches from Reference to Current
        vector<Match> mvMatches12;
        vector<bool> mvbMatched1;

        // Calibration
        cv::Mat mK;

        // Standard Deviation and Variance
        float mSigma, mSigma2;

        // Initialization thresholds
        int mMinTriangulatedOdo;
        int mMinTriangulatedF;
        int mMinTriangulatedH;

        int mMinMatched;

        // Ransac max iterations
        int mMaxIterations;

        // Ransac sets
        vector<vector<size_t>> mvSets;
    };

} // namespace LocSLAM

#endif // INITIALIZER_H
