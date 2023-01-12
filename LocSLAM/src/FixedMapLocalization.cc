/**
* This file is part of LocSLAM.
*
* Copyright (C) 2021 Lars Hammarstrand <lars.hammarstrand at chalmers dot se> (Chalmers University of Technology)
* For more information see <https://github.com/rulllars/SequentialLocalization>
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
* along with locSLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include "FixedMapLocalization.h"
#include "Optimizer.h"
#include "CameraCalibration.h"

#include <mutex>

namespace LocSLAM
{

    FixedMapLocalization::FixedMapLocalization(Map *pMap, Map *pLocalMap, CameraCalibration *pCameraCalibrater)
        : mbResetRequested(false), mpMap(pMap), mpLocalMap(pLocalMap), mpCameraCalibrater(pCameraCalibrater)
    {

        // Fixed map transformation
        mFixedMapState = NOT_INITIALIZED;

        mTlf = cv::Mat::eye(4, 4, CV_32F);
        mTlfCovariance = Eigen::MatrixXd::Zero(6, 6);
        mTlfCovariance(0, 0) = mSigmaPosition * mSigmaPosition;
        mTlfCovariance(1, 1) = mSigmaPosition * mSigmaPosition;
        mTlfCovariance(2, 2) = mSigmaPosition * mSigmaPosition;
        mTlfCovariance(3, 3) = mSigmaHeading * mSigmaHeading;
        mTlfCovariance(4, 4) = mSigmaHeading * mSigmaHeading;
        mTlfCovariance(5, 5) = mSigmaHeading * mSigmaHeading;

        mpLastKeyFrame = static_cast<KeyFrame *>(NULL);
        mpFirstKeyFrame = static_cast<KeyFrame *>(NULL);
        mpCurrentKeyFrame = static_cast<KeyFrame *>(NULL);

        mnQueSize = 5;
        mnWindowSize = 6 * mnQueSize;
    }

    void FixedMapLocalization::InsertNewKeyFrame(KeyFrame *pKF)
    {
        // Insert key frame
        mlNewKeyFrames.push_back(pKF);
        if (mFixedMapState == NOT_INITIALIZED)
            InitializeFixedMap();
    }

    int FixedMapLocalization::KeyFramesInQue()
    {
        return mlNewKeyFrames.size();
    }

    bool FixedMapLocalization::IsKFQueFull()
    {
        return mnKeyFramesInQue >= mnQueSize;
    }

    Map *FixedMapLocalization::GetMap()
    {
        return mpMap;
    }

    void FixedMapLocalization::FixateTransformation(bool bFixate)
    {
        if (bFixate && mFixedMapState >= INITIALIZED)
            mFixedMapState = FIX_TRANSFORM;
    }

    cv::Mat FixedMapLocalization::GetTransformation()
    {
        return mTlf.clone();
    }

    void FixedMapLocalization::UpdateTransformation(cv::Mat newTlf, Eigen::MatrixXd newCov)
    {
        g2o::Vector6d transDiff = (Converter::toSE3Quat(mTlf) * Converter::toSE3Quat(newTlf.inv())).toMinimalVector();

        SetCovariance(newCov);
        SetTransformation(newTlf);

        if (!IsInitializing() && (transDiff.norm() < 8e-4 || DistFirstToLastKeyFrame() > 200))
            FixateTransformation(true);
    }

    void FixedMapLocalization::SetTransformation(cv::Mat Tlf)
    {
        mTlf = Tlf.clone();
        mpMap->SetTransformation(Tlf);
    }

    void FixedMapLocalization::SetCovariance(Eigen::MatrixXd Cov)
    {
        mTlfCovariance = Cov;
    }

    Eigen::MatrixXd FixedMapLocalization::GetInformation()
    {
        return mTlfCovariance.inverse();
    }

    Eigen::MatrixXd FixedMapLocalization::GetCovariance()
    {
        return mTlfCovariance;
    }

    bool FixedMapLocalization::CheckNewKeyFrames()
    {
        return (!mlNewKeyFrames.empty());
    }

    void FixedMapLocalization::ProcessNewKeyFrame()
    {

        mpCurrentKeyFrame = mlNewKeyFrames.front();
        mlNewKeyFrames.pop_front();

        // Gating of fixed map correspondences
        double covSpread = pow(mTlfCovariance.determinant(), 1.0 / 6.0);

        bool isOutlier = false;
        if (IsInitialized() && covSpread < 1e-4)
        {
            isOutlier = !CheckFixMapPose(mpCurrentKeyFrame);
        }

        mpCurrentKeyFrame->SetOutlierFlag(isOutlier);

        // Associate fixed MapPoints to the new keyframe
        for (int i = 0; i < mpCurrentKeyFrame->mFixedMapMatches.nMatches; i++)
        {
            cv::Mat pos = Converter::toCvMat(mpCurrentKeyFrame->mFixedMapMatches.vMatches[i].mapPoint);
            MapPoint *pMP = new MapPoint(pos, mpCurrentKeyFrame, mpMap);

            pMP->AddObservation(mpCurrentKeyFrame, i);

            mpCurrentKeyFrame->AddFixedMapPoint(pMP, i);

            mpMap->AddMapPoint(pMP);
        }

        // Insert Keyframe in Map
        mpMap->AddKeyFrame(mpCurrentKeyFrame);

        while (mlCurrentKeyFrameWindow.size() >= mnWindowSize)
            mlCurrentKeyFrameWindow.pop_front();

        // Insert in KeyFrame window
        mlCurrentKeyFrameWindow.push_back(mpCurrentKeyFrame);

        bAddedKeyFrames = true;
        mnKeyFramesInQue++;
    }

    bool FixedMapLocalization::CheckFixMapPose(KeyFrame *pKF)
    {
        const double chi2Threshold = 15.033207751218953; // 98%

        double sigma2Head = pow(pKF->mFixedMapMatches.sigmaPriorHeading * CV_PI / 180.0, 2);
        double sigma2Pos = pow(pKF->mFixedMapMatches.sigmaPriorHeading, 2);
        Eigen::VectorXd sigma2(6);
        sigma2 << sigma2Head, sigma2Head, sigma2Head, sigma2Pos, sigma2Pos, sigma2Pos;
        Eigen::Matrix<double, 6, 6> fixPoseCovariance = sigma2.asDiagonal();

        g2o::SE3Quat pred_Tbf = Converter::toSE3Quat(pKF->GetPose() * mTlf);
        g2o::SE3Quat fix_Tbf = Converter::toSE3Quat(pKF->mTcb.inv()) * pKF->mFixedMapMatches.Tcf;
        g2o::SE3Quat e = fix_Tbf * pred_Tbf.inverse();

        Eigen::Matrix<double, 6, 6> innovationCovariance = pKF->GetPoseCovariance() + fixPoseCovariance;

        double chi2 = e.toMinimalVector().transpose() * innovationCovariance.inverse() * e.toMinimalVector();

        if (chi2 > chi2Threshold)
        {
            //cout << "FixMapLocalizer: Fix pose for KF " << pKF->mnId << " assest as outlier (chi2 : " << chi2 << "). Not included in LocalBA." << endl;
            return false;
        }
        else
        {
            return true;
        }
    }

    bool FixedMapLocalization::LargeDistanceLastKeyframe()
    {
        return (DistToLastKeyFrame() > 40.0f);
    }

    double FixedMapLocalization::DistToLastKeyFrame()
    {
        double transDist = 0;

        if (mpLastKeyFrame != NULL && mpCurrentKeyFrame != NULL)
        {
            g2o::SE3Quat T21 = Converter::toSE3Quat(mpLastKeyFrame->GetPose() * mpCurrentKeyFrame->GetPoseInverse());
            transDist = T21.translation().norm();
        }

        return transDist;
    }

    double FixedMapLocalization::DistToLastKeyFrame(KeyFrame *pKF)
    {
        double transDist = 0;

        if (mpLastKeyFrame != NULL && pKF != NULL)
        {
            g2o::SE3Quat T21 = Converter::toSE3Quat(mpLastKeyFrame->GetPose() * pKF->GetPoseInverse());
            transDist = T21.translation().norm();
        }

        return transDist;
    }

    double FixedMapLocalization::DistFirstToLastKeyFrame()
    {
        double transDist = 0;

        if (mpLastKeyFrame != NULL && mpFirstKeyFrame != NULL)
        {
            g2o::SE3Quat T21 = Converter::toSE3Quat(mpFirstKeyFrame->GetPose() * mpLastKeyFrame->GetPoseInverse());
            transDist = T21.translation().norm();
        }

        return transDist;
    }

    list<KeyFrame *> FixedMapLocalization::GetKeyFrameWindow()
    {

        if (mFixedMapState == FIX_TRANSFORM)
            return mlCurrentKeyFrameWindow;
        else
        {
            vector<KeyFrame *> vAllKFs = mpMap->GetAllKeyFrames();
            return std::list<KeyFrame *>(vAllKFs.begin(), vAllKFs.end());
        }
    }

    void FixedMapLocalization::MapPointCulling()
    {
    }

    void FixedMapLocalization::KeyFrameCulling()
    {
        int nPointThreshold = 5;

        vector<KeyFrame *> vAllKFs = mpMap->GetAllKeyFrames();

        for (vector<KeyFrame *>::iterator iter = vAllKFs.begin(), end = vAllKFs.end(); iter != end; iter++)
        {
            KeyFrame *pKF = *iter;

            if (pKF->nFixedMapPoints() < nPointThreshold)
            {
                mpMap->EraseKeyFrame(pKF);
                mlCurrentKeyFrameWindow.remove(pKF);
            }
        }
    }

    bool FixedMapLocalization::InitializeFixedMap()
    {
        // Parameters
        const int nMinInliers = 1;
        const double minDistance = 0.01;
        const double relativeTranslationThreshold = 0.5;
        const double translationThreshold = 1.0;
        const double relativeRotationThreshold = 0.01;
        const double rotationThreshold = 0.08;

        // Check if we have enough key frames
        if (mlNewKeyFrames.size() > nMinInliers)
        {
            KeyFrame *pLastKF = mlNewKeyFrames.back();

            //What's the proposed transform from local to fixed map according to the latest KF?
            cv::Mat Tlf_hypothesis = pLastKF->GetPose().inv() * pLastKF->GetFixedMapPosePrior();

            //Now, check that against all previous KFs and count how many of them approximately agree
            int nInliers = 0;
            double timeStampFirstKF = DBL_MAX;
            for (const auto pKF : mlNewKeyFrames)
            {
                cv::Mat Tlf_check = pKF->GetPose().inv() * pKF->GetFixedMapPosePrior();
                g2o::SE3Quat diff = Converter::toSE3Quat(Tlf_check * Tlf_hypothesis.inv());
                cv::Mat T21LocalFrame = pLastKF->GetPose() * pKF->GetPoseInverse();
                cv::Vec3d t21LocalFrame = T21LocalFrame.col(3).rowRange(0, 3);
                double translation = cv::norm(t21LocalFrame);
                double transDiff = diff.translation().norm();
                double rotDiff = diff.rotation().vec().norm();
                if (transDiff < translationThreshold + translation * relativeTranslationThreshold &&
                    rotDiff < rotationThreshold + translation * relativeRotationThreshold &&
                    translation >= minDistance)
                {
                    nInliers++;

                    // Set point to first KF in map
                    if (timeStampFirstKF > pKF->mTimeStamp)
                    {
                        mpFirstKeyFrame = pKF;
                        timeStampFirstKF = pKF->mTimeStamp;
                    }
                }
            }

            // Initialize if enough key frames are consistent
            if (nInliers >= nMinInliers)
            {
                // Could perhaps use mean of inliers?
                SetTransformation(Tlf_hypothesis);

                mFixedMapState = INITIALIZING;

                return true;
            }
        }
        return false;
    }

    void FixedMapLocalization::Reset()
    {
        mlNewKeyFrames.clear();
    }

} // namespace LocSLAM
