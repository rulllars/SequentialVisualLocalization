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

#include "Optimizer.h"

#include <Eigen/StdVector>

#include "Converter.h"

#include <mutex>

namespace LocSLAM
{

    void Optimizer::GlobalBundleAdjustemnt(Map *pLocalMap, Map *pFixedMap, bool bfixFirstKF, int nIterations, bool *pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
    {
        vector<KeyFrame *> vpLocalKFs = pLocalMap->GetAllKeyFrames();
        vector<MapPoint *> vpLocalMP = pLocalMap->GetAllMapPoints();

        vector<KeyFrame *> vpFixedKFs = pFixedMap->GetAllKeyFrames();
        vector<MapPoint *> vpFixedMP = pFixedMap->GetAllMapPoints();

        BundleAdjustment(vpLocalKFs, vpLocalMP, vpFixedKFs, vpFixedMP, pFixedMap->GetTransformation(), bfixFirstKF, nIterations, pbStopFlag, nLoopKF, bRobust);
    }

    void Optimizer::BundleAdjustment(const std::vector<KeyFrame *> &vpLocalKFs, const std::vector<MapPoint *> &vpLocalMPs,
                                     const std::vector<KeyFrame *> &vpFixedKFs, const std::vector<MapPoint *> &vpFixedMPs,
                                     cv::Mat Tlf, bool bFixFirstKF, int nIterations, bool *pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
    {
        vector<bool> vbNotIncludedLocalMP;
        vector<bool> vbNotIncludedFixedMP;

        vbNotIncludedLocalMP.resize(vpLocalMPs.size());

        g2o::SparseOptimizer optimizer;
        g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

        linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

        g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        optimizer.setAlgorithm(solver);

        if (pbStopFlag)
            optimizer.setForceStopFlag(pbStopFlag);

        long unsigned int maxKFid = 0;

        // Set KeyFrame vertices
        SetKeyFrameVertices(optimizer, vpLocalKFs, bFixFirstKF, maxKFid);

        // Set pose pose contraint on local KeyFrames
        SetPosePoseConstraints(optimizer, vpLocalKFs);

        // Set local and fixed map point vertices
        SetMapPointVertices(optimizer, vpLocalMPs, vbNotIncludedLocalMP, vpFixedMPs, Tlf, maxKFid);

        // Optimize!
        cout << "BundleAdjustment: Start" << endl;
        optimizer.setVerbose(true);
        optimizer.initializeOptimization();
        optimizer.optimize(nIterations);
        cout << "BundleAdjustment: Finished" << endl;

        //Keyframes
        for (size_t i = 0; i < vpLocalKFs.size(); i++)
        {
            KeyFrame *pKF = vpLocalKFs[i];
            if (pKF->isBad())
                continue;
            g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pKF->mnId));
            g2o::SE3Quat SE3quat = vSE3->estimate();
            if (nLoopKF == 0)
            {
                pKF->SetPose(Converter::toCvMat(SE3quat));
            }
            else
            {
                pKF->mTcwGBA.create(4, 4, CV_32F);
                Converter::toCvMat(SE3quat).copyTo(pKF->mTcwGBA);
                pKF->mnBAGlobalForKF = nLoopKF;
            }
        }

        //Points
        for (size_t i = 0; i < vpLocalMPs.size(); i++)
        {
            if (vbNotIncludedLocalMP[i])
                continue;

            MapPoint *pMP = vpLocalMPs[i];

            if (pMP->isBad())
                continue;
            g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(pMP->mnId + maxKFid + 1));

            if (nLoopKF == 0)
            {
                pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
                pMP->UpdateNormalAndDepth();
            }
            else
            {
                pMP->mPosGBA.create(3, 1, CV_32F);
                Converter::toCvMat(vPoint->estimate()).copyTo(pMP->mPosGBA);
                pMP->mnBAGlobalForKF = nLoopKF;
            }
        }
    }

    int Optimizer::PoseOptimization(Frame *pFrame, bool bPosePrior)
    {
        g2o::SparseOptimizer optimizer;
        g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

        linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

        g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        optimizer.setAlgorithm(solver);

        int nInitialCorrespondences = 0;

        // Set Frame vertex
        g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTbw));
        vSE3->setId(0);
        vSE3->setFixed(false);
        optimizer.addVertex(vSE3);

        // Add prior
        if (bPosePrior)
        {
            g2o::EdgeSE3PosePrior *e = new g2o::EdgeSE3PosePrior;

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
            e->setMeasurement(Converter::toSE3Quat(pFrame->mTbw));

            Eigen::Matrix<double, 6, 6> Cov = pFrame->mOdometry.GetCovariance();
            e->setInformation(Cov.inverse());

            optimizer.addEdge(e);
        }

        // Set MapPoint vertices
        const int N = pFrame->N;

        vector<g2o::EdgeSE3ProjectXYZOnlyPose *> vpEdgesMono;
        vector<size_t> vnIndexEdgeMono;
        vpEdgesMono.reserve(N);
        vnIndexEdgeMono.reserve(N);

        vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose *> vpEdgesStereo;
        vector<size_t> vnIndexEdgeStereo;
        vpEdgesStereo.reserve(N);
        vnIndexEdgeStereo.reserve(N);

        const float deltaMono = sqrt(5.991);
        const float deltaStereo = sqrt(7.815);

        {
            unique_lock<recursive_mutex> lock(MapPoint::mGlobalMutex);

            for (int i = 0; i < N; i++)
            {
                MapPoint *pMP = pFrame->mvpMapPoints[i];
                if (pMP)
                {
                    // Monocular observation
                    if (pFrame->mvuRight[i] < 0)
                    {
                        nInitialCorrespondences++;
                        pFrame->mvbOutlier[i] = false;

                        Eigen::Matrix<double, 2, 1> obs;
                        const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                        obs << kpUn.pt.x, kpUn.pt.y;

                        g2o::EdgeSE3ProjectXYZOnlyPose *e = new g2o::EdgeSE3ProjectXYZOnlyPose();

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
                        e->setMeasurement(obs);
                        const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                        e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(deltaMono);

                        e->fx = pFrame->mfx;
                        e->fy = pFrame->mfy;
                        e->cx = pFrame->mcx;
                        e->cy = pFrame->mcy;
                        cv::Mat Xw = pMP->GetWorldPos();
                        e->Xw[0] = Xw.at<float>(0);
                        e->Xw[1] = Xw.at<float>(1);
                        e->Xw[2] = Xw.at<float>(2);

                        // Set extrinsics
                        e->Tcb = Converter::toSE3Quat(pFrame->mTcb);

                        optimizer.addEdge(e);

                        vpEdgesMono.push_back(e);
                        vnIndexEdgeMono.push_back(i);
                    }
                    else // Stereo observation
                    {
                        nInitialCorrespondences++;
                        pFrame->mvbOutlier[i] = false;

                        //SET EDGE
                        Eigen::Matrix<double, 3, 1> obs;
                        const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                        const float &kp_ur = pFrame->mvuRight[i];
                        obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                        g2o::EdgeStereoSE3ProjectXYZOnlyPose *e = new g2o::EdgeStereoSE3ProjectXYZOnlyPose();

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
                        e->setMeasurement(obs);
                        const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                        Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
                        e->setInformation(Info);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(deltaStereo);

                        e->fx = pFrame->mfx;
                        e->fy = pFrame->mfy;
                        e->cx = pFrame->mcx;
                        e->cy = pFrame->mcy;
                        e->bf = pFrame->mbf;
                        cv::Mat Xw = pMP->GetWorldPos();
                        e->Xw[0] = Xw.at<float>(0);
                        e->Xw[1] = Xw.at<float>(1);
                        e->Xw[2] = Xw.at<float>(2);

                        // Set extrinsics
                        e->Tcb = Converter::toSE3Quat(pFrame->mTcb);

                        optimizer.addEdge(e);

                        vpEdgesStereo.push_back(e);
                        vnIndexEdgeStereo.push_back(i);
                    }
                }
            }
        }

        if (nInitialCorrespondences < 3)
            return 0;

        // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
        // A t the next optimization, outliers are not included, but at the end they can be classified as inliers again.
        const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
        const float chi2Stereo[4] = {7.815, 7.815, 7.815, 7.815};
        // const int its[4]={10,10,10,10};
        // LH: Increase opt iterations to hopefully keep more matches
        const int its[4] = {10, 10, 20, 20};

        int nBad = 0;
        for (size_t it = 0; it < 4; it++)
        {

            vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTbw));

            optimizer.initializeOptimization(0);
            optimizer.setVerbose(false);
            optimizer.optimize(its[it]);

            nBad = 0;
            for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
            {
                g2o::EdgeSE3ProjectXYZOnlyPose *e = vpEdgesMono[i];

                const size_t idx = vnIndexEdgeMono[i];

                if (pFrame->mvbOutlier[idx])
                {
                    e->computeError();
                }

                const float chi2 = e->chi2();

                if (chi2 > chi2Mono[it])
                {
                    pFrame->mvbOutlier[idx] = true;
                    e->setLevel(1);
                    nBad++;
                }
                else
                {
                    pFrame->mvbOutlier[idx] = false;
                    e->setLevel(0);
                }

                if (it == 2)
                    e->setRobustKernel(0);
            }

            for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
            {
                g2o::EdgeStereoSE3ProjectXYZOnlyPose *e = vpEdgesStereo[i];

                const size_t idx = vnIndexEdgeStereo[i];

                if (pFrame->mvbOutlier[idx])
                {
                    e->computeError();
                }

                const float chi2 = e->chi2();

                if (chi2 > chi2Stereo[it])
                {
                    pFrame->mvbOutlier[idx] = true;
                    e->setLevel(1);
                    nBad++;
                }
                else
                {
                    e->setLevel(0);
                    pFrame->mvbOutlier[idx] = false;
                }

                if (it == 2)
                    e->setRobustKernel(0);
            }

            if (nInitialCorrespondences - nBad < 10)
                break;
        }

        // Recover optimized pose and return number of inliers
        g2o::VertexSE3Expmap *vSE3_recov = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(0));
        g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
        cv::Mat pose = Converter::toCvMat(SE3quat_recov);
        pFrame->SetPose(pose);

        cout << "PoseOpt: Camera: " << pFrame->mCameraID << ": Finished... (" << nInitialCorrespondences - nBad << "/" << nInitialCorrespondences << ")" << endl;

        /* Debug info if bad tracking
        if (nInitialCorrespondences - nBad < 10)
        {
            cout << "PoseOpt: Camera " << pFrame->mCameraID << ": Only " << nInitialCorrespondences - nBad << " matches after optimization" << endl;
            cout << nBad << " matches removed!" << endl;
        }
        else
        {
            cout << "PoseOpt: Camera: " << pFrame->mCameraID << ": Finished... (" << nInitialCorrespondences - nBad << "/" << nInitialCorrespondences << ")" << endl;
        }
        */

        return nInitialCorrespondences - nBad;
    }

    void Optimizer::LocalBundleAdjustment(KeyFrame *pKF, bool *pbStopFlag, Map *pLocalMap, FixedMapLocalization *pFixedMapLocalizer, bool bDoWindowedFixedMapBundle, bool bDoCameraCalibration)
    {
        // Set true to get debug info
        bool bPrintDebug = false;

        // Parameters
        const float thHuberMono = sqrt(5.991);
        const float thHuberStereo = sqrt(7.815);
        const float thHuberFixed = sqrt(5.991);

        // Local KeyFrames: First Breath Search from Current Keyframe
        list<KeyFrame *> lLocalKeyFrames;

        vector<bool> bLocalKeyFrame;
        bLocalKeyFrame.resize(pKF->nNextId, false);

        vector<bool> bFixedKeyFrame;
        bFixedKeyFrame.resize(pKF->nNextId, false);

        // Get Map Mutex
        unique_lock<recursive_mutex> lock(pLocalMap->mMutexMapUpdate);

        KeyFrame *pFirstLocalKF = pKF;
        double firstKFTimeStamp = pKF->mTimeStamp;

        // If fix map is initializing -> bundle all frames
        vector<KeyFrame *> vNeighKFs;
        if (pFixedMapLocalizer->IsInitializing() ||
            (bDoWindowedFixedMapBundle && !pFixedMapLocalizer->IsFixedTransformation()) ||
            bDoCameraCalibration)
        {
            vNeighKFs = pLocalMap->GetAllKeyFrames();
            bDoWindowedFixedMapBundle = true;
        }
        else
        {
            lLocalKeyFrames.push_back(pKF);
            //pKF->mnBALocalForKF = pKF->mnId;
            bLocalKeyFrame[pKF->mnId] = true;

            vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
        }

        //if (!bDoWindowedFixedMapBundle && vNeighKFs.empty())
        //    return; // No local map points in frame;

        // **************************************************************
        // Adding keyframes that share VO points with current key frame
        // as local frames to be bundled
        // **************************************************************

        if (bPrintDebug)
            cout << "Adding local key frames: " << pKF->mnId << ", ";

        for (int i = 0, iend = vNeighKFs.size(); i < iend; i++)
        {
            KeyFrame *pKFi = vNeighKFs[i];

            if (!pKFi->isBad())
            {
                lLocalKeyFrames.push_back(pKFi);
                bLocalKeyFrame[pKFi->mnId] = true;

                if (bPrintDebug)
                    cout << pKFi->mnId << ", ";

                if (pKFi->mTimeStamp < firstKFTimeStamp)
                {
                    pFirstLocalKF = pKFi;
                    firstKFTimeStamp = pKFi->mTimeStamp;
                }
            }
        }

        if (bPrintDebug)
            cout << "done!" << endl;

        // **************************************************************
        // If doing fixed map bundleing: Add a window of keyframes
        // having associated fixed map points as local frames to be bundled.
        // **************************************************************

        if (bDoWindowedFixedMapBundle)
        {
            // Add fixed Key frame window
            list<KeyFrame *> lFixedKeyFrameWindow = pFixedMapLocalizer->GetKeyFrameWindow();
            if (bPrintDebug)
                cout << "Adding fixed key frames: " << endl;
            for (list<KeyFrame *>::iterator lit = lFixedKeyFrameWindow.begin(), lend = lFixedKeyFrameWindow.end(); lit != lend; lit++)
            {
                KeyFrame *pKFi = *lit;

                // Add fixed KF to local frames

                if (!bLocalKeyFrame[pKFi->mnId] && !pKFi->isBad())
                {
                    lLocalKeyFrames.push_back(pKFi);
                    bLocalKeyFrame[pKFi->mnId] = true;
                    if (bPrintDebug)
                        cout << pKFi->mnId << ", ";

                    if (pKFi->mTimeStamp < firstKFTimeStamp)
                    {
                        pFirstLocalKF = pKFi;
                        firstKFTimeStamp = pKFi->mTimeStamp;
                    }
                }
            }

            // If last inlier Key frame is not in window, add it
            KeyFrame *pLastFixedKF = pFixedMapLocalizer->GetLastKeyFrame();
            if (pLastFixedKF && pFixedMapLocalizer->IsInitialized() && !bLocalKeyFrame[pLastFixedKF->mnId])
            {
                lLocalKeyFrames.push_back(pLastFixedKF);
                bLocalKeyFrame[pLastFixedKF->mnId] = true;

                if (pLastFixedKF->mTimeStamp < firstKFTimeStamp)
                {
                    pFirstLocalKF = pLastFixedKF;
                    firstKFTimeStamp = pLastFixedKF->mTimeStamp;
                }
            }

            if (bPrintDebug)
                cout << "done!" << endl;
        }
        else if (pFixedMapLocalizer->IsInitialized())
        {
            // No windowed bundle:
            // Add last kay frame for which we had fixed map points as local frame for consistency

            KeyFrame *pLastFixedKF = pFixedMapLocalizer->GetLastKeyFrame();

            if (pLastFixedKF && !bLocalKeyFrame[pLastFixedKF->mnId])
            {
                lLocalKeyFrames.push_back(pLastFixedKF);
                bLocalKeyFrame[pLastFixedKF->mnId] = true;

                if (pLastFixedKF->mTimeStamp < firstKFTimeStamp)
                {
                    pFirstLocalKF = pLastFixedKF;
                    firstKFTimeStamp = pLastFixedKF->mTimeStamp;
                }
            }
        }

        // **************************************************************
        // Add KeyFrames with odometry connection to local key frames
        // also to local key frames to be bundled.
        // **************************************************************
        list<KeyFrame *> lOdometryKeyFrames;
        if (bPrintDebug)
            cout << "Adding key frames connected through odometry: ";
        for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++)
        {
            KeyFrame *pKFi = *lit;

            // Include next key frame?
            KeyFrame *pKFnext = pKFi->GetNextKeyFrame();
            while (pKFnext != NULL)
            {
                if (!bLocalKeyFrame[pKFnext->mnId] && !pKFnext->isBad())
                {
                    lOdometryKeyFrames.push_back(pKFnext);
                    bLocalKeyFrame[pKFnext->mnId] = true;

                    if (bPrintDebug)
                        cout << "(" << pKFi->mnId << "," << pKFnext->mnId << "), ";
                }
                else
                {
                    break;
                }

                KeyFrame *tmp = pKFnext->GetNextKeyFrame();
                pKFnext = tmp;
            }

            // Include previous key frame?
            KeyFrame *pKFprev = pKFi->GetPrevKeyFrame();
            while (pKFprev != NULL)
            {
                if (!bLocalKeyFrame[pKFprev->mnId] && !pKFprev->isBad() && pKFprev->mTimeStamp > firstKFTimeStamp)
                {
                    lOdometryKeyFrames.push_back(pKFprev);
                    bLocalKeyFrame[pKFprev->mnId] = true;

                    if (bPrintDebug)
                        cout << "(" << pKFprev->mnId << "," << pKFi->mnId << "), ";
                }
                else
                {
                    break;
                }

                KeyFrame *tmp = pKFprev->GetPrevKeyFrame();
                pKFprev = tmp;
            }
        }

        if (bPrintDebug)
            cout << "done!" << endl;

        // Append odometry frames to local frames
        lLocalKeyFrames.splice(lLocalKeyFrames.end(), lOdometryKeyFrames);

        // **************************************************************
        // Find all local map points seen in the local key frames to be bundled.
        // **************************************************************
        list<MapPoint *> lLocalMapPoints;
        for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++)
        {
            vector<MapPoint *> vpMPs = (*lit)->GetMapPointMatches();
            for (vector<MapPoint *>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; vit++)
            {
                MapPoint *pMP = *vit;
                if (pMP)
                    if (!pMP->isBad())
                        if (pMP->mnBALocalForKF != pKF->mnId)
                        {
                            lLocalMapPoints.push_back(pMP);
                            pMP->mnBALocalForKF = pKF->mnId;
                        }
            }
        }

        // **************************************************************
        // Add fixed key frames that share local map points with local frames
        // to ground the bundle.
        // **************************************************************

        list<KeyFrame *> lFixedCameras;

        // Add first local KeyFrame as fixed KeyFrame
        KeyFrame *pConnectingKF = static_cast<KeyFrame *>(NULL);
        if (pFirstLocalKF != NULL)
        {
            if (!bFixedKeyFrame[pFirstLocalKF->mnId]) // &&
                //!(pFixedMapLocalizer->IsInitializing() || (bDoWindowedFixedMapBundle && !pFixedMapLocalizer->IsFixedTransformation())))
            {
                lFixedCameras.push_back(pFirstLocalKF);
                bFixedKeyFrame[pFirstLocalKF->mnId] = true;
                lLocalKeyFrames.remove(pFirstLocalKF);
                bLocalKeyFrame[pFirstLocalKF->mnId] = false;

                pConnectingKF = pFirstLocalKF;
                pFirstLocalKF = pConnectingKF->GetNextKeyFrame();
            }
        }

        // Fixed map Keyframes that see Local MapPoints but that are not Local Keyframes
        for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++)
        {
            map<KeyFrame *, size_t> observations = (*lit)->GetObservations();
            for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
            {
                KeyFrame *pKFi = mit->first;

                if (!bLocalKeyFrame[pKFi->mnId] && !bFixedKeyFrame[pKFi->mnId])
                {
                    if (!pKFi->isBad())
                    {
                        lFixedCameras.push_back(pKFi);
                        bFixedKeyFrame[pKFi->mnId] = true;
                    }
                }
            }
        }

        if (bPrintDebug)
            if (lFixedCameras.empty())
                cout << "LocalBA:\tNo fixed cameras for KF " << pKF->mnId << "!" << endl
                     << endl
                     << endl;

        // Set local key frame list in maps for visualization
        pLocalMap->SetLocalKeyFrames(lLocalKeyFrames);
        pLocalMap->SetLocalFixedKeyFrames(lFixedCameras);

        // **************************************************************
        // Setup optimizer, add vertices and edges
        // **************************************************************
        g2o::SparseOptimizer optimizer;
        g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

        linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

        g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        optimizer.setAlgorithm(solver);

        if (pbStopFlag)
            optimizer.setForceStopFlag(pbStopFlag);

        unsigned long maxKFid = 0;

        // **************************************************************
        // Add free vertices from the local KeyFrames
        // **************************************************************
        if (bPrintDebug)
            cout << "LocalBA:\tAdding pose vertex id: ";
        for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++)
        {
            KeyFrame *pKFi = *lit;
            g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
            vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
            vSE3->setId(pKFi->mnId);

            if (bPrintDebug)
                cout << pKFi->mnId << ", ";

            vSE3->setFixed(pKFi->mnId == 0 && !pFixedMapLocalizer->IsInitializing());
            optimizer.addVertex(vSE3);
            if (pKFi->mnId > maxKFid)
                maxKFid = pKFi->mnId;
        }
        if (bPrintDebug)
            cout << "done!" << endl;

        // **************************************************************
        // Add fixed vertices from the fixed KeyFrames
        // **************************************************************
        if (bPrintDebug)
            cout << "LocalBA:\tAdding fixed pose vertex id: ";

        for (list<KeyFrame *>::iterator lit = lFixedCameras.begin(), lend = lFixedCameras.end(); lit != lend; lit++)
        {
            KeyFrame *pKFi = *lit;
            g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
            vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
            vSE3->setId(pKFi->mnId);

            if (bPrintDebug)
                cout << pKFi->mnId << ", ";

            vSE3->setFixed(true);
            optimizer.addVertex(vSE3);
            if (pKFi->mnId > maxKFid)
                maxKFid = pKFi->mnId;
        }
        if (bPrintDebug)
            cout << "done!" << endl;

        // **************************************************************
        // Add vertex for camera calibration
        // **************************************************************
        vector<int> vCalibVertices;
        if (bDoCameraCalibration)
        {

            // Get current delta calibration and covariance
            vector<Eigen::Matrix<double, 6, 6>> vCovariance = pFixedMapLocalizer->mpCameraCalibrater->GetCovariance();

            for (auto it_cov = vCovariance.begin(); it_cov != vCovariance.end(); ++it_cov)
            {
                g2o::VertexSE3Expmap *vSE3Calib = new g2o::VertexSE3Expmap();
                vSE3Calib->setEstimate(Converter::toSE3Quat(cv::Mat::eye(4, 4, CV_32F)));
                vSE3Calib->setId(++maxKFid);
                vCalibVertices.push_back(maxKFid);

                vSE3Calib->setFixed(false);
                optimizer.addVertex(vSE3Calib);

                // Add calibration prior
                g2o::EdgeSE3PosePrior *eCalib = new g2o::EdgeSE3PosePrior;
                eCalib->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(vCalibVertices.back())));
                eCalib->setMeasurement(Converter::toSE3Quat(cv::Mat::eye(4, 4, CV_32F)));
                eCalib->setInformation(it_cov->inverse());
                optimizer.addEdge(eCalib);
            }
        }

        // **************************************************************
        // Add vertex for the transformation from global (fixed) to local map
        // **************************************************************
        unsigned long nGlobalMapTransID = ++maxKFid;
        if (pFixedMapLocalizer->IsInitialized() || pFixedMapLocalizer->IsInitializing())
        {
            g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();

            if (bPrintDebug)
                cout << "FixMap transform: " << endl
                     << pFixedMapLocalizer->GetTransformation() << endl;

            vSE3->setEstimate(Converter::toSE3Quat(pFixedMapLocalizer->GetTransformation()));
            vSE3->setId(nGlobalMapTransID);

            if (bDoWindowedFixedMapBundle && !pFixedMapLocalizer->IsFixedTransformation())
                vSE3->setFixed(false);
            else
                vSE3->setFixed(true);

            optimizer.addVertex(vSE3);

            if (bDoWindowedFixedMapBundle && !pFixedMapLocalizer->IsFixedTransformation())
            {
                // Set pose prior on fixed -> local transformation
                g2o::EdgeSE3PosePrior *e = new g2o::EdgeSE3PosePrior;

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nGlobalMapTransID)));
                e->setMeasurement(Converter::toSE3Quat(pFixedMapLocalizer->GetTransformation()));

                e->setInformation(pFixedMapLocalizer->GetInformation());

                if (bPrintDebug)
                    cout << "FixMap covariance: " << endl
                         << pFixedMapLocalizer->GetCovariance() << endl;

                optimizer.addEdge(e);
            }
        }

        // **************************************************************
        // Add pose-pose contraints between local keyframes using odometry
        // **************************************************************
        const float thHuberPose = sqrt(12.5916); // 95 %
        if (bPrintDebug)
            cout << "LocalBA:\tAdding pose-pose constraint between vertices: ";
        for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++)
        {
            KeyFrame *pKFi = *lit;
            g2o::EdgeSE3PosePose *eSE3 = new g2o::EdgeSE3PosePose();

            if (pKFi->GetNextKeyFrame() != NULL)
            {
                // Set odometry constraint and information
                KeyFrame *pKFnext = pKFi->GetNextKeyFrame();

                if (!pKFnext->isBad())
                {
                    // Set vetices
                    eSE3->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
                    eSE3->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFnext->mnId)));

                    if (bPrintDebug)
                        cout << "(" << pKFi->mnId << ", " << pKFnext->mnId << "), ";

                    g2o::SE3Quat T21 = Converter::toSE3Quat(pKFnext->mOdometry.GetTransformation());
                    eSE3->setMeasurement(T21);

                    if (pKFnext->mOdometry.CheckCovariance())
                        cout << "LocalBA:\tAdjusted odometry covariance on KF " << pKFnext->mnId << "!" << endl;

                    Eigen::Matrix<double, 6, 6> informationMatrix = pKFnext->mOdometry.GetCovariance().inverse();
                    eSE3->setInformation(informationMatrix);

                    // Add to graph
                    optimizer.addEdge(eSE3);
                }
            }
        }
        if (bPrintDebug)
            cout << "done!" << endl;

        // Add pose pose constraint between last fixed KF and local KF
        if (pFirstLocalKF != NULL)
        {
            if (bPrintDebug)
                cout << "LocalBA:\tAdding pose-pose constraint between fixed vertices: " << endl;

            g2o::EdgeSE3PosePose *eSE3 = new g2o::EdgeSE3PosePose();

            if (pConnectingKF != NULL)
            {
                KeyFrame *pKF1 = pConnectingKF;
                KeyFrame *pKF2;
                if (pConnectingKF->mnId == pFirstLocalKF->mnId)
                    pKF2 = pConnectingKF->GetNextKeyFrame();
                else
                    pKF2 = pFirstLocalKF;

                if (!pKF1->isBad())
                {
                    // Set vetices
                    eSE3->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKF1->mnId)));
                    eSE3->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKF2->mnId)));

                    if (bPrintDebug)
                        cout << "(" << pKF1->mnId << ", " << pKF2->mnId << "), ";

                    // Set odometry constraint and information

                    g2o::SE3Quat T21 = Converter::toSE3Quat(pKF2->mOdometry.GetTransformation());
                    eSE3->setMeasurement(T21);

                    if (pKF2->mOdometry.CheckCovariance())
                        if (bPrintDebug)
                            cout << "LocalBA:\tAdjusted odometry covariance on KF " << pKF2->mnId << "!" << endl;

                    Eigen::MatrixXd informationMatrix = pKF2->mOdometry.GetCovariance().inverse();
                    eSE3->setInformation(informationMatrix);

                    // Add to graph
                    optimizer.addEdge(eSE3);
                }
            }

            if (bPrintDebug)
                cout << "done!" << endl;
        }

        // **************************************************************
        // Add local map point vertices
        // **************************************************************

        const size_t nExpectedSize = min(1000000UL, (lLocalKeyFrames.size() + lFixedCameras.size()) * lLocalMapPoints.size());

        vector<KeyFrame *> vpEdgeKFMono;
        vpEdgeKFMono.reserve(nExpectedSize);

        vector<MapPoint *> vpMapPointEdgeMono;
        vpMapPointEdgeMono.reserve(nExpectedSize);

        vector<g2o::EdgeSE3ProjectXYZCalib *> vpEdgesMonoCalib;
        vector<g2o::EdgeSE3ProjectXYZ *> vpEdgesMono;

        if (bDoCameraCalibration)
        {
            vpEdgesMonoCalib.reserve(nExpectedSize);
            SetMapPointVertices(optimizer, lLocalMapPoints, false, vpEdgesMonoCalib, vpEdgeKFMono, vpMapPointEdgeMono, maxKFid, thHuberMono, vCalibVertices);
        }
        else
        {
            vpEdgesMono.reserve(nExpectedSize);
            SetMapPointVertices(optimizer, lLocalMapPoints, false, vpEdgesMono, vpEdgeKFMono, vpMapPointEdgeMono, maxKFid, thHuberMono);
        }

        vector<g2o::EdgeStereoSE3ProjectXYZ *> vpEdgesStereo;
        vpEdgesStereo.reserve(nExpectedSize);

        vector<KeyFrame *> vpEdgeKFStereo;
        vpEdgeKFStereo.reserve(nExpectedSize);

        vector<MapPoint *> vpMapPointEdgeStereo;
        vpMapPointEdgeStereo.reserve(nExpectedSize);

        // Stereo camera observations are not currently supported
        //SetMapPointVertices(optimizer, lLocalMapPoints, false, vpEdgesStereo, vpEdgeKFStereo, vpMapPointEdgeStereo, maxKFid);

        // **************************************************************
        // Add fixed (global) map point vertices
        // **************************************************************
        vector<g2o::EdgeSE3ProjectXYZFixed *> vpEdgesFixed;
        vector<KeyFrame *> vpEdgeKFFixed;
        vector<MapPoint *> vpMapPointEdgeFixed;

        if (pFixedMapLocalizer->IsInitialized())
        {
            if (pFixedMapLocalizer->LargeDistanceLastKeyframe())
                // Increase Huber threshold to allow for larger basin of convergence
                SetMapPointVertices(optimizer, lLocalKeyFrames, vpEdgesFixed, vpEdgeKFFixed,
                                    vpMapPointEdgeFixed, nGlobalMapTransID, sqrt(20) * thHuberFixed, HUBER, true);
            else if (bDoWindowedFixedMapBundle)
                SetMapPointVertices(optimizer, lLocalKeyFrames, vpEdgesFixed, vpEdgeKFFixed,
                                    vpMapPointEdgeFixed, nGlobalMapTransID, thHuberFixed, CAUCHY, true);
            else
            {
                SetMapPointVertices(optimizer, lLocalKeyFrames, vpEdgesFixed, vpEdgeKFFixed,
                                    vpMapPointEdgeFixed, nGlobalMapTransID, thHuberFixed, TUKEY, true);
            }
        }

        if (pbStopFlag)
            if (*pbStopFlag)
                return;

        // Update bundled map points for visualization
        pLocalMap->SetLocalMapPoints(vpMapPointEdgeMono);
        pFixedMapLocalizer->GetMap()->SetLocalMapPoints(vpMapPointEdgeFixed);

        // **************************************************************
        // Start first bundle
        // **************************************************************

        if (bPrintDebug)
            cout << "LocalBA:\tStart first opt..." << endl;
        if (bPrintDebug)
            optimizer.setVerbose(true);
        optimizer.initializeOptimization();
        if (pFixedMapLocalizer->LargeDistanceLastKeyframe())
            optimizer.optimize(50);
        else
            optimizer.optimize(15);

        optimizer.computeActiveErrors();
        if (optimizer.activeRobustChi2() < 0 || isnan(optimizer.activeRobustChi2()))
        {
            cout << "Negative or NaN cost function!!" << endl;
        }

        if (bPrintDebug)
            cout << "LocalBA:\tFirst opt finished (chi2: " << optimizer.chi2() << ")" << endl;

        bool bDoMore = true;

        if (pbStopFlag)
            if (*pbStopFlag)
                bDoMore = false;

        if (bDoMore)
        {

            // **************************************************************
            // Check for outliers among local map points and remove from
            // second bundle.
            // **************************************************************

            // Check inlier observations
            int nOutliers = 0;
            for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
            {
                g2o::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
                MapPoint *pMP = vpMapPointEdgeMono[i];

                if (pMP->isBad())
                    continue;

                g2o::Vector3d rho;
                e->robustKernel()->robustify(e->chi2(), rho);

                if (e->chi2() > pow(thHuberMono, 2.0) || !e->isDepthPositive())
                {
                    e->setLevel(1);
                    nOutliers++;
                }

                e->setRobustKernel(0);
            }

            // Check inlier observations
            for (size_t i = 0, iend = vpEdgesMonoCalib.size(); i < iend; i++)
            {
                g2o::EdgeSE3ProjectXYZCalib *e = vpEdgesMonoCalib[i];
                MapPoint *pMP = vpMapPointEdgeMono[i];

                if (pMP->isBad())
                    continue;

                g2o::Vector3d rho;
                e->robustKernel()->robustify(e->chi2(), rho);

                if (e->chi2() > pow(thHuberMono, 2.0) || !e->isDepthPositive())
                {
                    e->setLevel(1);
                    nOutliers++;
                }

                e->setRobustKernel(0);
            }

            for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
            {
                g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
                MapPoint *pMP = vpMapPointEdgeStereo[i];

                if (pMP->isBad())
                    continue;

                g2o::Vector3d rho;
                e->robustKernel()->robustify(e->chi2(), rho);

                if (e->chi2() > pow(thHuberStereo, 2.0) || !e->isDepthPositive())
                {
                    e->setLevel(1);
                }

                e->setRobustKernel(0);
            }

            // **************************************************************
            // Check for outliers among fixed map points and remove from
            // second bundle.
            // **************************************************************

            // Check inlier observations
            int nFixedOutliers = 0;
            for (size_t i = 0, iend = vpEdgesFixed.size(); i < iend; i++)
            {
                g2o::EdgeSE3ProjectXYZFixed *e = vpEdgesFixed[i];
                MapPoint *pMP = vpMapPointEdgeFixed[i];

                if (pMP->isBad())
                    continue;

                g2o::Vector3d rho;
                e->robustKernel()->robustify(e->chi2(), rho);

                if (e->chi2() > pow(thHuberFixed, 2.0) || !e->isDepthPositive())
                {
                    //cout << "MP removed! (Chi2: " << e->chi2() << endl;
                    e->setLevel(1);
                    nFixedOutliers++;
                }

                e->setRobustKernel(0);
            }

            // **************************************************************
            // Start second bundle where outliers are removed
            // **************************************************************

            if (bPrintDebug)
                cout << "LocalBA:\tStart second opt (" << nOutliers << " local points removed, "
                     << nFixedOutliers << " fixed points removed of " << pFixedMapLocalizer->GetMap()->MapPointsInMap() << ")" << endl;

            if (bPrintDebug)
                optimizer.setVerbose(true);
            optimizer.initializeOptimization(0);
            if (bDoWindowedFixedMapBundle)
                optimizer.optimize(20);
            else
                optimizer.optimize(15);

            optimizer.computeActiveErrors();

            if (bPrintDebug)
                cout << "LocalBA:\tSecond opt finished (chi2: " << optimizer.chi2() << ")" << endl;
        }

        // **************************************************************
        // Clean-up by removeing outlier map points
        // **************************************************************

        vector<pair<KeyFrame *, MapPoint *>> vToErase;
        vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());

        // Check inlier observations
        for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
        {
            g2o::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
            MapPoint *pMP = vpMapPointEdgeMono[i];

            if (pMP->isBad())
                continue;

            if (e->chi2() > pow(thHuberMono, 2.0) || !e->isDepthPositive())
            {
                KeyFrame *pKFi = vpEdgeKFMono[i];
                pMP->ReportOutlierObservation(pKFi);

                if (pMP->GetNumOutlierObservation(pKFi) >= 1)
                    vToErase.push_back(make_pair(pKFi, pMP));
            }
        }

        // Check inlier observations
        for (size_t i = 0, iend = vpEdgesMonoCalib.size(); i < iend; i++)
        {
            g2o::EdgeSE3ProjectXYZCalib *e = vpEdgesMonoCalib[i];
            MapPoint *pMP = vpMapPointEdgeMono[i];

            if (pMP->isBad())
                continue;

            if (e->chi2() > pow(thHuberMono, 2.0) || !e->isDepthPositive())
            {
                KeyFrame *pKFi = vpEdgeKFMono[i];
                pMP->ReportOutlierObservation(pKFi);

                if (pMP->GetNumOutlierObservation(pKFi) >= 1)
                    vToErase.push_back(make_pair(pKFi, pMP));
            }
        }

        for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
        {
            g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
            MapPoint *pMP = vpMapPointEdgeStereo[i];

            if (pMP->isBad())
                continue;

            if (e->chi2() > pow(thHuberStereo, 2.0) || !e->isDepthPositive())
            {
                KeyFrame *pKFi = vpEdgeKFStereo[i];
                pMP->ReportOutlierObservation(pKFi);

                if (pMP->GetNumOutlierObservation(pKFi) >= 1)
                    vToErase.push_back(make_pair(pKFi, pMP));
            }
        }

        // Check inlier observations
        int nFixedOutliers = 0;
        vector<pair<KeyFrame *, MapPoint *>> vToEraseFixed;
        vToEraseFixed.reserve(vpEdgesFixed.size());

        for (size_t i = 0, iend = vpEdgesFixed.size(); i < iend; i++)
        {
            g2o::EdgeSE3ProjectXYZFixed *e = vpEdgesFixed[i];
            MapPoint *pMP = vpMapPointEdgeFixed[i];

            if (pMP->isBad())
                continue;

            if (e->chi2() > pow(thHuberFixed, 2.0) || !e->isDepthPositive())
            {
                e->setLevel(1);

                nFixedOutliers++;
                KeyFrame *pKFi = vpEdgeKFFixed[i];

                if (bDoWindowedFixedMapBundle && !pFixedMapLocalizer->LargeDistanceLastKeyframe())
                    pMP->ReportOutlierObservation(pKFi);

                if (pMP->GetNumOutlierObservation(pKFi) >= 1)
                    vToEraseFixed.push_back(make_pair(pKFi, pMP));
            }

            e->setRobustKernel(0);
        }

        // Get Map Mutex

        unsigned int erasedLocalMapPoints = 0;
        unsigned int erasedFixedMapPoints = 0;
        {
            unique_lock<recursive_mutex> lock(pLocalMap->mMutexMapUpdate);

            for (size_t i = 0; i < vToErase.size(); i++)
            {
                KeyFrame *pKFi = vToErase[i].first;
                MapPoint *pMPi = vToErase[i].second;
                pKFi->EraseMapPointMatch(pMPi);
                pMPi->EraseObservation(pKFi);
                erasedLocalMapPoints++;
            }
        }

        for (size_t i = 0; i < vToEraseFixed.size(); i++)
        {
            KeyFrame *pKFi = vToEraseFixed[i].first;
            MapPoint *pMPi = vToEraseFixed[i].second;
            pKFi->EraseFixedMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
            erasedFixedMapPoints++;
        }

        cout << "LocalBA:\tFinished bundle adjust of new KFs!" << endl;

        // **************************************************************
        // Update keyframes and map points with final bundle result
        // **************************************************************

        // Keyframes...
        int nLastFixedKeyFrameID = -1;
        for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++)
        {
            KeyFrame *pKF = *lit;
            g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pKF->mnId));

            g2o::SE3Quat SE3quat = vSE3->estimate();
            pKF->SetPose(Converter::toCvMat(SE3quat));
        }

        // Map Points...
        for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++)
        {
            MapPoint *pMP = *lit;
            g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(pMP->mnId + maxKFid + 1));

            pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
            pMP->UpdateNormalAndDepth();
        }

        // **************************************************************
        // Admin
        // **************************************************************

        // Rest Que if windowed bundle
        if (bDoWindowedFixedMapBundle)
        {
            pFixedMapLocalizer->mnKeyFramesInQue = 0;
        }

        // Check for fix map inlier poses after windowed bundle
        if (bDoWindowedFixedMapBundle)
        {
            //sort(vpEdgeKFFixed.begin(), vpEdgeKFFixed.begin());
            vector<KeyFrame *> vpFixMapKFs = vpEdgeKFFixed;
            sort(vpFixMapKFs.begin(), vpFixMapKFs.end());
            vector<KeyFrame *>::iterator itr = unique(vpFixMapKFs.begin(), vpFixMapKFs.end());
            vpFixMapKFs.resize(distance(vpFixMapKFs.begin(), itr));

            for (vector<KeyFrame *>::iterator lit = vpFixMapKFs.begin(), lend = vpFixMapKFs.end(); lit != lend; lit++)
            {
                KeyFrame *pKFi = *lit;

                g2o::VertexSE3Expmap *vSE3_KF = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pKFi->mnId));
                if (!vSE3_KF->fixed())
                {
                    Eigen::Matrix<double, 6, 6> H_KF = Eigen::MatrixXd::Zero(6, 6);
                    for (int i = 0; i < 6; i++)
                        for (int j = 0; j < 6; j++)
                            H_KF(i, j) = vSE3_KF->hessian(i, j);

                    pKFi->SetPoseCovariance(H_KF.inverse());
                }

                if (pFixedMapLocalizer->IsInitialized())
                    pKFi->SetOutlierFlag(!pFixedMapLocalizer->CheckFixMapPose(pKFi));

                // Update last bundled key frame
                if (!pKFi->isOutlier() && (pFixedMapLocalizer->GetLastKeyFrame() == NULL || pFixedMapLocalizer->GetLastKeyFrame()->mTimeStamp < pKFi->mTimeStamp))
                    pFixedMapLocalizer->SetLastKeyFrame(pKFi);
            }
        }

        // Update covariance of current KF
        if (!bFixedKeyFrame[pKF->mnId])
        {
            g2o::VertexSE3Expmap *vSE3_currKF = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pKF->mnId));
            Eigen::Matrix<double, 6, 6> H_currKF = Eigen::MatrixXd::Zero(6, 6);
            for (int i = 0; i < 6; i++)
                for (int j = 0; j < 6; j++)
                    H_currKF(i, j) = vSE3_currKF->hessian(i, j);

            pKF->SetPoseCovariance(H_currKF.inverse());
        }

        // Set last bundled key frame
        pLocalMap->SetLastBundledKeyFrame(pKF);

        if (bDoWindowedFixedMapBundle && (pFixedMapLocalizer->IsInitialized() || pFixedMapLocalizer->IsInitializing()))
        {

            // Update estimated global transformation if not fixed
            if (!pFixedMapLocalizer->IsFixedTransformation())
            {
                // Fixed to local transformation
                g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(nGlobalMapTransID));
                g2o::SE3Quat SE3quat = vSE3->estimate();

                Eigen::MatrixXd Hessian = Eigen::MatrixXd::Zero(6, 6);

                for (int i = 0; i < 6; i++)
                    for (int j = 0; j < 6; j++)
                        Hessian(i, j) = vSE3->hessian(i, j);

                if (bPrintDebug)
                    cout << "LocalBA:\tHessian: " << endl
                         << Hessian << endl;

                if (bPrintDebug)
                    cout << "LocalBA:\tTfl before BA: " << endl
                         << pFixedMapLocalizer->GetTransformation().inv() << endl;

                if (bPrintDebug)
                    cout << "LocalBA:\tTfl after BA: " << endl
                         << SE3quat.inverse() << endl;

                g2o::Vector6d transDiff = (Converter::toSE3Quat(pFixedMapLocalizer->GetTransformation().inv()) * SE3quat).toMinimalVector();

                if (bPrintDebug)
                    cout << "LocalBA:\tGlobal transformation update: " << endl
                         << transDiff << endl
                         << "Norm: " << transDiff.norm() << endl;

                pFixedMapLocalizer->UpdateTransformation(Converter::toCvMat(SE3quat), Hessian.inverse());
            }

            if (pFixedMapLocalizer->IsInitializing())
                pFixedMapLocalizer->SetInitialized();
        }

        if (bDoCameraCalibration)
        {
            vector<cv::Mat> vDeltaCalibration;
            vector<Eigen::Matrix<double, 6, 6>> vCovariance;

            for (int iCamera = 0; iCamera < vCalibVertices.size(); iCamera++)
            {
                g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(vCalibVertices[iCamera]));
                g2o::SE3Quat SE3quat = vSE3->estimate();
                //Eigen::Matrix<double, 6,6> hessian = vSE3->hessianData():
                vDeltaCalibration.push_back(Converter::toCvMat(SE3quat));

                Eigen::MatrixXd Hessian = Eigen::MatrixXd::Zero(6, 6);

                for (int i = 0; i < 6; i++)
                    for (int j = 0; j < 6; j++)
                        Hessian(i, j) = vSE3->hessian(i, j);

                vCovariance.push_back(Hessian.inverse());
            }

            pFixedMapLocalizer->mpCameraCalibrater->UpdateCalibration(vDeltaCalibration, vCovariance);
        }
    }

    void Optimizer::InitialBundleAdjustment(vector<KeyFrame *> pInitKFs, OdometryData &Odo, Map *pMap)
    {
        // Print debug info
        bool bPrintDebug = false;

        // MapPoints seen in init KeyFrames
        list<MapPoint *> lMapPoints;
        //vector<bool> vbAddedMapPoint;
        //vbAddedMapPoint.resize(pMap->MapPointsInMap(), false);

        int nKFs = 0;

        vector<MapPoint *> vpMPs = pInitKFs[1]->GetMapPointMatches();
        vector<MapPoint *> &vpMPsRef = pInitKFs[1]->GetMapPointMatchesRef();

        for (vector<MapPoint *>::iterator vit = vpMPsRef.begin(), vend = vpMPsRef.end(); vit != vend; vit++)
        {
            MapPoint *pMP = *vit;
            if (pMP && !pMP->isBad())
            {
                if (pMP->mnBALocalForKF != pInitKFs[1]->mnId)
                {
                    lMapPoints.push_back(pMP);
                    pMP->mnBALocalForKF = pInitKFs[1]->mnId;
                }
            }
        }

        // Fixed Keyframes that see Local MapPoints but that are not Local Keyframes
        list<KeyFrame *> lFixedCameras;
        vector<bool> bFixedKeyFrame;
        bFixedKeyFrame.resize(pInitKFs[1]->nNextId, false);

        vector<bool> bInitKeyFrame;
        bInitKeyFrame.resize(pInitKFs[1]->nNextId, false);
        for (int i = 0; i < pInitKFs.size(); i++)
        {
            bInitKeyFrame[pInitKFs[i]->mnId] = true;
        }

        for (list<MapPoint *>::iterator lit = lMapPoints.begin(), lend = lMapPoints.end(); lit != lend; lit++)
        {
            if ((*lit))
            {
                map<KeyFrame *, size_t> observations = (*lit)->GetObservations();
                for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
                {
                    KeyFrame *pKFi = mit->first;

                    if (!bFixedKeyFrame[pKFi->mnId] && !bInitKeyFrame[pKFi->mnId])
                    {
                        if (!pKFi->isBad())
                        {
                            lFixedCameras.push_back(pKFi);
                        }
                    }
                }
            }
        }

        // Setup optimizer
        g2o::SparseOptimizer optimizer;
        g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

        linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

        g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        optimizer.setAlgorithm(solver);

        unsigned long maxKFid = 0;

        // Set KeyFrame vertices
        for (int i = 0; i < 2; i++)
        {
            KeyFrame *pKF = pInitKFs[i];
            g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
            vSE3->setEstimate(Converter::toSE3Quat(pKF->GetPose()));
            vSE3->setId(pKF->mnId);

            if (bPrintDebug)
                cout << "InitBA: Add pose vertex id: " << pKF->mnId << endl;

            if (pInitKFs[0]->GetPrevKeyFrame() == NULL)
            {
                vSE3->setFixed(true);
            }
            else
            {
                nKFs++;
                vSE3->setFixed(false);
            }

            optimizer.addVertex(vSE3);

            if (pKF->mnId > maxKFid)
                maxKFid = pKF->mnId;
        }

        // Set pose pose contraint on KeyFrames
        const float thHuberPose = sqrt(12.5916); // 95 %

        g2o::EdgeSE3PosePose *eSE3 = new g2o::EdgeSE3PosePose();

        // Set pose-pose vetices
        eSE3->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pInitKFs[0]->mnId)));
        eSE3->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pInitKFs[1]->mnId)));

        if (bPrintDebug)
            cout << "InitBA: Add pose-pose constraint between vertices: (" << pInitKFs[0]->mnId << ", " << pInitKFs[1]->mnId << ")" << endl;

        g2o::SE3Quat T21 = Converter::toSE3Quat(Odo.GetTransformation());
        eSE3->setMeasurement(T21);
        //eSE3->setInformation(pNextKF->mOdometry.GetCovariance().inverse());
        Eigen::MatrixXd informationMatrix = Odo.GetCovariance().inverse();
        eSE3->setInformation(informationMatrix); //100 * Eigen::MatrixXd::Identity(6, 6));

        // Add to graph
        optimizer.addEdge(eSE3);

        if (bPrintDebug)
            cout << "InitBA: Adding fixed pose vertex id: ";

        for (list<KeyFrame *>::iterator lit = lFixedCameras.begin(), lend = lFixedCameras.end(); lit != lend; lit++)
        {
            KeyFrame *pKFi = *lit;
            g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
            vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
            vSE3->setId(pKFi->mnId);

            if (bPrintDebug)
                cout << pKFi->mnId << ", ";

            vSE3->setFixed(true);
            optimizer.addVertex(vSE3);
            if (pKFi->mnId > maxKFid)
                maxKFid = pKFi->mnId;
        }
        if (bPrintDebug)
            cout << "done!" << endl;

        // If other camera is initialized, add pose-pose contraint
        if (pInitKFs[0]->GetPrevKeyFrame() != NULL)
        {
            KeyFrame *pKF = pInitKFs[0]->GetPrevKeyFrame();
            g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
            vSE3->setEstimate(Converter::toSE3Quat(pKF->GetPose()));
            vSE3->setId(pKF->mnId);

            if (bPrintDebug)
                cout << "InitBA: Add fixed pose vertex id: " << pKF->mnId << endl;

            vSE3->setFixed(true);
            optimizer.addVertex(vSE3);

            if (pKF->mnId > maxKFid)
                maxKFid = pKF->mnId;

            g2o::EdgeSE3PosePose *eSE3 = new g2o::EdgeSE3PosePose();

            // Set pose-pose vetices
            nKFs += 2;
            eSE3->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKF->mnId)));
            eSE3->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pInitKFs[0]->mnId)));

            if (bPrintDebug)
                cout << "InitBA: Add pose-pose constraint between verteces: (" << pKF->mnId << ", " << pInitKFs[0]->mnId << ")" << endl;

            g2o::SE3Quat T21 = Converter::toSE3Quat(pInitKFs[0]->mOdometry.GetTransformation());
            eSE3->setMeasurement(T21);

            if (bPrintDebug)
            {
                g2o::SE3Quat T1 = Converter::toSE3Quat(pKF->GetPose());
                g2o::SE3Quat T2 = Converter::toSE3Quat(pInitKFs[0]->GetPose());
                g2o::SE3Quat deltaT = T2 * T1.inverse();
            }

            //eSE3->setInformation(pNextKF->mOdometry.GetCovariance().inverse());
            Eigen::MatrixXd informationMatrix = pInitKFs[0]->mOdometry.GetCovariance().inverse();
            eSE3->setInformation(informationMatrix); //100 * Eigen::MatrixXd::Identity(6, 6));

            // Add to graph
            optimizer.addEdge(eSE3);
        }

        // Set MapPoint vertices
        const int nExpectedSize = 2 * lMapPoints.size();

        vector<g2o::EdgeSE3ProjectXYZ *> vpEdgesMono;
        vpEdgesMono.reserve(nExpectedSize);

        vector<KeyFrame *> vpEdgeKFMono;
        vpEdgeKFMono.reserve(nExpectedSize);

        vector<MapPoint *> vpMapPointEdgeMono;
        vpMapPointEdgeMono.reserve(nExpectedSize);

        vector<g2o::EdgeStereoSE3ProjectXYZ *> vpEdgesStereo;
        vpEdgesStereo.reserve(nExpectedSize);

        vector<KeyFrame *> vpEdgeKFStereo;
        vpEdgeKFStereo.reserve(nExpectedSize);

        vector<MapPoint *> vpMapPointEdgeStereo;
        vpMapPointEdgeStereo.reserve(nExpectedSize);

        const float thHuberMono = sqrt(5.991);
        const float thHuberStereo = sqrt(7.815);

        for (list<MapPoint *>::iterator lit = lMapPoints.begin(), lend = lMapPoints.end(); lit != lend; lit++)
        {
            MapPoint *pMP = *lit;
            g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
            Eigen::Vector3d worldPos = Converter::toVector3d(pMP->GetWorldPos());
            vPoint->setEstimate(worldPos);
            int id = pMP->mnId + maxKFid + 1;
            vPoint->setId(id);
            vPoint->setMarginalized(true);
            optimizer.addVertex(vPoint);

            const map<KeyFrame *, size_t> observations = pMP->GetObservations();

            //Set edges
            for (map<KeyFrame *, size_t>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
            {
                KeyFrame *pKFi = mit->first;

                if (!pKFi->isBad())
                {
                    const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];

                    // Monocular observation
                    if (pKFi->mvuRight[mit->second] < 0)
                    {
                        Eigen::Matrix<double, 2, 1> obs;
                        obs << kpUn.pt.x, kpUn.pt.y;

                        g2o::EdgeSE3ProjectXYZ *e = new g2o::EdgeSE3ProjectXYZ();

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
                        e->setMeasurement(obs);
                        const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                        e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberMono);

                        e->fx = pKFi->fx;
                        e->fy = pKFi->fy;
                        e->cx = pKFi->cx;
                        e->cy = pKFi->cy;

                        // Set extrinsics
                        e->Tcb = Converter::toSE3Quat(pKFi->mTcb);

                        optimizer.addEdge(e);
                        vpEdgesMono.push_back(e);
                        vpEdgeKFMono.push_back(pKFi);
                        vpMapPointEdgeMono.push_back(pMP);
                    }
                    else // Stereo observation
                    {
                        Eigen::Matrix<double, 3, 1> obs;
                        const float kp_ur = pKFi->mvuRight[mit->second];
                        obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                        g2o::EdgeStereoSE3ProjectXYZ *e = new g2o::EdgeStereoSE3ProjectXYZ();

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
                        e->setMeasurement(obs);
                        const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                        Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
                        e->setInformation(Info);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberStereo);

                        e->fx = pKFi->fx;
                        e->fy = pKFi->fy;
                        e->cx = pKFi->cx;
                        e->cy = pKFi->cy;
                        e->bf = pKFi->mbf;

                        // Set extrinsics
                        e->Tcb = Converter::toSE3Quat(pKFi->mTcb);

                        optimizer.addEdge(e);
                        vpEdgesStereo.push_back(e);
                        vpEdgeKFStereo.push_back(pKFi);
                        vpMapPointEdgeStereo.push_back(pMP);
                    }
                }
            }
        }
        if (nKFs == 0)
        {
            cout << "InitBundle: No edges added, aborting optimization..." << endl;
            return;
        }
        if (bPrintDebug)
            cout << "InitBundle: Start first opt..." << endl;
        if (bPrintDebug)
            optimizer.setVerbose(true);
        optimizer.initializeOptimization();
        optimizer.optimize(10);
        if (bPrintDebug)
            cout << "InitBundle: Finished first opt..." << endl;

        // Check inlier observations
        int nOutliers = 0;
        for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
        {
            g2o::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
            MapPoint *pMP = vpMapPointEdgeMono[i];

            if (pMP->isBad())
                continue;

            if (e->chi2() > 5.991 || !e->isDepthPositive())
            {
                e->setLevel(1);
                nOutliers++;
            }

            e->setRobustKernel(0);
        }

        for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
        {
            g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
            MapPoint *pMP = vpMapPointEdgeStereo[i];

            if (pMP->isBad())
                continue;

            if (e->chi2() > 7.815 || !e->isDepthPositive())
            {
                e->setLevel(1);
            }

            e->setRobustKernel(0);
        }

        // Optimize again without the outliers

        if (bPrintDebug)
            cout << "InitBundle: Start second opt (" << nOutliers << " points removed)..." << endl;

        if (bPrintDebug)
            optimizer.setVerbose(true);
        optimizer.initializeOptimization(0);
        optimizer.optimize(5);
        if (bPrintDebug)
            cout << "InitBundle: Second opt finished..." << endl;

        vector<pair<KeyFrame *, MapPoint *>> vToErase;
        vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());

        // Check inlier observations
        for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
        {
            g2o::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
            MapPoint *pMP = vpMapPointEdgeMono[i];

            if (pMP->isBad())
                continue;

            if (e->chi2() > 5.991 || !e->isDepthPositive())
            {
                KeyFrame *pKFi = vpEdgeKFMono[i];
                vToErase.push_back(make_pair(pKFi, pMP));
            }
        }

        for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
        {
            g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
            MapPoint *pMP = vpMapPointEdgeStereo[i];

            if (pMP->isBad())
                continue;

            if (e->chi2() > 7.815 || !e->isDepthPositive())
            {
                KeyFrame *pKFi = vpEdgeKFStereo[i];
                vToErase.push_back(make_pair(pKFi, pMP));
            }
        }

        // Get Map Mutex
        unique_lock<recursive_mutex> lock(pMap->mMutexMapUpdate);

        if (!vToErase.empty())
        {
            for (size_t i = 0; i < vToErase.size(); i++)
            {
                KeyFrame *pKFi = vToErase[i].first;
                MapPoint *pMPi = vToErase[i].second;
                pKFi->EraseMapPointMatch(pMPi);
                pMPi->EraseObservation(pKFi);
            }
        }

        // Recover optimized data

        //Keyframes
        for (int i = 0; i < 2; i++)
        {
            KeyFrame *pKF = pInitKFs[i];
            g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pKF->mnId));
            g2o::SE3Quat SE3quat = vSE3->estimate();
            pKF->SetPose(Converter::toCvMat(SE3quat));
        }

        //Points
        for (list<MapPoint *>::iterator lit = lMapPoints.begin(), lend = lMapPoints.end(); lit != lend; lit++)
        {
            MapPoint *pMP = *lit;
            g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(pMP->mnId + maxKFid + 1));
            pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
            pMP->UpdateNormalAndDepth();
        }
    }

        int Optimizer::OptimizeSim3(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches1, g2o::Sim3 &g2oS12, const float th2, const bool bFixScale)
    {
        g2o::SparseOptimizer optimizer;
        g2o::BlockSolverX::LinearSolverType *linearSolver;

        linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

        g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        optimizer.setAlgorithm(solver);

        // Calibration
        const cv::Mat &K1 = pKF1->mK;
        const cv::Mat &K2 = pKF2->mK;

        // Camera poses
        const cv::Mat R1w = pKF1->GetCameraRotation();
        const cv::Mat t1w = pKF1->GetCameraTranslation();
        const cv::Mat R2w = pKF2->GetCameraRotation();
        const cv::Mat t2w = pKF2->GetCameraTranslation();

        // Set Sim3 vertex
        g2o::VertexSim3Expmap *vSim3 = new g2o::VertexSim3Expmap();
        vSim3->_fix_scale = bFixScale;
        vSim3->setEstimate(g2oS12);
        vSim3->setId(0);
        vSim3->setFixed(false);
        vSim3->_principle_point1[0] = K1.at<float>(0, 2);
        vSim3->_principle_point1[1] = K1.at<float>(1, 2);
        vSim3->_focal_length1[0] = K1.at<float>(0, 0);
        vSim3->_focal_length1[1] = K1.at<float>(1, 1);
        vSim3->_principle_point2[0] = K2.at<float>(0, 2);
        vSim3->_principle_point2[1] = K2.at<float>(1, 2);
        vSim3->_focal_length2[0] = K2.at<float>(0, 0);
        vSim3->_focal_length2[1] = K2.at<float>(1, 1);
        optimizer.addVertex(vSim3);

        // Set MapPoint vertices
        const int N = vpMatches1.size();
        const vector<MapPoint *> vpMapPoints1 = pKF1->GetMapPointMatches();
        vector<g2o::EdgeSim3ProjectXYZ *> vpEdges12;
        vector<g2o::EdgeInverseSim3ProjectXYZ *> vpEdges21;
        vector<size_t> vnIndexEdge;

        vnIndexEdge.reserve(2 * N);
        vpEdges12.reserve(2 * N);
        vpEdges21.reserve(2 * N);

        const float deltaHuber = sqrt(th2);

        int nCorrespondences = 0;

        for (int i = 0; i < N; i++)
        {
            if (!vpMatches1[i])
                continue;

            MapPoint *pMP1 = vpMapPoints1[i];
            MapPoint *pMP2 = vpMatches1[i];

            const int id1 = 2 * i + 1;
            const int id2 = 2 * (i + 1);

            const int i2 = pMP2->GetIndexInKeyFrame(pKF2);

            if (pMP1 && pMP2)
            {
                if (!pMP1->isBad() && !pMP2->isBad() && i2 >= 0)
                {
                    g2o::VertexSBAPointXYZ *vPoint1 = new g2o::VertexSBAPointXYZ();
                    cv::Mat P3D1w = pMP1->GetWorldPos();
                    cv::Mat P3D1c = R1w * P3D1w + t1w;
                    vPoint1->setEstimate(Converter::toVector3d(P3D1c));
                    vPoint1->setId(id1);
                    vPoint1->setFixed(true);
                    optimizer.addVertex(vPoint1);

                    g2o::VertexSBAPointXYZ *vPoint2 = new g2o::VertexSBAPointXYZ();
                    cv::Mat P3D2w = pMP2->GetWorldPos();
                    cv::Mat P3D2c = R2w * P3D2w + t2w;
                    vPoint2->setEstimate(Converter::toVector3d(P3D2c));
                    vPoint2->setId(id2);
                    vPoint2->setFixed(true);
                    optimizer.addVertex(vPoint2);
                }
                else
                    continue;
            }
            else
                continue;

            nCorrespondences++;

            // Set edge x1 = S12*X2
            Eigen::Matrix<double, 2, 1> obs1;
            const cv::KeyPoint &kpUn1 = pKF1->mvKeysUn[i];
            obs1 << kpUn1.pt.x, kpUn1.pt.y;

            g2o::EdgeSim3ProjectXYZ *e12 = new g2o::EdgeSim3ProjectXYZ();
            e12->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id2)));
            e12->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
            e12->setMeasurement(obs1);
            const float &invSigmaSquare1 = pKF1->mvInvLevelSigma2[kpUn1.octave];
            e12->setInformation(Eigen::Matrix2d::Identity() * invSigmaSquare1);

            g2o::RobustKernelHuber *rk1 = new g2o::RobustKernelHuber;
            e12->setRobustKernel(rk1);
            rk1->setDelta(deltaHuber);
            optimizer.addEdge(e12);

            // Set edge x2 = S21*X1
            Eigen::Matrix<double, 2, 1> obs2;
            const cv::KeyPoint &kpUn2 = pKF2->mvKeysUn[i2];
            obs2 << kpUn2.pt.x, kpUn2.pt.y;

            g2o::EdgeInverseSim3ProjectXYZ *e21 = new g2o::EdgeInverseSim3ProjectXYZ();

            e21->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id1)));
            e21->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
            e21->setMeasurement(obs2);
            float invSigmaSquare2 = pKF2->mvInvLevelSigma2[kpUn2.octave];
            e21->setInformation(Eigen::Matrix2d::Identity() * invSigmaSquare2);

            g2o::RobustKernelHuber *rk2 = new g2o::RobustKernelHuber;
            e21->setRobustKernel(rk2);
            rk2->setDelta(deltaHuber);
            optimizer.addEdge(e21);

            vpEdges12.push_back(e12);
            vpEdges21.push_back(e21);
            vnIndexEdge.push_back(i);
        }

        // Optimize!
        optimizer.initializeOptimization();
        optimizer.optimize(5);

        // Check inliers
        int nBad = 0;
        for (size_t i = 0; i < vpEdges12.size(); i++)
        {
            g2o::EdgeSim3ProjectXYZ *e12 = vpEdges12[i];
            g2o::EdgeInverseSim3ProjectXYZ *e21 = vpEdges21[i];
            if (!e12 || !e21)
                continue;

            if (e12->chi2() > th2 || e21->chi2() > th2)
            {
                size_t idx = vnIndexEdge[i];
                vpMatches1[idx] = static_cast<MapPoint *>(NULL);
                optimizer.removeEdge(e12);
                optimizer.removeEdge(e21);
                vpEdges12[i] = static_cast<g2o::EdgeSim3ProjectXYZ *>(NULL);
                vpEdges21[i] = static_cast<g2o::EdgeInverseSim3ProjectXYZ *>(NULL);
                nBad++;
            }
        }

        int nMoreIterations;
        if (nBad > 0)
            nMoreIterations = 10;
        else
            nMoreIterations = 5;

        if (nCorrespondences - nBad < 10)
            return 0;

        // Optimize again only with inliers

        optimizer.initializeOptimization();
        optimizer.optimize(nMoreIterations);

        int nIn = 0;
        for (size_t i = 0; i < vpEdges12.size(); i++)
        {
            g2o::EdgeSim3ProjectXYZ *e12 = vpEdges12[i];
            g2o::EdgeInverseSim3ProjectXYZ *e21 = vpEdges21[i];
            if (!e12 || !e21)
                continue;

            if (e12->chi2() > th2 || e21->chi2() > th2)
            {
                size_t idx = vnIndexEdge[i];
                vpMatches1[idx] = static_cast<MapPoint *>(NULL);
            }
            else
                nIn++;
        }

        // Recover optimized Sim3
        g2o::VertexSim3Expmap *vSim3_recov = static_cast<g2o::VertexSim3Expmap *>(optimizer.vertex(0));
        g2oS12 = vSim3_recov->estimate();

        return nIn;
    }

    void Optimizer::SetKeyFrameVertices(g2o::SparseOptimizer &optimizer, const vector<KeyFrame *> &vpKFs, bool bFixFirstKF, unsigned long &maxKFid)
    {
        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame *pKF = vpKFs[i];
            if (pKF->isBad())
                continue;
            g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
            vSE3->setEstimate(Converter::toSE3Quat(pKF->GetPose()));
            vSE3->setId(pKF->mnId);
            vSE3->setFixed(pKF->mnId == 0 && bFixFirstKF);
            optimizer.addVertex(vSE3);
            if (pKF->mnId > maxKFid)
                maxKFid = pKF->mnId;
        }
    }

    void Optimizer::SetPosePoseConstraints(g2o::SparseOptimizer &optimizer, const vector<KeyFrame *> &vpKFs)
    {
        const float thHuberPose = sqrt(12.5916); // 95 %
        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame *pKFi = vpKFs[i];
            g2o::EdgeSE3PosePose *eSE3 = new g2o::EdgeSE3PosePose();

            if (pKFi->GetNextKeyFrame() != NULL)
            {
                KeyFrame *pKFnext = pKFi->GetNextKeyFrame();

                // Set vetices
                eSE3->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
                eSE3->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFnext->mnId)));

                //cout << "BA: Add pose-pose constraint between vertices: (" << pKFi->mnId << ", " << pKFnext->mnId << ")" << endl;

                // Set odometry constraint and information
                g2o::SE3Quat T21 = Converter::toSE3Quat(pKFnext->mOdometry.GetTransformation());
                eSE3->setMeasurement(T21);
                //eSE3->setInformation(pNextKF->mOdometry.GetCovariance().inverse());
                Eigen::MatrixXd informationMatrix = pKFnext->mOdometry.GetCovariance().inverse();
                eSE3->setInformation(informationMatrix); //100 * Eigen::MatrixXd::Identity(6, 6));

                // Set cost function
                g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                eSE3->setRobustKernel(rk);
                rk->setDelta(thHuberPose);

                // Check that vertex is healthy
                eSE3->computeError();

                g2o::Vector3d rho;
                eSE3->robustKernel()->robustify(eSE3->chi2(), rho);

                // Add to graph
                optimizer.addEdge(eSE3);
            }
        }
    }

    void Optimizer::SetMapPointVertices(g2o::SparseOptimizer &optimizer, const vector<MapPoint *> &vpLocalMP, vector<bool> &vbNotIncludedLocalMP,
                                        const vector<MapPoint *> &vpFixedMP, cv::Mat Tlf, unsigned long &maxKFid)
    {
        const float thHuber2D = sqrt(5.99);
        const float thHuber3D = sqrt(7.815);
        const float thHuberFixed = sqrt(7.815);

        bool bRobust = true;

        // Set local MapPoint vertices
        for (size_t i = 0; i < vpLocalMP.size(); i++)
        {
            MapPoint *pMP = vpLocalMP[i];
            if (pMP->isBad())
                continue;
            g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
            vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
            const int id = pMP->mnId + maxKFid + 1;
            vPoint->setId(id);
            vPoint->setMarginalized(true);
            optimizer.addVertex(vPoint);

            const map<KeyFrame *, size_t> observations = pMP->GetObservations();

            int nEdges = 0;
            //SET EDGES
            for (map<KeyFrame *, size_t>::const_iterator mit = observations.begin(); mit != observations.end(); mit++)
            {

                KeyFrame *pKF = mit->first;
                if (pKF->isBad() || pKF->mnId > maxKFid)
                    continue;

                nEdges++;

                const cv::KeyPoint &kpUn = pKF->mvKeysUn[mit->second];

                if (pKF->mvuRight[mit->second] < 0)
                {
                    Eigen::Matrix<double, 2, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    g2o::EdgeSE3ProjectXYZ *e = new g2o::EdgeSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKF->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    if (bRobust)
                    {
                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuber2D);
                    }

                    // Set intrinsics
                    e->fx = pKF->fx;
                    e->fy = pKF->fy;
                    e->cx = pKF->cx;
                    e->cy = pKF->cy;

                    // Set extrinsics
                    e->Tcb = Converter::toSE3Quat(pKF->mTcb);

                    // Check that vertex is healthy
                    e->computeError();

                    g2o::Vector3d rho;
                    e->robustKernel()->robustify(e->chi2(), rho);

                    optimizer.addEdge(e);
                }
                else
                {
                    Eigen::Matrix<double, 3, 1> obs;
                    const float kp_ur = pKF->mvuRight[mit->second];
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    g2o::EdgeStereoSE3ProjectXYZ *e = new g2o::EdgeStereoSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKF->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
                    e->setInformation(Info);

                    if (bRobust)
                    {
                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuber3D);
                    }

                    e->fx = pKF->fx;
                    e->fy = pKF->fy;
                    e->cx = pKF->cx;
                    e->cy = pKF->cy;
                    e->bf = pKF->mbf;

                    // Set extrinsics
                    e->Tcb = Converter::toSE3Quat(pKF->mTcb);

                    // Check that vertex is healthy
                    e->computeError();
                    g2o::Vector3d rho;
                    e->robustKernel()->robustify(e->chi2(), rho);

                    optimizer.addEdge(e);
                }
            }

            if (nEdges == 0)
            {
                optimizer.removeVertex(vPoint);
                vbNotIncludedLocalMP[i] = true;
            }
            else
            {
                vbNotIncludedLocalMP[i] = false;
            }
        }

        // Set Fixed map transformation vertex
        unsigned long int nGlobalMapTransID = maxKFid + 1;
        if (!Tlf.empty())
        {
            g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
            vSE3->setEstimate(Converter::toSE3Quat(Tlf));
            vSE3->setId(nGlobalMapTransID);
            vSE3->setFixed(true);

            optimizer.addVertex(vSE3);
        }

        // Set fixed MapPoint vertices
        for (size_t i = 0; i < vpFixedMP.size(); i++)
        {
            MapPoint *pMP = vpFixedMP[i];

            if (pMP)
                if (!pMP->isBad())
                {
                    KeyFrame *pKFi = pMP->GetReferenceKeyFrame();
                    int idx = pMP->GetIndexInKeyFrame(pKFi);

                    Eigen::Vector2d obs = pKFi->mFixedMapMatches.vMatches[idx].imagePoint;

                    g2o::EdgeSE3ProjectXYZFixed *e = new g2o::EdgeSE3ProjectXYZFixed();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nGlobalMapTransID)));

                    e->setMeasurement(obs);
                    e->setInformation(pKFi->mFixedMapMatches.informationMatrix);

                    g2o::RobustKernelCauchy *rk = new g2o::RobustKernelCauchy;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberFixed);

                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;

                    // Set extrinsics
                    e->Tcb = Converter::toSE3Quat(pKFi->mTcb);

                    // Set fixed map point
                    Eigen::Vector3d point3D;
                    point3D = Converter::toVector3d(pMP->GetWorldPos());
                    e->point3d = point3D;

                    // Check that vertex is healthy
                    e->computeError();
                    g2o::Vector3d rho;
                    e->robustKernel()->robustify(e->chi2(), rho);

                    optimizer.addEdge(e);
                }
        }
    }

    // Mono map point vetices
    void Optimizer::SetMapPointVertices(g2o::SparseOptimizer &optimizer, list<MapPoint *> &lMapPoints, bool bFixedPoints,
                                        vector<g2o::EdgeSE3ProjectXYZ *> &vpEdges, vector<KeyFrame *> &vpEdgeKF, vector<MapPoint *> &vpMapPointEdge,
                                        unsigned long &maxKFid, const float thHuberMono)
    {
        //const float thHuberMono = sqrt(5.991);

        for (list<MapPoint *>::iterator lit = lMapPoints.begin(), lend = lMapPoints.end(); lit != lend; lit++)
        {
            MapPoint *pMP = *lit;
            g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
            vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
            int id = pMP->mnId + maxKFid + 1;
            vPoint->setId(id);
            vPoint->setMarginalized(true);
            optimizer.addVertex(vPoint);

            const map<KeyFrame *, size_t> observations = pMP->GetObservations();

            //Set edges
            for (map<KeyFrame *, size_t>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
            {
                KeyFrame *pKFi = mit->first;

                if (!pKFi->isBad())
                {
                    const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];

                    // Monocular observation
                    if (pKFi->mvuRight[mit->second] < 0)
                    {
                        Eigen::Matrix<double, 2, 1> obs;
                        obs << kpUn.pt.x, kpUn.pt.y;

                        g2o::EdgeSE3ProjectXYZ *e = new g2o::EdgeSE3ProjectXYZ();

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
                        e->setMeasurement(obs);
                        const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                        e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberMono);

                        e->fx = pKFi->fx;
                        e->fy = pKFi->fy;
                        e->cx = pKFi->cx;
                        e->cy = pKFi->cy;

                        // Set extrinsics
                        e->Tcb = Converter::toSE3Quat(pKFi->mTcb);

                        // Check that vertex is healthy
                        e->computeError();
                        g2o::Vector3d rho;
                        e->robustKernel()->robustify(e->chi2(), rho);

                        optimizer.addEdge(e);
                        vpEdges.push_back(e);
                        vpEdgeKF.push_back(pKFi);
                        vpMapPointEdge.push_back(pMP);
                    }
                }
            }
        }
    }

    // Mono map point vetices
    void Optimizer::SetMapPointVertices(g2o::SparseOptimizer &optimizer, list<MapPoint *> &lMapPoints, bool bFixedPoints,
                                        vector<g2o::EdgeSE3ProjectXYZCalib *> &vpEdges, vector<KeyFrame *> &vpEdgeKF, vector<MapPoint *> &vpMapPointEdge,
                                        unsigned long &maxKFid, const float thHuberMono, vector<int> calibVertices)
    {

        for (list<MapPoint *>::iterator lit = lMapPoints.begin(), lend = lMapPoints.end(); lit != lend; lit++)
        {
            MapPoint *pMP = *lit;
            g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
            vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
            int id = pMP->mnId + maxKFid + 1;
            vPoint->setId(id);
            vPoint->setMarginalized(true);
            optimizer.addVertex(vPoint);

            const map<KeyFrame *, size_t> observations = pMP->GetObservations();

            //Set edges
            for (map<KeyFrame *, size_t>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
            {
                KeyFrame *pKFi = mit->first;

                if (!pKFi->isBad())
                {
                    const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];

                    // Monocular observation
                    if (pKFi->mvuRight[mit->second] < 0)
                    {
                        Eigen::Matrix<double, 2, 1> obs;
                        obs << kpUn.pt.x, kpUn.pt.y;

                        g2o::EdgeSE3ProjectXYZCalib *e = new g2o::EdgeSE3ProjectXYZCalib();

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
                        e->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(calibVertices[pKFi->mCameraID])));

                        e->setMeasurement(obs);
                        const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                        e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberMono);

                        e->fx = pKFi->fx;
                        e->fy = pKFi->fy;
                        e->cx = pKFi->cx;
                        e->cy = pKFi->cy;

                        // Set extrinsics
                        e->Tcb = Converter::toSE3Quat(pKFi->mTcb);

                        // Check that vertex is healthy
                        e->computeError();
                        g2o::Vector3d rho;
                        e->robustKernel()->robustify(e->chi2(), rho);

                        optimizer.addEdge(e);
                        vpEdges.push_back(e);
                        vpEdgeKF.push_back(pKFi);
                        vpMapPointEdge.push_back(pMP);
                    }
                }
            }
        }
    }

    // Stereo map point vetices
    void Optimizer::SetMapPointVertices(g2o::SparseOptimizer &optimizer, list<MapPoint *> &lMapPoints, bool bFixedPoints,
                                        vector<g2o::EdgeStereoSE3ProjectXYZ *> &vpEdges, vector<KeyFrame *> &vpEdgeKF, vector<MapPoint *> &vpMapPointEdge,
                                        unsigned long &maxKFid, const float thHuberStereo)
    {
        //const float thHuberStereo = sqrt(7.815);

        for (list<MapPoint *>::iterator lit = lMapPoints.begin(), lend = lMapPoints.end(); lit != lend; lit++)
        {
            MapPoint *pMP = *lit;
            g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
            vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
            int id = pMP->mnId + maxKFid + 1;
            vPoint->setId(id);
            vPoint->setMarginalized(true);
            optimizer.addVertex(vPoint);

            const map<KeyFrame *, size_t> observations = pMP->GetObservations();

            //Set edges
            for (map<KeyFrame *, size_t>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
            {
                KeyFrame *pKFi = mit->first;

                if (!pKFi->isBad())
                {
                    const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];

                    // Stereo observation
                    if (pKFi->mvuRight[mit->second] >= 0)
                    {
                        Eigen::Matrix<double, 3, 1> obs;
                        const float kp_ur = pKFi->mvuRight[mit->second];
                        obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                        g2o::EdgeStereoSE3ProjectXYZ *e = new g2o::EdgeStereoSE3ProjectXYZ();

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
                        e->setMeasurement(obs);
                        const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                        Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
                        e->setInformation(Info);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberStereo);

                        e->fx = pKFi->fx;
                        e->fy = pKFi->fy;
                        e->cx = pKFi->cx;
                        e->cy = pKFi->cy;
                        e->bf = pKFi->mbf;

                        // Set extrinsics
                        e->Tcb = Converter::toSE3Quat(pKFi->mTcb);

                        // Check that vertex is healthy
                        e->computeError();
                        g2o::Vector3d rho;
                        e->robustKernel()->robustify(e->chi2(), rho);

                        optimizer.addEdge(e);
                        vpEdges.push_back(e);
                        vpEdgeKF.push_back(pKFi);
                        vpMapPointEdge.push_back(pMP);
                    }
                }
            }
        }
    }

    void Optimizer::SetMapPointVertices(g2o::SparseOptimizer &optimizer, list<KeyFrame *> &lKeyFrames,
                                        vector<g2o::EdgeSE3ProjectXYZFixed *> &vpEdges, vector<KeyFrame *> &vpEdgeKF, vector<MapPoint *> &vpMapPointEdge,
                                        unsigned long &nGlobalMapTransID, float thHuberFixed, eRobustCostFunction costFunction, bool bAddOutlierCandidates)
    {
        //float thHuberFixed = sqrt(7.815);

        for (list<KeyFrame *>::iterator lit = lKeyFrames.begin(), lend = lKeyFrames.end(); lit != lend; lit++)
        {
            bool bPrintDebug = false;

            KeyFrame *pKFi = *lit;

            //if (bAddOutlierCandidates || !pKFi->isOutlier())
            //{
            vector<MapPoint *> vpMPs = pKFi->GetFixedMapPointMatches();

            if (!vpMPs.empty())
                if (bPrintDebug)
                    cout << pKFi->mnId << ", ";

            for (vector<MapPoint *>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; vit++)
            {
                MapPoint *pMP = *vit;
                if (pMP)
                    if (!pMP->isBad())
                    {
                        int idx = pMP->GetIndexInKeyFrame(pKFi);

                        Eigen::Vector2d obs = pKFi->mFixedMapMatches.vMatches[idx].imagePoint;

                        g2o::EdgeSE3ProjectXYZFixed *e = new g2o::EdgeSE3ProjectXYZFixed();

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nGlobalMapTransID)));

                        e->setMeasurement(obs);
                        //if (!pKFi->isOutlier())
                        e->setInformation(pKFi->mFixedMapMatches.informationMatrix);
                        //else
                        //    e->setInformation(pow(0.1, 2.0) * pKFi->mFixedMapMatches.informationMatrix);

                        if (!pKFi->isOutlier() || bAddOutlierCandidates)
                        {
                            if (costFunction == TUKEY && pKFi->isOutlier())
                            {
                                g2o::RobustKernelTukey *rk = new g2o::RobustKernelTukey;
                                e->setRobustKernel(rk);
                                double deltaSqr = pow(thHuberFixed, 2.0);
                                rk->setDeltaSqr(deltaSqr, 1.0 / deltaSqr);
                            }
                            else if (costFunction == CAUCHY || (costFunction == TUKEY && !pKFi->isOutlier()))
                            {
                                g2o::RobustKernelCauchy *rk = new g2o::RobustKernelCauchy;
                                e->setRobustKernel(rk);
                                rk->setDelta(thHuberFixed);
                            }
                            else if (costFunction == HUBER)
                            {
                                g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                                e->setRobustKernel(rk);
                                rk->setDelta(thHuberFixed);
                            }
                        }

                        e->fx = pKFi->fx;
                        e->fy = pKFi->fy;
                        e->cx = pKFi->cx;
                        e->cy = pKFi->cy;

                        // Set extrinsics
                        e->Tcb = Converter::toSE3Quat(pKFi->mTcb);

                        // Set fixed map point
                        Eigen::Vector3d point3D;
                        point3D = Converter::toVector3d(pMP->GetWorldPos());
                        e->point3d = point3D;

                        // Check that vertex is healthy
                        e->computeError();
                        g2o::Vector3d rho;
                        e->robustKernel()->robustify(e->chi2(), rho);

                        optimizer.addEdge(e);

                        vpEdges.push_back(e);
                        vpEdgeKF.push_back(pKFi);
                        vpMapPointEdge.push_back(pMP);
                    }
            }
            if (bPrintDebug)
                cout << "done!" << endl;
            //}
        }
    }

} // namespace LocSLAM
