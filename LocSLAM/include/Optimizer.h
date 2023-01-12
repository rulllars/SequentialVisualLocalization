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

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "Frame.h"
#include "FixedMapLocalization.h"

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace LocSLAM
{

    class Optimizer
    {
    public:
        void static BundleAdjustment(const std::vector<KeyFrame *> &vpLocalKF, const std::vector<MapPoint *> &vpLocalMP,
                                     const std::vector<KeyFrame *> &vpFixedKF, const std::vector<MapPoint *> &vpFixedMP,
                                     cv::Mat Tlf, bool bFixFirstKF,
                                     int nIterations = 5, bool *pbStopFlag = NULL, const unsigned long nLoopKF = 0,
                                     const bool bRobust = true);
        void static GlobalBundleAdjustemnt(Map *pLocalMap, Map *pFixedMap, bool bFixFirstKF = true, int nIterations = 5, bool *pbStopFlag = NULL,
                                           const unsigned long nLoopKF = 0, const bool bRobust = true);
        void static LocalBundleAdjustment(KeyFrame *pKF, bool *pbStopFlag, Map *pMap, FixedMapLocalization *pFixedMapLicalizer, bool bDoWindowedFixedMapBundle = false, bool bDoCameraCalibration = false);

        void static InitialBundleAdjustment(vector<KeyFrame *> pInitKFs, OdometryData &Odo, Map *pMap);

        int static PoseOptimization(Frame *pFrame, bool bUsePrior = false);

        // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono)
        static int OptimizeSim3(KeyFrame *pKF1, KeyFrame *pKF2, std::vector<MapPoint *> &vpMatches1,
                                g2o::Sim3 &g2oS12, const float th2, const bool bFixScale);

    private:
        enum eRobustCostFunction
        {
            TUKEY = 0,
            CAUCHY = 1,
            HUBER = 2,
            PSEUDOHUBER = 3
        };
        void static SetKeyFrameVertices(g2o::SparseOptimizer &optimizer, const vector<KeyFrame *> &vpKFs,
                                        bool bFixFirstKF, unsigned long &maxKFid);

        void static SetPosePoseConstraints(g2o::SparseOptimizer &optimizer, const vector<KeyFrame *> &vpKFs);

        // Global bundle
        void static SetMapPointVertices(g2o::SparseOptimizer &optimizer, const vector<MapPoint *> &vpLocalMP, vector<bool> &vbNotIncludedLocalMP,
                                        const vector<MapPoint *> &vpFixedMP, cv::Mat Tlf, unsigned long &maxKFid);

        // Mono camera points
        void static SetMapPointVertices(g2o::SparseOptimizer &optimizer, list<MapPoint *> &lMapPoints, bool bFixedPoints,
                                        vector<g2o::EdgeSE3ProjectXYZ *> &vpEdges, vector<KeyFrame *> &vpEdgeKF, vector<MapPoint *> &vpMapPointEdge,
                                        unsigned long &maxKFid, const float thHuberMono = sqrt(5.991));

        // Mono camera points with calibration
        void static SetMapPointVertices(g2o::SparseOptimizer &optimizer, list<MapPoint *> &lMapPoints, bool bFixedPoints,
                                        vector<g2o::EdgeSE3ProjectXYZCalib *> &vpEdges, vector<KeyFrame *> &vpEdgeKF, vector<MapPoint *> &vpMapPointEdge,
                                        unsigned long &maxKFid, const float thHuberMono, vector<int> calibVertices);

        // Stereo camera points with calibration
        void static SetMapPointVertices(g2o::SparseOptimizer &optimizer, list<MapPoint *> &lMapPoints, bool bFixedPoints,
                                        vector<g2o::EdgeStereoSE3ProjectXYZ *> &vpEdges, vector<KeyFrame *> &vpEdgeKF, vector<MapPoint *> &vpMapPointEdge,
                                        unsigned long &maxKFid, const float thHuberStereo = sqrt(7.815));

        // Fixed map points
        void static SetMapPointVertices(g2o::SparseOptimizer &optimizer, list<KeyFrame *> &lKeyFrames,
                                        vector<g2o::EdgeSE3ProjectXYZFixed *> &vpEdges, vector<KeyFrame *> &vpEdgeKF, vector<MapPoint *> &vpMapPointEdge,
                                        unsigned long &maxKFid, const float thHuberFixed = sqrt(7.815), const eRobustCostFunction robustCost = HUBER, const bool bAddOutlierCandidates = true);
    };
};     // namespace LocSLAM
#endif // OPTIMIZER_H
