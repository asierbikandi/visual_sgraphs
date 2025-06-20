/**
 * This file is a modified version of a file from ORB-SLAM3.
 * 
 * Modifications Copyright (C) 2023-2025 SnT, University of Luxembourg
 * Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger Voos
 * 
 * Original Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 * 
 * This file is part of vS-Graphs, which is free software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 *
 * vS-Graphs is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program.
 * If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "Map.h"
#include "Frame.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "LoopClosing.h"

#include <math.h>
#include <boost/bind.hpp>

#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/sparse_block_matrix.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"

namespace ORB_SLAM3
{
    class LoopClosing;

    class Optimizer
    {
    public:
        void static BundleAdjustment(const std::vector<KeyFrame *> &vpKF, const std::vector<MapPoint *> &vpMP,
                                     const std::vector<Marker *> &vpMarkers, const std::vector<Plane *> &vpPlanes,
                                     const std::vector<Door *> &vpDoors, const std::vector<Room *> &vpRooms,
                                     int nIterations = 5, bool *pbStopFlag = NULL, const unsigned long nLoopKF = 0,
                                     const bool bRobust = true, double markerImpact = 0.1);

        void static GlobalBundleAdjustemnt(Map *pMap, int nIterations = 5, bool *pbStopFlag = NULL,
                                           const unsigned long nLoopKF = 0, const bool bRobust = true,
                                           double markerImpact = 0.1);

        void static FullInertialBA(Map *pMap, int its, const bool bFixLocal = false, const unsigned long nLoopKF = 0,
                                   bool *pbStopFlag = NULL, bool bInit = false, float priorG = 1e2, float priorA = 1e6,
                                   Eigen::VectorXd *vSingVal = NULL, bool *bHess = NULL);

        void static LocalBundleAdjustment(KeyFrame *pKF, bool *pbStopFlag, Map *pMap, int &countFixedKF, int &num_OptKF,
                                          int &num_MPs, int &num_edges, double markerImpact = 0.1);

        /**
         * @brief Local Bundle Adjustment for loop closure detection
         *
         * @param pMainKF Main KeyFrame
         * @param vpAdjustKF Non-fixed KeyFrames to adjust
         * @param vpFixedKF Fixed KeyFrames to set
         * @param pbStopFlag Flag to forcely stop the optimization
         */
        void static LoopClosureLocalBundleAdjustment(KeyFrame *pMainKF, vector<KeyFrame *> vpAdjustKF,
                                                     vector<KeyFrame *> vpFixedKF, bool *pbStopFlag);

        int static PoseOptimization(Frame *pFrame);
        int static PoseInertialOptimizationLastKeyFrame(Frame *pFrame, bool bRecInit = false);
        int static PoseInertialOptimizationLastFrame(Frame *pFrame, bool bRecInit = false);

        // if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise (mono)
        void static OptimizeEssentialGraph(Map *pMap, KeyFrame *pLoopKF, KeyFrame *pCurKF,
                                           const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                           const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                           const map<KeyFrame *, set<KeyFrame *>> &LoopConnections,
                                           const bool &bFixScale);

        /**
         * @brief Optimize the Essential Graph when a loop closure is detected
         *
         * @param pCurKF Current KeyFrame
         * @param vpFixedKFs Fixed KeyFrames
         * @param vpFixedCorrectedKFs Corrected Fixed KeyFrames
         * @param vpNonFixedKFs Non-Fixed KeyFrames
         * @param vpNonCorrectedMPs Non-Corrected MapPoints
         * @param vpCurrentMapDoors Current Map Doors
         * @param vpCurrentMapPlanes Current Map Planes
         * @param vpCurrentMapMarkers Current Map Markers
         * @param vpCurrentDetMapRooms Current Detected Map Rooms
         * @param vpCurrentMrkMapRooms Current Marker-based Map Rooms
         * @param vpClusterPoints Cluster points of the map set by `voxblox_skeleton`
         */
        void static OptimizeEssentialGraph(KeyFrame *pCurKF, vector<KeyFrame *> &vpFixedKFs,
                                           vector<KeyFrame *> &vpFixedCorrectedKFs, vector<KeyFrame *> &vpNonFixedKFs,
                                           vector<MapPoint *> &vpNonCorrectedMPs, vector<Door *> &vpCurrentMapDoors,
                                           vector<Plane *> &vpCurrentMapPlanes, vector<Marker *> &vpCurrentMapMarkers,
                                           vector<Room *> &vpCurrentDetMapRooms, vector<Room *> &vpCurrentMrkMapRooms,
                                           vector<vector<Eigen::Vector3d>> &vpClusterPoints);

        // For inertial loopclosing
        void static OptimizeEssentialGraph4DoF(Map *pMap, KeyFrame *pLoopKF, KeyFrame *pCurKF, const LoopClosing::KeyFrameAndPose &NonCorrectedSim3, const LoopClosing::KeyFrameAndPose &CorrectedSim3, const map<KeyFrame *, set<KeyFrame *>> &LoopConnections);

        // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono) (NEW)
        static int OptimizeSim3(KeyFrame *pKF1, KeyFrame *pKF2, std::vector<MapPoint *> &vpMatches1,
                                g2o::Sim3 &g2oS12, const float th2, const bool bFixScale,
                                Eigen::Matrix<double, 7, 7> &mAcumHessian, const bool bAllPoints = false);

        // For inertial systems

        void static LocalInertialBA(KeyFrame *pKF, bool *pbStopFlag, Map *pMap, int &countFixedKF, int &num_OptKF, int &num_MPs, int &num_edges, bool bLarge = false, bool bRecInit = false);
        void static MergeInertialBA(KeyFrame *pCurrKF, KeyFrame *pMergeKF, bool *pbStopFlag, Map *pMap, LoopClosing::KeyFrameAndPose &corrPoses);

        // Marginalize block element (start:end,start:end). Perform Schur complement.
        // Marginalized elements are filled with zeros.
        static Eigen::MatrixXd Marginalize(const Eigen::MatrixXd &H, const int &start, const int &end);

        // Inertial pose-graph
        void static InertialOptimization(Map *pMap, Eigen::Matrix3d &Rwg, double &scale, Eigen::Vector3d &bg, Eigen::Vector3d &ba, bool bMono, Eigen::MatrixXd &covInertial, bool bFixedVel = false, bool bGauss = false, float priorG = 1e2, float priorA = 1e6);
        void static InertialOptimization(Map *pMap, Eigen::Vector3d &bg, Eigen::Vector3d &ba, float priorG = 1e2, float priorA = 1e6);
        void static InertialOptimization(Map *pMap, Eigen::Matrix3d &Rwg, double &scale);

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

} // namespace ORB_SLAM3

#endif // OPTIMIZER_H
