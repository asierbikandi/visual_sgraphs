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

#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>
#include <openssl/md5.h>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/string.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>

namespace ORB_SLAM3
{

    Verbose::eLevel Verbose::th = Verbose::VERBOSITY_NORMAL;

    System::System(const string &strVocFile, const string &strSettingsFile, const string &strSysParamsFile, const eSensor sensor,
                   const bool bUseViewer, const int initFr, const string &strSequence) : mSensor(sensor), mpViewer(static_cast<Viewer *>(NULL)), mbReset(false), mbResetActiveMap(false),
                                                                                         mbActivateLocalizationMode(false), mbDeactivateLocalizationMode(false), mbShutDown(false)
    {
        // Output welcome message
        cout << endl
             << "Visual S-Graphs Copyright © 2023-2024 by Ali Tourani, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger Voos, SnT - University of Luxembourg." << endl
             << "Based on ORB-SLAM3 Copyright © 2017-2023 by C. Campos, R. Elvira, J.J. Gómez, J.M.M. Montiel, and J.D. Tardós, University of Zaragoza." << endl
             << "To redistribute the software please see LICENSE.txt." << endl
             << endl;

        // Input sensor
        cout << "Input sensor is set to: ";
        if (mSensor == MONOCULAR)
            cout << "Monocular" << endl;
        else if (mSensor == STEREO)
            cout << "Stereo" << endl;
        else if (mSensor == RGBD)
            cout << "RGB-D" << endl;
        else if (mSensor == IMU_MONOCULAR)
            cout << "Monocular-Inertial" << endl;
        else if (mSensor == IMU_STEREO)
            cout << "Stereo-Inertial" << endl;
        else if (mSensor == IMU_RGBD)
            cout << "RGB-D-Inertial" << endl;

        // Check settings file
        cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
        if (!fsSettings.isOpened())
        {
            cerr << "Failed to open settings file at: " << strSettingsFile << endl;
            exit(-1);
        }

        cv::FileNode node = fsSettings["File.version"];
        if (!node.empty() && node.isString() && node.string() == "1.0")
        {
            settings_ = new Settings(strSettingsFile, mSensor);

            mStrLoadAtlasFromFile = settings_->atlasLoadFile();
            mStrSaveAtlasToFile = settings_->atlasSaveFile();

            cout << (*settings_) << endl;
        }
        else
        {
            settings_ = nullptr;
            cv::FileNode node = fsSettings["System.LoadAtlasFromFile"];
            if (!node.empty() && node.isString())
            {
                mStrLoadAtlasFromFile = (string)node;
            }

            node = fsSettings["System.SaveAtlasToFile"];
            if (!node.empty() && node.isString())
            {
                mStrSaveAtlasToFile = (string)node;
            }
        }

        node = fsSettings["loopClosing"];
        bool activeLC = true;
        if (!node.empty())
        {
            activeLC = static_cast<int>(fsSettings["loopClosing"]) != 0;
        }

        mStrVocabularyFilePath = strVocFile;

        // Loading ORB Vocabulary
        cout << "[ORB Vocabulary]" << endl;
        cout << "- Loading ORB Vocabulary ..." << endl;

        mpVocabulary = new ORBVocabulary();
        bool bVocLoad = mpVocabulary->loadFromBinFile(strVocFile);
        if (!bVocLoad)
        {
            cerr << "- Wrong path to vocabulary. " << endl;
            cerr << "- Failed to open at: " << strVocFile << endl;
            exit(-1);
        }
        cout << "- Vocabulary loaded!" << endl
             << endl;

        // Create KeyFrame Database
        cout << "[Atlas Map Manager]" << endl;
        mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

        bool loadedAtlas = false;
        if (mStrLoadAtlasFromFile.empty())
        {
            // Create the Atlas
            cout << "- Initializing Atlas from scratch ..." << endl;
            mpAtlas = new Atlas(0);
            cout << "- Atlas is ready!" << endl
                 << endl;
        }
        else
        {
            // Load the file with an earlier session
            // clock_t start = clock();
            cout << "- Initializing Atlas from file: " << mStrLoadAtlasFromFile << "... " << endl;
            bool isRead = LoadAtlas(FileType::BINARY_FILE);

            if (!isRead)
            {
                cout << "- Error while loading the file, please try again!" << endl;
                exit(-1);
            }
            else
            {
                cout << "- Atlas is ready!" << endl
                     << endl;
            }

            loadedAtlas = true;

            mpAtlas->CreateNewMap();
        }

        // Setup the system parameters
        cout << "[System Params]" << endl;
        SystemParams *sysParams = SystemParams::GetParams();
        sysParams->SetParams(strSysParamsFile);

        // Parse the environment database
        cout << "[Environment Markers JSON File]" << endl;
        parseJsonDatabase(sysParams->general.env_database);

        // If the sensor is integrated with IMU, initialize the IMU first
        if (mSensor == IMU_STEREO || mSensor == IMU_MONOCULAR || mSensor == IMU_RGBD)
            mpAtlas->SetInertialSensor();

        // Create Drawers. These are used by the Viewer
        mpFrameDrawer = new FrameDrawer(mpAtlas);
        mpMapDrawer = new MapDrawer(mpAtlas, strSettingsFile, settings_);

        // Initialize the Tracking thread
        mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                                 mpAtlas, mpKeyFrameDatabase, strSettingsFile, mSensor, settings_, strSequence);

        // Set the value of marker impact
        mpTracker->SetMarkerImpact(sysParams->markers.impact);

        // Initialize the Local Mapping thread and launch
        mpLocalMapper = new LocalMapping(this, mpAtlas, mSensor == MONOCULAR || mSensor == IMU_MONOCULAR,
                                         mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor == IMU_RGBD, strSequence);
        mptLocalMapping = new thread(&ORB_SLAM3::LocalMapping::Run, mpLocalMapper);
        mpLocalMapper->mInitFr = initFr;
        if (settings_)
            mpLocalMapper->mThFarPoints = settings_->thFarPoints();
        else
            mpLocalMapper->mThFarPoints = fsSettings["thFarPoints"];
        if (mpLocalMapper->mThFarPoints != 0)
        {
            cout << "Discard points further than " << mpLocalMapper->mThFarPoints << " m from current camera" << endl;
            mpLocalMapper->mbFarPoints = true;
        }
        else
            mpLocalMapper->mbFarPoints = false;

        // Initialize the Loop Closing thread and launch
        mpLoopCloser = new LoopClosing(mpAtlas, mpKeyFrameDatabase, mpVocabulary, mSensor != MONOCULAR, activeLC);
        mptLoopClosing = new thread(&ORB_SLAM3::LoopClosing::Run, mpLoopCloser);

        // 🚀 [vS-Graphs v.2.0] Initialize Geometric Segmentation thread and launch
        bool hasDepthCloud = (mSensor == System::RGBD || mSensor == System::IMU_RGBD);
        mpGeometricSegmentation = new GeometricSegmentation(mpAtlas, hasDepthCloud, envDoors, envRooms);
        mptGeometricSegmentation = new thread(&GeometricSegmentation::Run, mpGeometricSegmentation);

        // 🚀 [vS-Graphs v.2.0] Initialize Semantic Segmentation thread and launch
        // [TODO] - launch threads based on flags
        mpSemanticSegmentation = new SemanticSegmentation(mpAtlas);
        mptSemanticSegmentation = new thread(&SemanticSegmentation::Run, mpSemanticSegmentation);

        // 🚀 [vS-Graphs v.2.0] Initialize Semantics Manager thread and launch
        mpSemanticsManager = new SemanticsManager(mpAtlas);
        mptSemanticsManager = new thread(&SemanticsManager::Run, mpSemanticsManager);

        // Set pointers between threads
        mpTracker->SetLoopClosing(mpLoopCloser);
        mpTracker->SetLocalMapper(mpLocalMapper);
        mpTracker->SetGeometricSegmentation(mpGeometricSegmentation);

        mpLocalMapper->SetTracker(mpTracker);
        mpLocalMapper->SetLoopCloser(mpLoopCloser);

        mpLoopCloser->SetTracker(mpTracker);
        mpLoopCloser->SetLocalMapper(mpLocalMapper);

        // Initialize the Viewer thread and launch
        if (bUseViewer)
        {
            mpViewer = new Viewer(this, mpFrameDrawer, mpMapDrawer, mpTracker, strSettingsFile, settings_);
            mptViewer = new thread(&Viewer::Run, mpViewer);
            mpTracker->SetViewer(mpViewer);
            mpLoopCloser->mpViewer = mpViewer;
            mpViewer->both = mpFrameDrawer->both;
        }

        // Fix verbosity
        Verbose::SetTh(Verbose::VERBOSITY_QUIET);
    }

    void System::parseJsonDatabase(string jsonFilePath)
    {
        // Skip the parsing
        if (jsonFilePath.empty())
        {
            std::cout << "- No JSON file describing the environment is provided. Skipping ..." << std::endl;
            return;
        }
        // Creating an object of the database loader
        ORB_SLAM3::DBParser parser;
        // Load JSON file
        json envData = parser.jsonParser(jsonFilePath);
        // Getting semantic entities
        envRooms = parser.getEnvRooms(envData);
        envDoors = parser.getEnvDoors(envData);
        // Printing the success message
        std::cout << "- JSON loaded and candidates created!\n";
    }

    void System::addSegmentedImage(std::tuple<uint64_t, cv::Mat, pcl::PCLPointCloud2::Ptr> *tuple)
    {
        // Adding the segmented image to the buffer of the SemanticSegmentation
        if (SystemParams::GetParams()->general.mode_of_operation == SystemParams::general::ModeOfOperation::GEO)
        {
            // just clear the pointcloud of the keyframe and return, as semantic segmentation is not running
            ORB_SLAM3::KeyFrame *pKF = mpAtlas->GetKeyFrameById(std::get<0>(*tuple));
            pKF->clearPointCloud();
            return;
        }

        mpSemanticSegmentation->AddSegmentedFrameToBuffer(tuple);
    }

    std::vector<std::vector<Eigen::Vector3d>> System::getSkeletonCluster()
    {
        return mpAtlas->GetSkeletoClusterPoints();
    }

    void System::setSkeletonCluster(const std::vector<std::vector<Eigen::Vector3d>> &skeletonClusterPoints)
    {
        // Adding the skeleton cluster to the SemanticSegmentation
        mpAtlas->SetSkeletonClusterPoints(skeletonClusterPoints);
    }

    void System::setGNNRoomCandidates(const std::vector<ORB_SLAM3::Room *> &gnnRoomCandidates)
    {
        // [TODO] Add the GNN room candidates to the SemanticsManager
    }

    Sophus::SE3f System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp,
                                     const vector<IMU::Point> &vImuMeas, string filename, const std::vector<Marker *> markers)
    {
        if (mSensor != STEREO && mSensor != IMU_STEREO)
        {
            cerr << "ERROR: you called TrackStereo but input sensor was not set to Stereo nor Stereo-Inertial." << endl;
            exit(-1);
        }

        cv::Mat imLeftToFeed, imRightToFeed;
        if (settings_ && settings_->needToRectify())
        {
            cv::Mat M1l = settings_->M1l();
            cv::Mat M2l = settings_->M2l();
            cv::Mat M1r = settings_->M1r();
            cv::Mat M2r = settings_->M2r();

            cv::remap(imLeft, imLeftToFeed, M1l, M2l, cv::INTER_LINEAR);
            cv::remap(imRight, imRightToFeed, M1r, M2r, cv::INTER_LINEAR);
        }
        else if (settings_ && settings_->needToResize())
        {
            cv::resize(imLeft, imLeftToFeed, settings_->newImSize());
            cv::resize(imRight, imRightToFeed, settings_->newImSize());
        }
        else
        {
            imLeftToFeed = imLeft.clone();
            imRightToFeed = imRight.clone();
        }

        // Check mode change
        {
            unique_lock<mutex> lock(mMutexMode);
            if (mbActivateLocalizationMode)
            {
                mpLocalMapper->RequestStop();

                // Wait until Local Mapping has effectively stopped
                while (!mpLocalMapper->isStopped())
                {
                    usleep(1000);
                }

                mpTracker->InformOnlyTracking(true);
                mbActivateLocalizationMode = false;
            }
            if (mbDeactivateLocalizationMode)
            {
                mpTracker->InformOnlyTracking(false);
                mpLocalMapper->Release();
                mbDeactivateLocalizationMode = false;
            }
        }

        {
            unique_lock<mutex> lock(mMutexReset);
            if (mbReset)
            {
                mpTracker->Reset();
                mbReset = false;
                mbResetActiveMap = false;
            }
            else if (mbResetActiveMap)
            {
                mpTracker->ResetActiveMap();
                mbResetActiveMap = false;
            }
        }

        if (mSensor == System::IMU_STEREO)
            for (size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
                mpTracker->GrabImuData(vImuMeas[i_imu]);

        Sophus::SE3f Tcw = mpTracker->GrabImageStereo(imLeftToFeed, imRightToFeed, timestamp, filename,
                                                      markers, envDoors, envRooms);

        unique_lock<mutex> lock2(mMutexState);
        mTrackingState = mpTracker->mState;
        mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
        mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

        return Tcw;
    }

    Sophus::SE3f System::TrackRGBD(const cv::Mat &colorImg, const cv::Mat &depthmap,
                                   const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &mainCloud,
                                   const double &timestamp, const vector<IMU::Point> &vImuMeas, string filename,
                                   const std::vector<Marker *> markers)
    {
        // Check if the sensor is correctly set as RGB-D
        if (mSensor != RGBD && mSensor != IMU_RGBD)
        {
            cerr << "[Error] Improper sensor-type is set for 'TrackRGBD'! Exiting ..." << endl;
            exit(-1);
        }

        // Obtain the images
        cv::Mat imToFeed = colorImg.clone();
        cv::Mat imDepthToFeed = depthmap.clone();
        if (settings_ && settings_->needToResize())
        {
            cv::Mat resizedImage;
            cv::resize(colorImg, resizedImage, settings_->newImSize());
            imToFeed = resizedImage;
            cv::resize(depthmap, imDepthToFeed, settings_->newImSize());
        }

        // Check for mode change
        {
            unique_lock<mutex> lock(mMutexMode);
            if (mbActivateLocalizationMode)
            {
                mpLocalMapper->RequestStop();
                // Wait until Local Mapping has effectively stopped
                while (!mpLocalMapper->isStopped())
                    usleep(1000);
                mpTracker->InformOnlyTracking(true);
                mbActivateLocalizationMode = false;
            }
            if (mbDeactivateLocalizationMode)
            {
                mpTracker->InformOnlyTracking(false);
                mpLocalMapper->Release();
                mbDeactivateLocalizationMode = false;
            }
        }

        // Check reset
        {
            unique_lock<mutex> lock(mMutexReset);
            if (mbReset)
            {
                mpTracker->Reset();
                mbReset = false;
                mbResetActiveMap = false;
            }
            else if (mbResetActiveMap)
            {
                mpTracker->ResetActiveMap();
                mbResetActiveMap = false;
            }
        }

        // Apply IMU measurements
        if (mSensor == System::IMU_RGBD)
            for (size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
                mpTracker->GrabImuData(vImuMeas[i_imu]);

        // Track RGB-D images
        Sophus::SE3f Tcw = mpTracker->GrabImageRGBD(imToFeed, imDepthToFeed, mainCloud, timestamp,
                                                    filename, markers, envDoors, envRooms);

        unique_lock<mutex> lock2(mMutexState);
        mTrackingState = mpTracker->mState;
        mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
        mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
        return Tcw;
    }

    Sophus::SE3f System::TrackMonocular(const cv::Mat &im, const double &timestamp, const vector<IMU::Point> &vImuMeas,
                                        string filename, const std::vector<Marker *> markers)
    {
        // Multi-thread to prevent race conditions
        {
            unique_lock<mutex> lock(mMutexReset);
            if (mbShutDown)
                return Sophus::SE3f();
        }

        // Check if the sensor is Monocular
        if (mSensor != MONOCULAR && mSensor != IMU_MONOCULAR)
        {
            cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular nor Monocular-Inertial." << endl;
            exit(-1);
        }

        // Obtain the images
        cv::Mat imToFeed = im.clone();
        if (settings_ && settings_->needToResize())
        {
            cv::Mat resizedImage;
            cv::resize(im, resizedImage, settings_->newImSize());
            imToFeed = resizedImage;
        }

        // Check mode change
        {
            unique_lock<mutex> lock(mMutexMode);
            if (mbActivateLocalizationMode)
            {
                mpLocalMapper->RequestStop();

                // Wait until Local Mapping has effectively stopped
                while (!mpLocalMapper->isStopped())
                {
                    usleep(1000);
                }

                mpTracker->InformOnlyTracking(true);
                mbActivateLocalizationMode = false;
            }
            if (mbDeactivateLocalizationMode)
            {
                mpTracker->InformOnlyTracking(false);
                mpLocalMapper->Release();
                mbDeactivateLocalizationMode = false;
            }
        }

        // Check reset
        {
            unique_lock<mutex> lock(mMutexReset);
            if (mbReset)
            {
                mpTracker->Reset();
                mbReset = false;
                mbResetActiveMap = false;
            }
            else if (mbResetActiveMap)
            {
                mpTracker->ResetActiveMap();
                mbResetActiveMap = false;
            }
        }

        if (mSensor == System::IMU_MONOCULAR)
            for (size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
                mpTracker->GrabImuData(vImuMeas[i_imu]);

        Sophus::SE3f Tcw = mpTracker->GrabImageMonocular(imToFeed, timestamp, filename, markers, envDoors, envRooms);

        unique_lock<mutex> lock2(mMutexState);
        mTrackingState = mpTracker->mState;
        mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
        mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
        return Tcw;
    }

    void System::ActivateLocalizationMode()
    {
        unique_lock<mutex> lock(mMutexMode);
        mbActivateLocalizationMode = true;
    }

    void System::DeactivateLocalizationMode()
    {
        unique_lock<mutex> lock(mMutexMode);
        mbDeactivateLocalizationMode = true;
    }

    bool System::MapChanged()
    {
        static int n = 0;
        int curn = mpAtlas->GetLastBigChangeIdx();
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
        unique_lock<mutex> lock(mMutexReset);
        mbReset = true;
    }

    void System::ResetActiveMap()
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetActiveMap = true;
    }

    void System::Shutdown()
    {
        {
            unique_lock<mutex> lock(mMutexReset);
            mbShutDown = true;
        }

        cout << "Shutdown" << endl;

        mpLocalMapper->RequestFinish();
        mpLoopCloser->RequestFinish();

        if (!mStrSaveAtlasToFile.empty())
        {
            Verbose::PrintMess("Atlas saving to file " + mStrSaveAtlasToFile, Verbose::VERBOSITY_NORMAL);
            SaveAtlas(FileType::BINARY_FILE);
        }

#ifdef REGISTER_TIMES
        mpTracker->PrintTimeStats();
#endif
    }

    bool System::isShutDown()
    {
        unique_lock<mutex> lock(mMutexReset);
        return mbShutDown;
    }

    void System::SaveTrajectoryTUM(const string &filename)
    {
        cout << endl
             << "Saving camera trajectory to " << filename << " ..." << endl;
        if (mSensor == MONOCULAR)
        {
            cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
            return;
        }

        vector<KeyFrame *> vpKFs = mpAtlas->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        Sophus::SE3f Two = vpKFs[0]->GetPoseInverse();

        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
        // We need to get first the keyframe pose and then concatenate the relative transformation.
        // Frames not localized (tracking failure) are not saved.

        // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
        // which is true when tracking failed (lbL).
        list<ORB_SLAM3::KeyFrame *>::iterator lRit = mpTracker->mlpReferences.begin();
        list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
        list<bool>::iterator lbL = mpTracker->mlbLost.begin();
        for (list<Sophus::SE3f>::iterator lit = mpTracker->mlRelativeFramePoses.begin(),
                                          lend = mpTracker->mlRelativeFramePoses.end();
             lit != lend; lit++, lRit++, lT++, lbL++)
        {
            if (*lbL)
                continue;

            KeyFrame *pKF = *lRit;

            Sophus::SE3f Trw;

            // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
            while (pKF->isBad())
            {
                Trw = Trw * pKF->mTcp;
                pKF = pKF->GetParent();
            }

            Trw = Trw * pKF->GetPose() * Two;

            Sophus::SE3f Tcw = (*lit) * Trw;
            Sophus::SE3f Twc = Tcw.inverse();

            Eigen::Vector3f twc = Twc.translation();
            Eigen::Quaternionf q = Twc.unit_quaternion();

            f << setprecision(6) << *lT << " " << setprecision(9) << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }
        f.close();
    }

    void System::SaveKeyFrameTrajectoryTUM(const string &filename)
    {
        cout << endl
             << "Saving keyframe trajectory to " << filename << " ..." << endl;

        vector<KeyFrame *> vpKFs = mpAtlas->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame *pKF = vpKFs[i];

            if (pKF->isBad())
                continue;

            Sophus::SE3f Twc = pKF->GetPoseInverse();
            Eigen::Quaternionf q = Twc.unit_quaternion();
            Eigen::Vector3f t = Twc.translation();
            f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t(0) << " " << t(1) << " " << t(2)
              << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }

        f.close();
    }

    void System::SaveTrajectoryEuRoC(const string &filename)
    {

        cout << endl
             << "Saving trajectory to " << filename << " ..." << endl;

        vector<Map *> vpMaps = mpAtlas->GetAllMaps();
        int numMaxKFs = 0;
        Map *pBiggerMap;
        std::cout << "There are " << std::to_string(vpMaps.size()) << " maps in the atlas" << std::endl;
        for (Map *pMap : vpMaps)
        {
            std::cout << "  Map " << std::to_string(pMap->GetId()) << " has " << std::to_string(pMap->GetAllKeyFrames().size()) << " KFs" << std::endl;
            if (pMap->GetAllKeyFrames().size() > numMaxKFs)
            {
                numMaxKFs = pMap->GetAllKeyFrames().size();
                pBiggerMap = pMap;
            }
        }

        vector<KeyFrame *> vpKFs = pBiggerMap->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        Sophus::SE3f Twb; // Can be word to cam0 or world to b depending on IMU or not.
        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor == IMU_RGBD)
            Twb = vpKFs[0]->GetImuPose();
        else
            Twb = vpKFs[0]->GetPoseInverse();

        ofstream f;
        f.open(filename.c_str());
        // cout << "file open" << endl;
        f << fixed;

        // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
        // We need to get first the keyframe pose and then concatenate the relative transformation.
        // Frames not localized (tracking failure) are not saved.

        // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
        // which is true when tracking failed (lbL).
        list<ORB_SLAM3::KeyFrame *>::iterator lRit = mpTracker->mlpReferences.begin();
        list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
        list<bool>::iterator lbL = mpTracker->mlbLost.begin();

        for (auto lit = mpTracker->mlRelativeFramePoses.begin(),
                  lend = mpTracker->mlRelativeFramePoses.end();
             lit != lend; lit++, lRit++, lT++, lbL++)
        {
            if (*lbL)
                continue;

            KeyFrame *pKF = *lRit;

            Sophus::SE3f Trw;

            // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
            if (!pKF)
                continue;

            while (pKF->isBad())
            {
                Trw = Trw * pKF->mTcp;
                pKF = pKF->GetParent();
            }

            if (!pKF || pKF->GetMap() != pBiggerMap)
                continue;

            Trw = Trw * pKF->GetPose() * Twb; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

            if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor == IMU_RGBD)
            {
                Sophus::SE3f Twb = (pKF->mImuCalib.mTbc * (*lit) * Trw).inverse();
                Eigen::Quaternionf q = Twb.unit_quaternion();
                Eigen::Vector3f twb = Twb.translation();
                f << setprecision(6) << 1e9 * (*lT) << " " << setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
            }
            else
            {
                Sophus::SE3f Twc = ((*lit) * Trw).inverse();
                Eigen::Quaternionf q = Twc.unit_quaternion();
                Eigen::Vector3f twc = Twc.translation();
                f << setprecision(6) << 1e9 * (*lT) << " " << setprecision(9) << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
            }
        }

        f.close();
        cout << endl
             << "End of saving trajectory to " << filename << " ..." << endl;
    }

    void System::SaveTrajectoryEuRoC(const string &filename, Map *pMap)
    {

        cout << endl
             << "Saving trajectory of map " << pMap->GetId() << " to " << filename << " ..." << endl;

        int numMaxKFs = 0;

        vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        Sophus::SE3f Twb; // Can be word to cam0 or world to b dependingo on IMU or not.
        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor == IMU_RGBD)
            Twb = vpKFs[0]->GetImuPose();
        else
            Twb = vpKFs[0]->GetPoseInverse();

        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
        // We need to get first the keyframe pose and then concatenate the relative transformation.
        // Frames not localized (tracking failure) are not saved.

        // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
        // which is true when tracking failed (lbL).
        list<ORB_SLAM3::KeyFrame *>::iterator lRit = mpTracker->mlpReferences.begin();
        list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
        list<bool>::iterator lbL = mpTracker->mlbLost.begin();

        for (auto lit = mpTracker->mlRelativeFramePoses.begin(),
                  lend = mpTracker->mlRelativeFramePoses.end();
             lit != lend; lit++, lRit++, lT++, lbL++)
        {
            if (*lbL)
                continue;

            KeyFrame *pKF = *lRit;

            Sophus::SE3f Trw;

            // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
            if (!pKF)
                continue;

            while (pKF->isBad())
            {
                Trw = Trw * pKF->mTcp;
                pKF = pKF->GetParent();
            }

            if (!pKF || pKF->GetMap() != pMap)
                continue;

            Trw = Trw * pKF->GetPose() * Twb; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

            if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor == IMU_RGBD)
            {
                Sophus::SE3f Twb = (pKF->mImuCalib.mTbc * (*lit) * Trw).inverse();
                Eigen::Quaternionf q = Twb.unit_quaternion();
                Eigen::Vector3f twb = Twb.translation();
                f << setprecision(6) << 1e9 * (*lT) << " " << setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
            }
            else
            {
                Sophus::SE3f Twc = ((*lit) * Trw).inverse();
                Eigen::Quaternionf q = Twc.unit_quaternion();
                Eigen::Vector3f twc = Twc.translation();
                f << setprecision(6) << 1e9 * (*lT) << " " << setprecision(9) << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
            }
        }
        f.close();
        cout << endl
             << "End of saving trajectory to " << filename << " ..." << endl;
    }

    void System::SaveKeyFrameTrajectoryEuRoC(const string &filename)
    {
        cout << endl
             << "Saving keyframe trajectory to " << filename << " ..." << endl;

        vector<Map *> vpMaps = mpAtlas->GetAllMaps();
        Map *pBiggerMap;
        int numMaxKFs = 0;
        for (Map *pMap : vpMaps)
        {
            if (pMap && pMap->GetAllKeyFrames().size() > numMaxKFs)
            {
                numMaxKFs = pMap->GetAllKeyFrames().size();
                pBiggerMap = pMap;
            }
        }

        if (!pBiggerMap)
        {
            std::cout << "There is not a map!!" << std::endl;
            return;
        }

        vector<KeyFrame *> vpKFs = pBiggerMap->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame *pKF = vpKFs[i];

            if (!pKF || pKF->isBad())
                continue;
            if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor == IMU_RGBD)
            {
                Sophus::SE3f Twb = pKF->GetImuPose();
                Eigen::Quaternionf q = Twb.unit_quaternion();
                Eigen::Vector3f twb = Twb.translation();
                f << setprecision(6) << 1e9 * pKF->mTimeStamp << " " << setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
            }
            else
            {
                Sophus::SE3f Twc = pKF->GetPoseInverse();
                Eigen::Quaternionf q = Twc.unit_quaternion();
                Eigen::Vector3f t = Twc.translation();
                f << setprecision(6) << 1e9 * pKF->mTimeStamp << " " << setprecision(9) << t(0) << " " << t(1) << " " << t(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
            }
        }
        f.close();
    }

    void System::SaveKeyFrameTrajectoryEuRoC(const string &filename, Map *pMap)
    {
        cout << endl
             << "Saving keyframe trajectory of map " << pMap->GetId() << " to " << filename << " ..." << endl;

        vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame *pKF = vpKFs[i];

            if (!pKF || pKF->isBad())
                continue;
            if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor == IMU_RGBD)
            {
                Sophus::SE3f Twb = pKF->GetImuPose();
                Eigen::Quaternionf q = Twb.unit_quaternion();
                Eigen::Vector3f twb = Twb.translation();
                f << setprecision(6) << 1e9 * pKF->mTimeStamp << " " << setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
            }
            else
            {
                Sophus::SE3f Twc = pKF->GetPoseInverse();
                Eigen::Quaternionf q = Twc.unit_quaternion();
                Eigen::Vector3f t = Twc.translation();
                f << setprecision(6) << 1e9 * pKF->mTimeStamp << " " << setprecision(9) << t(0) << " " << t(1) << " " << t(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
            }
        }
        f.close();
    }

    void System::SaveTrajectoryKITTI(const string &filename)
    {
        cout << endl
             << "Saving camera trajectory to " << filename << " ..." << endl;
        if (mSensor == MONOCULAR)
        {
            cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
            return;
        }

        vector<KeyFrame *> vpKFs = mpAtlas->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        Sophus::SE3f Tow = vpKFs[0]->GetPoseInverse();

        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
        // We need to get first the keyframe pose and then concatenate the relative transformation.
        // Frames not localized (tracking failure) are not saved.

        // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
        // which is true when tracking failed (lbL).
        list<ORB_SLAM3::KeyFrame *>::iterator lRit = mpTracker->mlpReferences.begin();
        list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
        for (list<Sophus::SE3f>::iterator lit = mpTracker->mlRelativeFramePoses.begin(),
                                          lend = mpTracker->mlRelativeFramePoses.end();
             lit != lend; lit++, lRit++, lT++)
        {
            ORB_SLAM3::KeyFrame *pKF = *lRit;

            Sophus::SE3f Trw;

            if (!pKF)
                continue;

            while (pKF->isBad())
            {
                Trw = Trw * pKF->mTcp;
                pKF = pKF->GetParent();
            }

            Trw = Trw * pKF->GetPose() * Tow;

            Sophus::SE3f Tcw = (*lit) * Trw;
            Sophus::SE3f Twc = Tcw.inverse();
            Eigen::Matrix3f Rwc = Twc.rotationMatrix();
            Eigen::Vector3f twc = Twc.translation();

            f << setprecision(9) << Rwc(0, 0) << " " << Rwc(0, 1) << " " << Rwc(0, 2) << " " << twc(0) << " " << Rwc(1, 0) << " " << Rwc(1, 1) << " " << Rwc(1, 2) << " " << twc(1) << " " << Rwc(2, 0) << " " << Rwc(2, 1) << " " << Rwc(2, 2) << " " << twc(2) << endl;
        }
        f.close();
    }

    void System::SaveDebugData(const int &initIdx)
    {
        // 0. Save initialization trajectory
        SaveTrajectoryEuRoC("init_FrameTrajectoy_" + to_string(mpLocalMapper->mInitSect) + "_" + to_string(initIdx) + ".txt");

        // 1. Save scale
        ofstream f;
        f.open("init_Scale_" + to_string(mpLocalMapper->mInitSect) + ".txt", ios_base::app);
        f << fixed;
        f << mpLocalMapper->mScale << endl;
        f.close();

        // 2. Save gravity direction
        f.open("init_GDir_" + to_string(mpLocalMapper->mInitSect) + ".txt", ios_base::app);
        f << fixed;
        f << mpLocalMapper->mRwg(0, 0) << "," << mpLocalMapper->mRwg(0, 1) << "," << mpLocalMapper->mRwg(0, 2) << endl;
        f << mpLocalMapper->mRwg(1, 0) << "," << mpLocalMapper->mRwg(1, 1) << "," << mpLocalMapper->mRwg(1, 2) << endl;
        f << mpLocalMapper->mRwg(2, 0) << "," << mpLocalMapper->mRwg(2, 1) << "," << mpLocalMapper->mRwg(2, 2) << endl;
        f.close();

        // 3. Save computational cost
        f.open("init_CompCost_" + to_string(mpLocalMapper->mInitSect) + ".txt", ios_base::app);
        f << fixed;
        f << mpLocalMapper->mCostTime << endl;
        f.close();

        // 4. Save biases
        f.open("init_Biases_" + to_string(mpLocalMapper->mInitSect) + ".txt", ios_base::app);
        f << fixed;
        f << mpLocalMapper->mbg(0) << "," << mpLocalMapper->mbg(1) << "," << mpLocalMapper->mbg(2) << endl;
        f << mpLocalMapper->mba(0) << "," << mpLocalMapper->mba(1) << "," << mpLocalMapper->mba(2) << endl;
        f.close();

        // 5. Save covariance matrix
        f.open("init_CovMatrix_" + to_string(mpLocalMapper->mInitSect) + "_" + to_string(initIdx) + ".txt", ios_base::app);
        f << fixed;
        for (int i = 0; i < mpLocalMapper->mcovInertial.rows(); i++)
        {
            for (int j = 0; j < mpLocalMapper->mcovInertial.cols(); j++)
            {
                if (j != 0)
                    f << ",";
                f << setprecision(15) << mpLocalMapper->mcovInertial(i, j);
            }
            f << endl;
        }
        f.close();

        // 6. Save initialization time
        f.open("init_Time_" + to_string(mpLocalMapper->mInitSect) + ".txt", ios_base::app);
        f << fixed;
        f << mpLocalMapper->mInitTime << endl;
        f.close();
    }

    int System::GetTrackingState()
    {
        unique_lock<mutex> lock(mMutexState);
        return mTrackingState;
    }

    vector<MapPoint *> System::GetTrackedMapPoints()
    {
        unique_lock<mutex> lock(mMutexState);
        return mTrackedMapPoints;
    }

    vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
    {
        unique_lock<mutex> lock(mMutexState);
        return mTrackedKeyPointsUn;
    }

    cv::Mat System::GetCurrentFrame()
    {
        return mpFrameDrawer->DrawFrame();
    }

    std::vector<KeyFrame *> System::GetAllKeyFrames()
    {
        return mpAtlas->GetAllKeyFrames();
    }

    Sophus::SE3f System::GetCamTwc()
    {
        return mpTracker->GetCamTwc();
    }

    Sophus::SE3f System::GetImuTwb()
    {
        return mpTracker->GetImuTwb();
    }

    Eigen::Vector3f System::GetImuVwb()
    {
        return mpTracker->GetImuVwb();
    }

    bool System::isImuPreintegrated()
    {
        return mpTracker->isImuPreintegrated();
    }

    double System::GetTimeFromIMUInit()
    {
        double aux = mpLocalMapper->GetCurrKFTime() - mpLocalMapper->mFirstTs;
        if ((aux > 0.) && mpAtlas->isImuInitialized())
            return mpLocalMapper->GetCurrKFTime() - mpLocalMapper->mFirstTs;
        else
            return 0.f;
    }

    bool System::isLost()
    {
        if (!mpAtlas->isImuInitialized())
            return false;
        else
        {
            if ((mpTracker->mState == Tracking::LOST)) //||(mpTracker->mState==Tracking::RECENTLY_LOST))
                return true;
            else
                return false;
        }
    }

    bool System::isFinished()
    {
        return (GetTimeFromIMUInit() > 0.1);
    }

    void System::ChangeDataset()
    {
        if (mpAtlas->GetCurrentMap()->KeyFramesInMap() < 12)
        {
            mpTracker->ResetActiveMap();
        }
        else
        {
            mpTracker->CreateMapInAtlas();
        }

        mpTracker->NewDataset();
    }

    float System::GetImageScale()
    {
        return mpTracker->GetImageScale();
    }

#ifdef REGISTER_TIMES
    void System::InsertRectTime(double &time)
    {
        mpTracker->vdRectStereo_ms.push_back(time);
    }

    void System::InsertResizeTime(double &time)
    {
        mpTracker->vdResizeImage_ms.push_back(time);
    }

    void System::InsertTrackTime(double &time)
    {
        mpTracker->vdTrackTotal_ms.push_back(time);
    }
#endif

    bool System::SaveAtlas(int type)
    {
        try
        {
            if (!mStrSaveAtlasToFile.empty())
            {
                // Save the current session
                mpAtlas->PreSave();

                string pathSaveFileName = "./";
                pathSaveFileName = pathSaveFileName.append(mStrSaveAtlasToFile);
                pathSaveFileName = pathSaveFileName.append(".osa");

                string strVocabularyChecksum = CalculateCheckSum(mStrVocabularyFilePath, TEXT_FILE);
                std::size_t found = mStrVocabularyFilePath.find_last_of("/\\");
                string strVocabularyName = mStrVocabularyFilePath.substr(found + 1);

                if (type == TEXT_FILE) // File text
                {
                    cout << "Starting to write the save text file to " << pathSaveFileName.c_str() << endl;
                    std::remove(pathSaveFileName.c_str());
                    std::ofstream ofs(pathSaveFileName, std::ios::binary);
                    boost::archive::text_oarchive oa(ofs);

                    oa << strVocabularyName;
                    oa << strVocabularyChecksum;
                    oa << mpAtlas;
                    cout << "End to write the save text file" << endl;
                }
                else if (type == BINARY_FILE) // File binary
                {
                    cout << "Starting to write the save binary file to " << pathSaveFileName.c_str() << endl;
                    std::remove(pathSaveFileName.c_str());
                    std::ofstream ofs(pathSaveFileName, std::ios::binary);
                    boost::archive::binary_oarchive oa(ofs);
                    oa << strVocabularyName;
                    oa << strVocabularyChecksum;
                    oa << mpAtlas;
                    cout << "End to write save binary file" << endl;
                }
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << std::endl;
            return false;
        }
        catch (...)
        {
            std::cerr << "Unknows exeption" << std::endl;
            return false;
        }

        return true;
    }

    bool System::LoadAtlas(int type)
    {
        string strFileVoc, strVocChecksum;
        bool isRead = false;

        string pathLoadFileName = "./";
        pathLoadFileName = pathLoadFileName.append(mStrLoadAtlasFromFile);
        pathLoadFileName = pathLoadFileName.append(".osa");

        if (type == TEXT_FILE) // File text
        {
            cout << "Starting to read the save text file " << pathLoadFileName.c_str() << endl;
            std::ifstream ifs(pathLoadFileName, std::ios::binary);
            if (!ifs.good())
            {
                cout << "Load file not found" << endl;
                return false;
            }
            boost::archive::text_iarchive ia(ifs);
            ia >> strFileVoc;
            ia >> strVocChecksum;
            ia >> mpAtlas;
            cout << "End to load the save text file " << endl;
            isRead = true;
        }
        else if (type == BINARY_FILE) // File binary
        {
            cout << "Starting to read the save binary file " << pathLoadFileName.c_str() << endl;
            std::ifstream ifs(pathLoadFileName, std::ios::binary);
            if (!ifs.good())
            {
                cout << "Load file not found" << endl;
                return false;
            }
            boost::archive::binary_iarchive ia(ifs);
            ia >> strFileVoc;
            ia >> strVocChecksum;
            ia >> mpAtlas;
            cout << "End to load the save binary file" << endl;
            isRead = true;
        }

        if (isRead)
        {
            // Check if the vocabulary is the same
            string strInputVocabularyChecksum = CalculateCheckSum(mStrVocabularyFilePath, TEXT_FILE);

            if (strInputVocabularyChecksum.compare(strVocChecksum) != 0)
            {
                cout << "The vocabulary load isn't the same which the load session was created " << endl;
                cout << "-Vocabulary name: " << strFileVoc << endl;
                return false; // Both are differents
            }

            mpAtlas->SetKeyFrameDababase(mpKeyFrameDatabase);
            mpAtlas->SetORBVocabulary(mpVocabulary);
            mpAtlas->PostLoad();

            return true;
        }
        return false;
    }

    string System::CalculateCheckSum(string filename, int type)
    {
        string checksum = "";

        unsigned char c[MD5_DIGEST_LENGTH];

        std::ios_base::openmode flags = std::ios::in;
        if (type == BINARY_FILE) // Binary file
            flags = std::ios::in | std::ios::binary;

        ifstream f(filename.c_str(), flags);
        if (!f.is_open())
        {
            cout << "[E] Unable to open the in file " << filename << " for Md5 hash." << endl;
            return checksum;
        }

        MD5_CTX md5Context;
        char buffer[1024];

        MD5_Init(&md5Context);
        while (int count = f.readsome(buffer, sizeof(buffer)))
        {
            MD5_Update(&md5Context, buffer, count);
        }

        f.close();

        MD5_Final(c, &md5Context);

        for (int i = 0; i < MD5_DIGEST_LENGTH; i++)
        {
            char aux[10];
            sprintf(aux, "%02x", c[i]);
            checksum = checksum + aux;
        }

        return checksum;
    }

    ORB_SLAM3::Map *System::GetCurrentMap()
    {
        ORB_SLAM3::Map *pActiveMap = mpAtlas->GetCurrentMap();
        return pActiveMap;
    }

    vector<MapPoint *> System::GetAllMapPoints()
    {
        Map *pActiveMap = mpAtlas->GetCurrentMap();
        return pActiveMap->GetAllMapPoints();
    }

    vector<Sophus::SE3f> System::GetAllKeyframePoses()
    {
        vector<KeyFrame *> vpKFs = mpAtlas->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        vector<Sophus::SE3f> vKFposes;

        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame *pKF = vpKFs[i];

            if (pKF->isBad())
                continue;

            // Twb can be world frame to cam0 frame (without IMU) or body in world frame (with IMU)
            Sophus::SE3f Twb;
            if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor == IMU_RGBD) // with IMU
                Twb = vpKFs[i]->GetImuPose();
            else // without IMU
                Twb = vpKFs[i]->GetPoseInverse();

            vKFposes.push_back(Twb);
        }

        return vKFposes;
    }

    Sophus::SE3f System::GetKeyFramePose(KeyFrame *pKF)
    {
        if (pKF->isBad())
            return Sophus::SE3f();

        // Twb can be world frame to cam0 frame (without IMU) or body in world frame (with IMU)
        Sophus::SE3f Twb;
        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor == IMU_RGBD) // with IMU
            Twb = pKF->GetImuPose();
        else // without IMU
            Twb = pKF->GetPoseInverse();

        return Twb;
    }

    vector<Marker *> System::GetAllMarkers()
    {
        Map *pActiveMap = mpAtlas->GetCurrentMap();
        return pActiveMap->GetAllMarkers();
    }

    vector<Door *> System::GetAllDoors()
    {
        Map *pActiveMap = mpAtlas->GetCurrentMap();
        return pActiveMap->GetAllDoors();
    }

    vector<Plane *> System::GetAllPlanes()
    {
        Map *pActiveMap = mpAtlas->GetCurrentMap();
        return pActiveMap->GetAllPlanes();
    }

    vector<Room *> System::GetAllRooms()
    {
        Map *pActiveMap = mpAtlas->GetCurrentMap();
        return pActiveMap->GetAllRooms();
    }

    bool System::SaveMap(const string &filename)
    {
        mStrSaveAtlasToFile = filename;
        if (!mStrSaveAtlasToFile.empty())
        {
            Verbose::PrintMess("Atlas saving to file " + mStrSaveAtlasToFile, Verbose::VERBOSITY_NORMAL);
            return SaveAtlas(FileType::BINARY_FILE);
        }
        return false;
    }

    bool System::SaveMapPointsAsPCD(const string &filename)
    {
        try
        {
            // make a pointcloud out of all map points
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            vector<MapPoint *> vpMPs = mpAtlas->GetCurrentMap()->GetAllMapPoints();
            for (size_t i = 0; i < vpMPs.size(); i++)
            {
                MapPoint *pMP = vpMPs[i];
                if (pMP->isBad())
                    continue;

                Eigen::Vector3d P3Dw = pMP->GetWorldPos().cast<double>();
                pcl::PointXYZ point;
                point.x = P3Dw.x();
                point.y = P3Dw.y();
                point.z = P3Dw.z();
                cloud->push_back(point);
            }

            // save the pointcloud
            pcl::io::savePCDFileBinary(filename + ".pcd", *cloud);

            return true;
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << std::endl;
            return false;
        }
        catch (...)
        {
            std::cerr << "Unknows exeption" << std::endl;
            return false;
        }
    }

}
