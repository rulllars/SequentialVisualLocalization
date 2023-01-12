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

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#include <iomanip>
#include <ctime>
#include <regex>
#include <random>

#include <opencv2/core/core.hpp>

#include "System.h"
#include "SLAMDataTypes.h"

#if defined(WIN32) || defined(_WIN32)
#define PATH_SEPARATOR "\\"
#else
#define PATH_SEPARATOR "/"
#endif

using namespace std;

bool LoadImages(
    const string &strPathToDataset,
    const string &strPathToImgNames,
    const int seqId,
    vector<string> &vstrImageFilenames,
    vector<int> &vCameraID,
    vector<double> &vTimestamps,
    const int nOffset, int nFrames);

bool LoadOdometry(
    const string &strPathToDataset,
    const string &strOdoFile,
    vector<LocSLAM::OdometryData> &vOdometry,
    const double startTime,
    double sigmaAngle,
    double sigmaTrans);

bool LoadFixedMapMatches(
    LocSLAM::FixedMapMatches &FrameFixedMapMatches,
    const string &strBasePath,
    const string &strCorrDir,
    const string &strCorrSuffix,
    const string &strPoseDir,
    const string &strPoseSuffix,
    const string &strImageName,
    const int cameraID,
    const double sampleRatio = 1.01);

int main(int argc, char **argv)
{
    if (argc != 4)
    {
        cerr << endl
             << "Usage: ./locSLAM path_to_vocabulary path_to_settings settings_file" << endl;
        return 1;
    }

    string strSettingsFolder = argv[2];
    if (!(strSettingsFolder.find_last_of(PATH_SEPARATOR) == (strSettingsFolder.length() - 1)))
        strSettingsFolder = strSettingsFolder + PATH_SEPARATOR;

    string strSettingsFile = argv[3];
    string strSettingsPath = strSettingsFolder + strSettingsFile;
    cv::FileStorage fSettings(strSettingsPath, cv::FileStorage::READ);

    // Viewer
    bool bUseViewer = (int)fSettings["Viewer.UseViewer"] == 1;

    // Retrieve load specific parameters
    const string loadBasePath = fSettings["Data.LoadBasePath"];
    const string imageFileList = fSettings["Data.ImageFileList"];
    const int sequenceId = fSettings["Data.SequenceId"];
    const string saveBasePath = fSettings["Data.SaveBasePath"];

    const int nOffset = fSettings["Load.Offset"];
    const int nFrames = fSettings["Load.nFrames"];
    const int nCameras = fSettings["nCameras"];

    cv::Mat mat = cv::Mat::eye(nCameras, 1, CV_32SC1);
    fSettings["supportedCameras"] >> mat;
    vector<int> vSupportedCameras = mat;
    // Global map parameters
    int tmp = fSettings["FixedMap.UseFixedMap"];
    bool bUseFixedMap = tmp == 1;

    string strPathFixedMap = fSettings["FixedMap.BasePath"];
    string strCorrDir = fSettings["FixedMap.CorrespondencePath"];
    string strCorrSuffix = fSettings["FixedMap.CorrespondenceSuffix"];
    string strPoseDir = fSettings["FixedMap.PosePath"];
    string strPoseSuffix = fSettings["FixedMap.PoseSuffix"];
    double sampleRatio = fSettings["FixedMap.SampleRatio"];

    double fixedMapMinInterval = fSettings["FixedMap.MinInterval"];
    double sigmaFeature = fSettings["FixedMap.SigmaFeature"];
    double sigmaPriorPos = fSettings["FixedMap.SigmaPriorPos"];
    double sigmaPriorHead = fSettings["FixedMap.sigmaPriorHead"];

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    vector<int> vCameraID;
    LoadImages(loadBasePath, imageFileList, sequenceId, vstrImageFilenames, vCameraID, vTimestamps, nOffset, nFrames);

    int nImages = vstrImageFilenames.size();

    // Load odometry data
    //If the odometry sample is more than a microsecond off from the image, then we ought to interpolate.
    const double maxTimeDiff = 1e-3;
    vector<LocSLAM::OdometryData> vOdometry;
    string strOdoFile = fSettings["Data.OdometryFile"];
    double sigmaAngle = fSettings["Tracking.OdoPredictAngleUncertainty"];
    double sigmaTrans = fSettings["Tracking.OdoPredictTransUncertainty"];

    bool bOdometry = LoadOdometry(loadBasePath, strOdoFile, vOdometry, vTimestamps[0] - maxTimeDiff, sigmaAngle, sigmaTrans);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    LocSLAM::System SLAM(argv[1], strSettingsPath, LocSLAM::System::MONOCULAR, bUseViewer, bOdometry, bUseFixedMap);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl
         << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl
         << endl;

    // Main loop
    cv::Mat im;
    int iOdo = 0;
    vector<double> vtLastFixedMapMatch(nCameras, vTimestamps[0] + fmin(0.0, 0.2 - fixedMapMinInterval));

    for (int ni = 0; ni < nImages; ni++)
    {
        if (vSupportedCameras[vCameraID[ni]])
        {
            // Read image from file
            std::string fn = (fs::path(loadBasePath) / fs::path(vstrImageFilenames[ni])).string();
            im = cv::imread(fn, CV_LOAD_IMAGE_UNCHANGED);
            double tframe = vTimestamps[ni];
            int idFrame = vCameraID[ni];

            if (im.empty())
            {
                cerr << endl
                     << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
                SLAM.Shutdown();
                return 1;
            }

            // string strImageName = vstrImageFilenames[ni].substr(vstrImageFilenames[ni].length() - 35, vstrImageFilenames[ni].length());
            string strImageName = vstrImageFilenames[ni];
            string strImageFullName = (fs::path(loadBasePath) / fs::path(vstrImageFilenames[ni])).string();

            // Load global map correspondences
            LocSLAM::FixedMapMatches frameFixedMapMatches;
            frameFixedMapMatches.sigmaFeature = sigmaFeature;
            frameFixedMapMatches.sigmaPriorPosition = sigmaPriorPos;
            frameFixedMapMatches.sigmaPriorHeading = sigmaPriorHead;

            frameFixedMapMatches.informationMatrix = 1.0 / pow(sigmaFeature, 2.0) * Eigen::Matrix2d::Identity();

            if (bUseFixedMap && tframe >= vtLastFixedMapMatch[idFrame] + fixedMapMinInterval)
            {
                bool success = LoadFixedMapMatches(frameFixedMapMatches, strPathFixedMap,
                                                   strCorrDir, strCorrSuffix, strPoseDir, strPoseSuffix, strImageName, vCameraID[ni], sampleRatio);
                if (success && frameFixedMapMatches.nMatches > 0)
                {
                    vtLastFixedMapMatch[idFrame] = tframe;
                }
            }

            // Pre-integrate odometry to nearest timestamp
            LocSLAM::OdometryData integratedOdometry;
            while (iOdo < vOdometry.size() && fabs(vOdometry[iOdo].mTimeStamp - tframe) < fabs(integratedOdometry.mTimeStamp - tframe))
            {
                integratedOdometry.Integrate(vOdometry[iOdo++]);
            }

            // Check odometry time stamp
            if (bOdometry && fabs(integratedOdometry.mTimeStamp - tframe) > maxTimeDiff)
            {
                cout << endl
                     << "Time mismatch between odometry and images: " << abs(integratedOdometry.mTimeStamp - tframe) << endl;
            }

            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

            // Pass the image to the SLAM system
            SLAM.TrackMonocularOdo(im, integratedOdometry, frameFixedMapMatches, tframe, idFrame, strImageFullName);

            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

            double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();

            vTimesTrack[ni] = ttrack;

            // Wait to load the next frame
            double T = 0;
            if (ni < nImages - 1)
                T = vTimestamps[ni + 1] - tframe;
            else if (ni > 0)
                T = tframe - vTimestamps[ni - 1];

            // LocSLAM is not currently real-time. Run LocalMapping() in a separate thread for this to be usefull.
            //if(ttrack<T)
            //   usleep((T-ttrack)*1e6);
        }
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(), vTimesTrack.end());
    float totaltime = 0;
    for (int ni = 0; ni < nImages; ni++)
    {
        totaltime += vTimesTrack[ni];
    }
    cout << "-------" << endl
         << endl;
    cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
    cout << "mean tracking time: " << totaltime / nImages << endl;

    SLAM.SaveTrajectoryCMU(strSettingsFolder + strSettingsFile + ".result.txt", false);
    SLAM.SaveTrajectoryCMU(strSettingsFolder + strSettingsFile + ".bench-result.txt", true);
    SLAM.SaveTrackingTrajectoryCMU(strSettingsFolder + strSettingsFile + ".tracking.bench-result.txt", true);
    SLAM.SaveTrackingTrajectoryCMU(strSettingsFolder + strSettingsFile + ".tracking.result.txt", false);
    SLAM.SaveKeyFrameTrajectoryCMU(strSettingsFolder + strSettingsFile + ".KF.result.txt", false);

    return 0;
}

double ExtractTimestamp(const std::regex &time_re, const string &s)
{
    // Parse time stamp - assuming timestamp in microseconds is part of filename
    std::smatch m;
    bool matched = std::regex_search(s, m, time_re);
    if (matched)
    {
        std::string s1 = m[1];
        return std::stod(s1) * 1e-6;
    }
    else
        return NAN;
}

template <typename T>
vector<size_t> sort_indexes(const vector<T> &v)
{

    // initialize original index locations
    vector<size_t> idx(v.size());
    iota(idx.begin(), idx.end(), 0);

    // sort indexes based on comparing values in v
    // using std::stable_sort instead of std::sort
    // to avoid unnecessary index re-orderings
    // when v contains elements of equal values
    stable_sort(idx.begin(), idx.end(),
                [&v](size_t i1, size_t i2)
                { return v[i1] < v[i2]; });

    return idx;
}

bool LoadImages(
    const string &strPathToDataset,
    const string &strPathToImgNames,
    const int seqId,
    vector<string> &vstrImageFilenames,
    vector<int> &vCameraID,
    vector<double> &vTimestamps,
    const int nOffset, int nFrames)
{
    string strSequenceId = std::to_string(seqId);
    ifstream fImgNames;
    fImgNames.open(fs::path(strPathToDataset) / fs::path(strPathToImgNames));

    // No image list file?
    if (fImgNames.fail())
    {
        std::cout << "Could not load list of images to run!" << endl;
        return false;
    }

    //Skip to beginning of sequence with id seqId parameter
    string fileName = "";
    string strTimeStampRead = "";
    string strCamIdRead = "";
    string strSeqIdRead = "";
    while (!fImgNames.eof() && (strSeqIdRead.size() < strSequenceId.size() ||
                                !std::equal(strSequenceId.rbegin(), strSequenceId.rend(), strSeqIdRead.rbegin())))
        fImgNames >> fileName >> strTimeStampRead >> strCamIdRead >> strSeqIdRead;

    int nFiles = 0;
    while (!fImgNames.eof() && std::equal(strSequenceId.rbegin(), strSequenceId.rend(), strSeqIdRead.rbegin()))
    {
        // Parse odometry data to struct
        if (fileName.size() > 4)
        {
            vstrImageFilenames.push_back(fileName);
            vTimestamps.push_back(std::stod(strTimeStampRead) * 1e-6);
            vCameraID.push_back(std::stoi(strCamIdRead));

            // Increase read lines
            nFiles++;
        }

        //Read next line
        fImgNames >> fileName >> strTimeStampRead >> strCamIdRead >> strSeqIdRead;
    }

    // Check if we will actually have files to load
    if (nFiles - nOffset <= 0)
    {
        cerr << "LoadImages: No files to load! Files in sequence: " << nFiles << " Offset: " << nOffset << endl;
        return false;
    }

    // Make sure images are in order
    vector<string> tmpNames(vstrImageFilenames);
    vector<double> tmpTimes(vTimestamps);
    vector<int> tmpCameraID(vCameraID);
    vector<size_t> sortIdx = sort_indexes(vTimestamps);
    size_t j = 0;
    for (auto i : sort_indexes(vTimestamps))
    {
        vstrImageFilenames[j] = tmpNames[i];
        vTimestamps[j] = tmpTimes[i];
        vCameraID[j] = tmpCameraID[i];
        j++;
    }

    if (nOffset > 0)
    {
        vstrImageFilenames.erase(vstrImageFilenames.begin(), vstrImageFilenames.begin() + nOffset);
        vTimestamps.erase(vTimestamps.begin(), vTimestamps.begin() + nOffset);
        vCameraID.erase(vCameraID.begin(), vCameraID.begin() + nOffset);
    }
    if (nFrames > 0)
    {
        vstrImageFilenames.erase(vstrImageFilenames.begin() + nFrames, vstrImageFilenames.end());
        vTimestamps.erase(vTimestamps.begin() + nFrames, vTimestamps.end());
        vCameraID.erase(vCameraID.begin() + nFrames, vCameraID.end());
    }
}

bool LoadOdometry(const string &strPathToDataset, const string &strOdoFile, vector<LocSLAM::OdometryData> &vOdometry, const double startTime, double sigmaAngle, double sigmaTrans)
{
    ifstream fOdometry;
    string strPathOdometryFile = (fs::path(strPathToDataset) / fs::path(strOdoFile)).string();
    string fileEnd = strPathOdometryFile.substr(strPathOdometryFile.size() - 1);
    if (fileEnd == "/")
    {
        strPathOdometryFile += "odoMeas.txt";
    }
    fOdometry.open(strPathOdometryFile.c_str());

    // No odometry data?
    if (fOdometry.fail())
    {
        cout << "No odometry data loaded." << endl;
        return false;
    }

    // First line(s) may contain some header information
    while (!fOdometry.eof() && !isdigit(fOdometry.peek()))
    {
        string s;
        getline(fOdometry, s);
    }

    cout << "Reading odometry from " << strPathOdometryFile << "... ";

    int lineCount = 0;
    bool startRead = false;
    while (!fOdometry.eof())
    {
        double time, dt;
        float v_x, v_y, v_z, w_x, w_y, w_z;

        // Parse odometry data to struct
        fOdometry >> time >> dt >> v_x >> v_y >> v_z >> w_x >> w_y >> w_z;

        // Increase read lines
        lineCount++;

        if (time >= startTime - 1e-2 && time <= startTime + 1e-2)
            startRead = true;

        // If we have read enough lines start appending data
        if (startRead)
        {
            LocSLAM::OdometryData odo(time, dt, v_x, v_y, v_z, w_x, w_y, w_z, sigmaAngle, sigmaTrans);
            vOdometry.push_back(odo);
        }
    }
    cout << "Done. " << endl;

    return true;
}

bool LoadFixedMapMatches(
    LocSLAM::FixedMapMatches &FrameFixedMapMatches,
    const string &strBasePath,
    const string &strCorrDir,
    const string &strCorrSuffix,
    const string &strPoseDir,
    const string &strPoseSuffix,
    const string &strImageName,
    const int cameraID,
    const double sampleRatio)
{

    int nMaxMatches = 4000;
    int nMinMatches = 40;

    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(0.0, 1.0);

    // Load matches
    ifstream fMatches;
    string strPathCorrespondenceFile = (fs::path(strBasePath) / fs::path(strCorrDir) / fs::path(strImageName + strCorrSuffix)).string();
    fMatches.open(strPathCorrespondenceFile);

    if (fMatches.fail())
    {
        //cout << "LoadFixedMap:\tNo global correspondences found for camera " << cameraID << endl;
        return false;
    }

    // Number of matches
    int nTotalMatches = 0;
    std::string line;
    while (std::getline(fMatches, line))
        ++nTotalMatches;

    // Adjust sample ratio if we have few matches
    double sampleProbability = max(sampleRatio, (double)nMinMatches / (double)nTotalMatches);

    // Reset stream
    fMatches.clear();
    fMatches.seekg(0);

    LocSLAM::Correspondence2d3d c;
    while ((fMatches >> c.imagePoint[0] >> c.imagePoint[1] >> c.mapPoint[0] >> c.mapPoint[1] >> c.mapPoint[2]) &&
           FrameFixedMapMatches.nMatches < nMaxMatches)
    {
        if (distribution(generator) <= sampleProbability)
        {
            FrameFixedMapMatches.vMatches.push_back(c);
            FrameFixedMapMatches.nMatches++;
        }
    }

    cout << "LoadFixedMap:\tFound " << FrameFixedMapMatches.nMatches << " fix map correspondences for camera " << cameraID << "!" << endl;

    // Load pose prior
    ifstream fPose;
    string strPathPoseFile = (fs::path(strBasePath) / fs::path(strPoseDir) / fs::path(strImageName + strPoseSuffix)).string();
    fPose.open(strPathPoseFile);

    if (fPose.fail())
    {
        cout << "LoadGlobalCorr: No pose prior found at: " << strPathPoseFile << endl;
        return false;
    }

    double q_w, q_x, q_y, q_z;
    Eigen::Vector3d vt_cf;

    fPose >> q_w >> q_x >> q_y >> q_z >> vt_cf[0] >> vt_cf[1] >> vt_cf[2];

    Eigen::Quaternion<double> qQ_cf(q_w, q_x, q_y, q_z);

    g2o::SE3Quat Tcf = g2o::SE3Quat(qQ_cf, vt_cf);
    g2o::SE3Quat Tfc = Tcf.inverse();

    FrameFixedMapMatches.Tcf = Tcf;

    return true;
}
