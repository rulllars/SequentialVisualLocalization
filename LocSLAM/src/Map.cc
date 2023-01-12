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

#include "Map.h"

#include <mutex>

namespace LocSLAM
{

    Map::Map(bool bFixedMap) : mnMaxKFid(0), mnBigChangeIdx(0), mbIsFixedMap(bFixedMap)
    {
        mpLastBundledKF = static_cast<KeyFrame *>(NULL);
    }

    void Map::AddKeyFrame(KeyFrame *pKF)
    {
        unique_lock<recursive_mutex> lock(mMutexMap);
        mspKeyFrames.insert(pKF);
        if (pKF->mnId > mnMaxKFid)
            mnMaxKFid = pKF->mnId;
    }

    void Map::AddMapPoint(MapPoint *pMP)
    {
        unique_lock<recursive_mutex> lock(mMutexMap);
        mspMapPoints.insert(pMP);
    }

    void Map::EraseMapPoint(MapPoint *pMP)
    {
        unique_lock<recursive_mutex> lock(mMutexMap);
        mspMapPoints.erase(pMP);

        //delete pMP;

        pMP = static_cast<MapPoint *>(NULL);
    }

    void Map::EraseKeyFrame(KeyFrame *pKF)
    {

        unique_lock<recursive_mutex> lock(mMutexMap);
        mspKeyFrames.erase(pKF);

        set<MapPoint *> spMPs = pKF->GetMapPoints();

        for (set<MapPoint *>::iterator it = spMPs.begin(); it != spMPs.end(); it++)
        {
            MapPoint *pMP = *it;
            pMP->EraseObservation(pKF);
        }
    }

    void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
    {
        unique_lock<recursive_mutex> lock(mMutexMap);
        mvpReferenceMapPoints = vpMPs;
    }

    void Map::InformNewBigChange()
    {
        unique_lock<recursive_mutex> lock(mMutexMap);
        mnBigChangeIdx++;
    }

    int Map::GetLastBigChangeIdx()
    {
        unique_lock<recursive_mutex> lock(mMutexMap);
        return mnBigChangeIdx;
    }

    KeyFrame *Map::GetLastBundledKeyFrame()
    {
        return mpLastBundledKF;
    }
    void *Map::SetLastBundledKeyFrame(KeyFrame *pKF)
    {
        mpLastBundledKF = pKF;
    }

    void Map::SetLocalKeyFrames(list<KeyFrame *> lLocalKeyFrames)
    {
        mlLocalKeyFrames = lLocalKeyFrames;
    }

    void Map::SetLocalFixedKeyFrames(list<KeyFrame *> lLocalFixedKeyFrames)
    {
        mlLocalFixedKeyFrames = lLocalFixedKeyFrames;
    }

    list<KeyFrame *> Map::GetLocalKeyFrames()
    {
        //unique_lock<recursive_mutex> lock(mMutexMap);
        list<KeyFrame *> localKeyFrames(mlLocalKeyFrames);
        return localKeyFrames;
    }

    list<KeyFrame *> Map::GetLocalFixedKeyFrames()
    {
        //unique_lock<recursive_mutex> lock(mMutexMap);
        list<KeyFrame *> localFixedKeyFrames(mlLocalFixedKeyFrames);
        return localFixedKeyFrames;
    }

    void Map::SetLocalMapPoints(vector<MapPoint *> vpMapPoints)
    {
        mlLocalMapPoints.assign(vpMapPoints.begin(), vpMapPoints.end());
    }

    list<MapPoint *> Map::GetLocalMapPoints()
    {
        unique_lock<recursive_mutex> lock(mMutexMap);
        return mlLocalMapPoints;
    }

    void Map::PrintKeyFrameOdoConnections()
    {
        std::set<KeyFrame *>::iterator it = mspKeyFrames.begin();

        // Iterate till the end of set
        while (it != mspKeyFrames.end())
        {
            KeyFrame *pKF = (*it);

            int currId = pKF->mnId;

            int nextId;
            if (pKF->GetNextKeyFrame() != NULL)
                nextId = pKF->GetNextKeyFrame()->mnId;
            else
                nextId = -1;

            // Print the element
            std::cout << "(" << currId << " , " << nextId << ")" << endl;
            //Increment the iterator
            it++;
        }
    }

    vector<KeyFrame *> Map::GetAllKeyFrames()
    {
        unique_lock<recursive_mutex> lock(mMutexMap);
        return vector<KeyFrame *>(mspKeyFrames.begin(), mspKeyFrames.end());
    }

    vector<MapPoint *> Map::GetAllMapPoints()
    {
        unique_lock<recursive_mutex> lock(mMutexMap);
        return vector<MapPoint *>(mspMapPoints.begin(), mspMapPoints.end());
    }

    long unsigned int Map::MapPointsInMap()
    {
        unique_lock<recursive_mutex> lock(mMutexMap);
        return mspMapPoints.size();
    }

    long unsigned int Map::KeyFramesInMap()
    {
        unique_lock<recursive_mutex> lock(mMutexMap);
        return mspKeyFrames.size();
    }

    long unsigned int Map::KeyFramesInMapForCamera(int cameraID)
    {
        unique_lock<recursive_mutex> lock(mMutexMap);

        std::set<KeyFrame *>::iterator it = mspKeyFrames.begin();

        // Iterate till the end of set
        unsigned long nKFs = 0;
        while (it != mspKeyFrames.end())
        {
            KeyFrame *pKFi = *it;

            if (pKFi->mCameraID == cameraID)
                nKFs++;

            it++;
        }

        return nKFs;
    }

    vector<MapPoint *> Map::GetReferenceMapPoints()
    {
        unique_lock<recursive_mutex> lock(mMutexMap);
        return mvpReferenceMapPoints;
    }

    long unsigned int Map::GetMaxKFid()
    {
        unique_lock<recursive_mutex> lock(mMutexMap);
        return mnMaxKFid;
    }

    void Map::clear()
    {
        for (set<MapPoint *>::iterator sit = mspMapPoints.begin(), send = mspMapPoints.end(); sit != send; sit++)
            delete *sit;

        for (set<KeyFrame *>::iterator sit = mspKeyFrames.begin(), send = mspKeyFrames.end(); sit != send; sit++)
            delete *sit;

        mspMapPoints.clear();
        mspKeyFrames.clear();
        mnMaxKFid = 0;
        mvpReferenceMapPoints.clear();
        mvpKeyFrameOrigins.clear();
    }

    void Map::SetTransformation(const cv::Mat &Tlf)
    {
        Tlf.copyTo(mTlf);
    }

    cv::Mat Map::GetTransformation()
    {
        return mTlf.clone();
    }

} // namespace LocSLAM
