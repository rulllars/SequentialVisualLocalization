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

#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include <set>

#include <mutex>

namespace LocSLAM
{

    class MapPoint;
    class KeyFrame;

    class Map
    {
    public:
        Map(bool bFixedMap = false);

        void AddKeyFrame(KeyFrame *pKF);
        void AddMapPoint(MapPoint *pMP);
        void EraseMapPoint(MapPoint *pMP);
        void EraseKeyFrame(KeyFrame *pKF);
        void SetReferenceMapPoints(const std::vector<MapPoint *> &vpMPs);
        void InformNewBigChange();
        int GetLastBigChangeIdx();

        KeyFrame *GetLastBundledKeyFrame();
        void *SetLastBundledKeyFrame(KeyFrame *pKF);

        void SetLocalKeyFrames(list<KeyFrame *> lLocalKeyFrames);
        void SetLocalFixedKeyFrames(list<KeyFrame *> lLocalFixedKeyFrames);
        list<KeyFrame *> GetLocalKeyFrames();
        list<KeyFrame *> GetLocalFixedKeyFrames();

        void SetLocalMapPoints(vector<MapPoint *> lpLocalMapPoints);
        list<MapPoint *> GetLocalMapPoints();

        void PrintKeyFrameOdoConnections();

        std::vector<KeyFrame *> GetAllKeyFrames();
        std::vector<MapPoint *> GetAllMapPoints();
        std::vector<MapPoint *> GetReferenceMapPoints();

        long unsigned int MapPointsInMap();
        long unsigned KeyFramesInMap();
        long unsigned KeyFramesInMapForCamera(int cameraId);

        long unsigned int GetMaxKFid();

        void clear();

        const bool mbIsFixedMap;
        void SetTransformation(const cv::Mat &Tlf);
        cv::Mat GetTransformation();

        vector<KeyFrame *> mvpKeyFrameOrigins;

        std::recursive_mutex mMutexMapUpdate;

        // This avoid that two points are created simultaneously in separate threads (id conflict)
        std::recursive_mutex mMutexPointCreation;

        std::recursive_mutex mMutexMap;

    protected:
        std::set<MapPoint *> mspMapPoints;
        std::set<KeyFrame *> mspKeyFrames;

        std::vector<MapPoint *> mvpReferenceMapPoints;

        // Keep track of KFs and map points that are currently bundled
        KeyFrame *mpLastBundledKF;
        list<KeyFrame *> mlLocalKeyFrames;
        list<KeyFrame *> mlLocalFixedKeyFrames;
        list<MapPoint *> mlLocalMapPoints;

        long unsigned int mnMaxKFid;

        // Index related to a big change in the map (loop closure, global BA)
        int mnBigChangeIdx;

        cv::Mat mTlf;
    };

} // namespace LocSLAM

#endif // MAP_H
