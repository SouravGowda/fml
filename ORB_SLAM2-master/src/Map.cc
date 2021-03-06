/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Map.h"
#include "System.h"
#if PARALLEL_THREAD
#include<mutex>
#include<vector>
#endif
namespace ORB_SLAM2
{
#if EXECUTION_FLOW_DUMP
extern ofstream globalDataDump;
#endif
Map::Map():mnMaxKFid(0),mnBigChangeIdx(0)
{
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
#if EXECUTION_FLOW_DUMP
	globalDataDump<<"Map2:AddKeyFrame"<<"\n";
#endif
#if PARALLEL_THREAD
    unique_lock<mutex> lock(mMutexMap);
#endif
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
}

void Map::AddMapPoint(MapPoint *pMP)
{
#if PARALLEL_THREAD
    unique_lock<mutex> lock(mMutexMap);
#endif
    mspMapPoints.insert(pMP);
}

void Map::EraseMapPoint(MapPoint *pMP)
{
#if PARALLEL_THREAD
    unique_lock<mutex> lock(mMutexMap);
#endif
    mspMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
#if EXECUTION_FLOW_DUMP
	globalDataDump<<"Map5:EraseKeyFrame"<<"\n";
#endif
#if PARALLEL_THREAD
    unique_lock<mutex> lock(mMutexMap);
#endif
    mspKeyFrames.erase(pKF);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
#if EXECUTION_FLOW_DUMP
	globalDataDump<<"Map6:SetReferenceMapPoints"<<"\n";
#endif
#if PARALLEL_THREAD
   unique_lock<mutex> lock(mMutexMap);
#endif
    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
#if PARALLEL_THREAD
    unique_lock<mutex> lock(mMutexMap);
#endif
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
#if PARALLEL_THREAD
    unique_lock<mutex> lock(mMutexMap);
#endif
    return mnBigChangeIdx;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
#if EXECUTION_FLOW_DUMP
	globalDataDump<<"Map9:GetAllKeyFrames"<<"\n";
#endif
#if PARALLEL_THREAD
    unique_lock<mutex> lock(mMutexMap);
#endif
    return std::vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
#if EXECUTION_FLOW_DUMP
	globalDataDump<<"Map10:GetAllMapPoints"<<"\n";
#endif
#if PARALLEL_THREAD
    unique_lock<mutex> lock(mMutexMap);
#endif
    return std::vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap()
{
#if EXECUTION_FLOW_DUMP
	globalDataDump<<"Map11:MapPointsInMap"<<"\n";
#endif
#if PARALLEL_THREAD
    unique_lock<mutex> lock(mMutexMap);
#endif
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
#if EXECUTION_FLOW_DUMP
	globalDataDump<<"Map12:KeyFramesInMap"<<"\n";
#endif
#if PARALLEL_THREAD
    unique_lock<mutex> lock(mMutexMap);
#endif
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
#if EXECUTION_FLOW_DUMP
	globalDataDump<<"Map13:GetReferenceMapPoints"<<"\n";
#endif
#if PARALLEL_THREAD
    unique_lock<mutex> lock(mMutexMap);
#endif
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid()
{
#if EXECUTION_FLOW_DUMP
	globalDataDump<<"Map14:GetMaxKFid"<<"\n";
#endif
#if PARALLEL_THREAD
    unique_lock<mutex> lock(mMutexMap);
#endif
    return mnMaxKFid;
}

void Map::clear()
{
#if EXECUTION_FLOW_DUMP
	globalDataDump<<"Map15:clear"<<"\n";
#endif
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

} //namespace ORB_SLAM
