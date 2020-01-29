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

#include "MapPoint.h"
#include "ORBmatcher.h"
#include "System.h"
#if PARALLEL_THREAD
#include<mutex>
#endif
namespace ORB_SLAM2
{
#if EXECUTION_FLOW_DUMP
extern ofstream globalDataDump;
#endif
long unsigned int MapPoint::nNextId=0;
#if PARALLEL_THREAD
mutex MapPoint::mGlobalMutex;
#endif

MapPoint::MapPoint(const cv::Mat &Pos, KeyFrame *pRefKF, Map* pMap):
    mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(static_cast<MapPoint*>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap)
{
	//globalDataDump<<"MP1:MapPoint"<<"\n";
    Pos.copyTo(mWorldPos);
    mNormalVector = cv::Mat::zeros(3,1,CV_32F);
#if PARALLEL_THREAD
    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
#endif
    mnId=nNextId++;
}

MapPoint::MapPoint(const cv::Mat &Pos, Map* pMap, Frame* pFrame, const int &idxF):
    mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mnTrackReferenceForFrame(0), mnLastFrameSeen(0),
    mnBALocalForKF(0), mnFuseCandidateForKF(0),mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(static_cast<KeyFrame*>(NULL)), mnVisible(1),
    mnFound(1), mbBad(false), mpReplaced(NULL), mpMap(pMap)
{
#if EXECUTION_FLOW_DUMP
	globalDataDump<<"MP2:MapPoint"<<"\n";
#endif
    Pos.copyTo(mWorldPos);
    cv::Mat Ow = pFrame->GetCameraCenter();
    mNormalVector = mWorldPos - Ow;
    mNormalVector = mNormalVector/cv::norm(mNormalVector);

    cv::Mat PC = Pos - Ow;
    const float dist = cv::norm(PC);
    const int level = pFrame->mvKeysUn[idxF].octave;
    const float levelScaleFactor =  pFrame->mvScaleFactors[level];
    const int nLevels = pFrame->mnScaleLevels;

    mfMaxDistance = dist*levelScaleFactor;
    mfMinDistance = mfMaxDistance/pFrame->mvScaleFactors[nLevels-1];

    pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
#if PARALLEL_THREAD
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
#endif
    mnId=nNextId++;
}

void MapPoint::SetWorldPos(const cv::Mat &Pos)
{
	//globalDataDump<<"MP3:SetWorldPos"<<"\n";
#if PARALLEL_THREAD
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
#endif
    Pos.copyTo(mWorldPos);
}

cv::Mat MapPoint::GetWorldPos()
{
	//globalDataDump<<"MP4"<<"\n";//called many times
#if PARALLEL_THREAD
    unique_lock<mutex> lock(mMutexPos);
#endif
    return mWorldPos.clone();
}

cv::Mat MapPoint::GetNormal()
{
	//globalDataDump<<"MP5:GetNormal"<<"\n";
#if PARALLEL_THREAD
    unique_lock<mutex> lock(mMutexPos);
#endif
    return mNormalVector.clone();
}

KeyFrame* MapPoint::GetReferenceKeyFrame()
{
#if EXECUTION_FLOW_DUMP
	globalDataDump<<"MP6:GetReferenceKeyFrame"<<"\n";
#endif
#if PARALLEL_THREAD
    unique_lock<mutex> lock(mMutexFeatures);
#endif
    return mpRefKF;
}

void MapPoint::AddObservation(KeyFrame* pKF, size_t idx)
{
	//globalDataDump<<"MP7:AddObservation"<<"\n";
#if PARALLEL_THREAD
    unique_lock<mutex> lock(mMutexFeatures);
#endif
    if(mObservations.count(pKF))
        return;
    mObservations[pKF]=idx;

    if(pKF->mvuRight[idx]>=0)
        nObs+=2;
    else
        nObs++;
}

void MapPoint::EraseObservation(KeyFrame* pKF)
{
	//globalDataDump<<"MP8"<<"\n";
    bool bBad=false;
    {
#if PARALLEL_THREAD
        unique_lock<mutex> lock(mMutexFeatures);
#endif
        if(mObservations.count(pKF))
        {
            int idx = mObservations[pKF];
            if(pKF->mvuRight[idx]>=0)
                nObs-=2;
            else
                nObs--;

            mObservations.erase(pKF);

            if(mpRefKF==pKF)
                mpRefKF=mObservations.begin()->first;

            // If only 2 observations or less, discard point
            if(nObs<=2)
                bBad=true;
        }
    }

    if(bBad)
        SetBadFlag();
}

map<KeyFrame*, size_t> MapPoint::GetObservations()
{
#if PARALLEL_THREAD
    unique_lock<mutex> lock(mMutexFeatures);
#endif
    return mObservations;
}

int MapPoint::Observations()
{
#if PARALLEL_THREAD
    unique_lock<mutex> lock(mMutexFeatures);
#endif
    return nObs;
}

void MapPoint::SetBadFlag()
{
	//globalDataDump<<"MP11:SetBadFlag"<<"\n";
    map<KeyFrame*,size_t> obs;
    {
#if PARALLEL_THREAD
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
#endif
        mbBad=true;
        obs = mObservations;
        mObservations.clear();
    }
    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        pKF->EraseMapPointMatch(mit->second);
    }

    mpMap->EraseMapPoint(this);
}

MapPoint* MapPoint::GetReplaced()
{
#if PARALLEL_THREAD
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
#endif
    return mpReplaced;
}

void MapPoint::Replace(MapPoint* pMP)
{
	//globalDataDump<<"MP13:Replace"<<"\n";
    if(pMP->mnId==this->mnId)
        return;

    int nvisible, nfound;
    map<KeyFrame*,size_t> obs;
    {
#if PARALLEL_THREAD
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
#endif
        obs=mObservations;
        mObservations.clear();
        mbBad=true;
        nvisible = mnVisible;
        nfound = mnFound;
        mpReplaced = pMP;
    }

    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        // Replace measurement in keyframe
        KeyFrame* pKF = mit->first;

        if(!pMP->IsInKeyFrame(pKF))
        {
            pKF->ReplaceMapPointMatch(mit->second, pMP);
            pMP->AddObservation(pKF,mit->second);
        }
        else
        {
            pKF->EraseMapPointMatch(mit->second);
        }
    }
    pMP->IncreaseFound(nfound);
    pMP->IncreaseVisible(nvisible);
    pMP->ComputeDistinctiveDescriptors();

    mpMap->EraseMapPoint(this);
}

bool MapPoint::isBad()
{
#if PARALLEL_THREAD
    unique_lock<mutex> lock(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
#endif
    return mbBad;
}

void MapPoint::IncreaseVisible(int n)
{
	//globalDataDump<<"MP15:IncreaseVisible"<<"\n";
#if PARALLEL_THREAD
    unique_lock<mutex> lock(mMutexFeatures);
#endif
    mnVisible+=n;
}

void MapPoint::IncreaseFound(int n)
{
	//globalDataDump<<"MP16:IncreaseFound"<<"\n";
#if PARALLEL_THREAD
    unique_lock<mutex> lock(mMutexFeatures);
#endif
    mnFound+=n;
}

float MapPoint::GetFoundRatio()
{
	//globalDataDump<<"MP16:GetFoundRatio"<<"\n";
#if PARALLEL_THREAD
    unique_lock<mutex> lock(mMutexFeatures);
#endif
    return static_cast<float>(mnFound)/mnVisible;
}

void MapPoint::ComputeDistinctiveDescriptors()
{
	//globalDataDump<<"MP18:ComputeDistinctiveDescriptors"<<"\n";
    // Retrieve all observed descriptors
    vector<cv::Mat> vDescriptors;

    map<KeyFrame*,size_t> observations;

    {
#if PARALLEL_THREAD
        unique_lock<mutex> lock1(mMutexFeatures);
#endif
        if(mbBad)
            return;
        observations=mObservations;
    }

    if(observations.empty())
        return;

    vDescriptors.reserve(observations.size());

    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;

        if(!pKF->isBad())
            vDescriptors.push_back(pKF->mDescriptors.row(mit->second));
    }

    if(vDescriptors.empty())
        return;

    // Compute distances between them
    const size_t N = vDescriptors.size();

    float Distances[N][N];
    for(size_t i=0;i<N;i++)
    {
        Distances[i][i]=0;
        for(size_t j=i+1;j<N;j++)
        {
            int distij = ORBmatcher::DescriptorDistance(vDescriptors[i],vDescriptors[j]);
            Distances[i][j]=distij;
            Distances[j][i]=distij;
        }
    }

    // Take the descriptor with least median distance to the rest
    int BestMedian = INT_MAX;
    int BestIdx = 0;
    for(size_t i=0;i<N;i++)
    {
        vector<int> vDists(Distances[i],Distances[i]+N);
        sort(vDists.begin(),vDists.end());
        int median = vDists[0.5*(N-1)];

        if(median<BestMedian)
        {
            BestMedian = median;
            BestIdx = i;
        }
    }

    {
#if PARALLEL_THREAD
        unique_lock<mutex> lock(mMutexFeatures);
#endif
        mDescriptor = vDescriptors[BestIdx].clone();
    }
}

cv::Mat MapPoint::GetDescriptor()
{
	//globalDataDump<<"MP19:GetDescriptor"<<"\n";
#if PARALLEL_THREAD
    unique_lock<mutex> lock(mMutexFeatures);
#endif
    return mDescriptor.clone();
}

int MapPoint::GetIndexInKeyFrame(KeyFrame *pKF)
{
#if EXECUTION_FLOW_DUMP
	globalDataDump<<"MP20:GetIndexInKeyFrame"<<"\n";
#endif
#if PARALLEL_THREAD
    unique_lock<mutex> lock(mMutexFeatures);
#endif
    if(mObservations.count(pKF))
        return mObservations[pKF];
    else
        return -1;
}

bool MapPoint::IsInKeyFrame(KeyFrame *pKF)
{
	//globalDataDump<<"MP21:IsInKeyFrame"<<"\n";
#if PARALLEL_THREAD
    unique_lock<mutex> lock(mMutexFeatures);
#endif
    return (mObservations.count(pKF));
}

void MapPoint::UpdateNormalAndDepth()
{
	//globalDataDump<<"MP22:UpdateNormalAndDepth"<<"\n";
	map<KeyFrame*,size_t> observations;
    KeyFrame* pRefKF;
    cv::Mat Pos;
    {
#if PARALLEL_THREAD
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
#endif
        if(mbBad)
            return;
        observations=mObservations;
        pRefKF=mpRefKF;
        Pos = mWorldPos.clone();
    }

    if(observations.empty())
        return;

    cv::Mat normal = cv::Mat::zeros(3,1,CV_32F);
    int n=0;
    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        cv::Mat Owi = pKF->GetCameraCenter();
        cv::Mat normali = mWorldPos - Owi;
        normal = normal + normali/cv::norm(normali);
        n++;
    }

    cv::Mat PC = Pos - pRefKF->GetCameraCenter();
    const float dist = cv::norm(PC);
    const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
    const float levelScaleFactor =  pRefKF->mvScaleFactors[level];
    const int nLevels = pRefKF->mnScaleLevels;

    {
#if PARALLEL_THREAD
        unique_lock<mutex> lock3(mMutexPos);
#endif
        mfMaxDistance = dist*levelScaleFactor;
        mfMinDistance = mfMaxDistance/pRefKF->mvScaleFactors[nLevels-1];
        mNormalVector = normal/n;
    }
}

float MapPoint::GetMinDistanceInvariance()
{
#if PARALLEL_THREAD
    unique_lock<mutex> lock(mMutexPos);
#endif
    return 0.8f*mfMinDistance;
}

float MapPoint::GetMaxDistanceInvariance()
{
#if PARALLEL_THREAD
    unique_lock<mutex> lock(mMutexPos);
#endif
    return 1.2f*mfMaxDistance;
}

int MapPoint::PredictScale(const float &currentDist, KeyFrame* pKF)
{
    float ratio;
    {
#if PARALLEL_THREAD
        unique_lock<mutex> lock(mMutexPos);
#endif
        ratio = mfMaxDistance/currentDist;
    }

    int nScale = ceil(log(ratio)/pKF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pKF->mnScaleLevels)
        nScale = pKF->mnScaleLevels-1;

    return nScale;
}

int MapPoint::PredictScale(const float &currentDist, Frame* pF)
{
    float ratio;
    {
#if PARALLEL_THREAD
        unique_lock<mutex> lock(mMutexPos);
#endif
        ratio = mfMaxDistance/currentDist;
    }

    int nScale = ceil(log(ratio)/pF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pF->mnScaleLevels)
        nScale = pF->mnScaleLevels-1;

    return nScale;
}



} //namespace ORB_SLAM
