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



#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>
/*#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>*/

namespace ORB_SLAM2
{
#if EXECUTION_FLOW_DUMP
 ofstream globalDataDump("SeqFlow19_1512Final.txt",ofstream::out);
#endif
#if FRAME_MAP_DUMP
int cnt = 0;
#endif
System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
               const bool bUseViewer):mSensor(sensor), mpViewer(static_cast<Viewer*>(NULL)), mbReset(false),mbActivateLocalizationMode(false),
        mbDeactivateLocalizationMode(false)
{
#if EXECUTION_FLOW_DUMP
	globalDataDump <<"System"<<"\n";
#endif
    // Output welcome message
    cout << endl <<
    "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it" << endl <<
    "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;

    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }


    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;

    //Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    //Create the Map
    mpMap = new Map();
#if VIEWER
    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpMap);
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);
#endif
    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpMap, mSensor==MONOCULAR);
#if PARALLEL_THREAD
    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run,mpLocalMapper);
#endif


#if LOOP_CLOSING
    //Initialize the Loop Closing thread and launch
    mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR);//commented by Reecha on 1/11/2019
#if PARALLEL_THREAD
    mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);
#endif
#endif

#if VIEWER
    //Initialize the Viewer thread and launch
    if(bUseViewer)
    {
        mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile);
#if PARALLEL_THREAD
        mptViewer = new thread(&Viewer::Run, mpViewer);
#endif
        mpViewer->Run(); //added by shweta on 18/10/19
        mpTracker->SetViewer(mpViewer);
    }
#endif

#if 1//PARALLEL_THREAD
    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
#if LOOP_CLOSING	
    mpTracker->SetLoopClosing(mpLoopCloser);//commented by Reecha on 1/11/2019
#endif
    mpLocalMapper->SetTracker(mpTracker);
#if LOOP_CLOSING		
    mpLocalMapper->SetLoopCloser(mpLoopCloser);//commented by Reecha on 1/11/2019

    mpLoopCloser->SetTracker(mpTracker);//commented by Reecha on 1/11/2019
    mpLoopCloser->SetLocalMapper(mpLocalMapper);//commented by Reecha on 1/11/2019
#endif
#endif
}

cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
{
#if EXECUTION_FLOW_DUMP
	globalDataDump<<"System2:TrackStereo"<<"\n";
#endif
    if(mSensor!=STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
        exit(-1);
    }   

    // Check mode change
    {
#if PARALLEL_THREAD
        unique_lock<mutex> lock(mMutexMode);
#endif
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
#if PARALLEL_THREAD
    unique_lock<mutex> lock(mMutexReset);
#endif
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft,imRight,timestamp);
#if PARALLEL_THREAD
    unique_lock<mutex> lock2(mMutexState);
#endif
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
{
#if EXECUTION_FLOW_DUMP
	globalDataDump<<"System3:TrackRGBD"<<"\n";
#endif
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }    

    // Check mode change
    {
#if PARALLEL_THREAD
        unique_lock<mutex> lock(mMutexMode);
#endif
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
#if PARALLEL_THREAD
    unique_lock<mutex> lock(mMutexReset);
#endif
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageRGBD(im,depthmap,timestamp);
#if PARALLEL_THREAD
    unique_lock<mutex> lock2(mMutexState);
#endif
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
#if EXECUTION_FLOW_DUMP
	globalDataDump<<"System4:TrackMonocular"<<"\n";
#endif
    if(mSensor!=MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }

    // Check mode change
    {
#if PARALLEL_THREAD
        unique_lock<mutex> lock(mMutexMode);
#endif
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();
            //mpLocalMapper->Stop();//added
#if 1//PARALLEL_THREAD
            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }
#endif
            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
#if PARALLEL_THREAD
    unique_lock<mutex> lock(mMutexReset);
#endif
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp);
#if FRAME_MAP_DUMP
    cnt++;
#endif	
#if PARALLEL_THREAD
    unique_lock<mutex> lock2(mMutexState);
#endif
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return Tcw;
}

void System::ActivateLocalizationMode()
{
#if PARALLEL_THREAD
    unique_lock<mutex> lock(mMutexMode);
#endif
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
#if EXECUTION_FLOW_DUMP
	globalDataDump<<"System 6:DeactivateLocalizationMode"<<"\n";
#endif
#if PARALLEL_THREAD
    unique_lock<mutex> lock(mMutexMode);
#endif
    mbDeactivateLocalizationMode = true;
}

bool System::MapChanged()
{
#if EXECUTION_FLOW_DUMP
	globalDataDump<<"System7:MapChanged"<<"\n";
#endif
    static int n=0;
    int curn = mpMap->GetLastBigChangeIdx();
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}

void System::Reset()
{
#if EXECUTION_FLOW_DUMP
	globalDataDump<<"System8:Reset"<<"\n";
#endif
#if PARALLEL_THREAD
    unique_lock<mutex> lock(mMutexReset);
#endif
    mbReset = true;
}

void System::Shutdown()
{
#if EXECUTION_FLOW_DUMP
	globalDataDump<<"System9:Shutdown"<<"\n";
#endif
#if PARALLEL_THREAD
    mpLocalMapper->RequestFinish();//commented by Reecha on 1/11/2019
#endif
#if VIEWER
    if(mpViewer)
    {
        mpViewer->RequestFinish();
        while(!mpViewer->isFinished())
            usleep(5000);
    }
#endif
    // Wait until all thread have effectively stopped   
#if PARALLEL_THREAD
#if LOOP_CLOSING
	while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())//commented by Reecha on 1/11/2019    
#else
	while(!mpLocalMapper->isFinished())
#endif	
    {
        usleep(5000);//System Reseting
    }
#endif
#if VIEWER
    if(mpViewer)
        pangolin::BindToContext("ORB-SLAM2: Map Viewer");
#endif
#if EXECUTION_FLOW_DUMP
    globalDataDump.close();
#endif
}

void System::SaveTrajectoryTUM(const string &filename)
{
#if EXECUTION_FLOW_DUMP
	globalDataDump<<"System10"<<"\n";
#endif
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = Converter::toQuaternion(Rwc);

        f << setprecision(6) << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

#if 0//original function definition
void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mnFrameId << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}
#endif
void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    vector<MapPoint*> vpMPs = mpMap->GetAllMapPoints();

    if(vpMPs.empty())
            return;

    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();


    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    //FILE *fpKF = fopen("Keyframe.csv","a");
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

       // cv::Mat R = pKF->GetRotation().t();
        cv::Mat R = pKF->GetRotation();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        //fprintf(fpKF,"%d,%d\n",pKF->mnFrameId,pKF->mnId);
        /*f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
*/
        f << setprecision(9)<< pKF->mnFrameId<<" " << R.at<float>(0,0) << " " << R.at<float>(0,1)  << " " << R.at<float>(0,2) << " "  << t.at<float>(0) << " " <<
                    R.at<float>(1,0) << " " << R.at<float>(1,1)  << " " << R.at<float>(1,2) << " "  << t.at<float>(1) << " " <<
                    R.at<float>(2,0) << " " << R.at<float>(2,1)  << " " << R.at<float>(2,2) << " "  << t.at<float>(2) << endl;

    }
    //fclose(fpKF);
    f.close();
#if 0
    FILE *fpMP = fopen("MapPoints.csv","a");
    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
        {
            if(vpMPs[i]->isBad())
                continue;
            cv::Mat pos = vpMPs[i]->GetWorldPos();
 #if 1//MAP
            fprintf(fpMP,"%f,%f,%f,%d,%d,%d,%d\n",pos.at<float>(0),pos.at<float>(1),pos.at<float>(2),vpMPs[i]->mnId,vpMPs[i]->mnFirstFrame,vpMPs[i]->mnFirstKFid,vpMPs[i]->nObs);
#endif

        }
    fclose(fpMP);
#endif
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveTrajectoryKITTI(const string &filename)
{
#if EXECUTION_FLOW_DUMP
	globalDataDump<<"System 12:SaveTrajectoryKITTI"<<"\n";
#endif
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM2::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
          //  cout << "bad parent" << endl;
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

int System::GetTrackingState()
{
#if EXECUTION_FLOW_DUMP
	globalDataDump<<"System 13:GetTrackingState"<<"\n";
#endif
#if PARALLEL_THREAD
    unique_lock<mutex> lock(mMutexState);
#endif
    return mTrackingState;
}

vector<MapPoint*> System::GetTrackedMapPoints()
{
#if EXECUTION_FLOW_DUMP
	globalDataDump<<"System 14:GetTrackedMapPoints"<<"\n";
#endif
#if PARALLEL_THREAD
    unique_lock<mutex> lock(mMutexState);
#endif
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
#if EXECUTION_FLOW_DUMP
	globalDataDump<<"System 15:GetTrackedKeyPointsUn"<<"\n";
#endif
#if PARALLEL_THREAD
    unique_lock<mutex> lock(mMutexState);
#endif
    return mTrackedKeyPointsUn;
}

} //namespace ORB_SLAM
