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

#include <thread>
#include <iomanip>

#include "System.h"
#include "Converter.h"
#include "Tracking.h"
#include "LocalMapping.h"

#include "IFrameSubscriber.h"
#include "IFrameDrawer.h"
#include "IMapPublisher.h"
#include "Failure.h"

#include "utils.h"

#include <unistd.h>

namespace ORB_SLAM2
{

bool has_suffix(const std::string &str, const std::string &suffix) {
  std::size_t index = str.find(suffix, str.size() - suffix.size());
  return (index != std::string::npos);
}

System::IBuilder::~IBuilder() { }

System::GenericBuilder::GenericBuilder(const std::string &strVocFile,
                                       const std::string &strSettingsFile,
                                       eSensor sensor) :
    mSensor(sensor),
    mSettings(strSettingsFile.c_str(), cv::FileStorage::READ)
{

    if (!mSettings.isOpened())
        throw Failure(std::string("Failed to open settings file at: ") + strSettingsFile);

    clock_t tStart = clock();
    if (has_suffix(strVocFile, ".txt")) {

      if ( !mVocabulary.loadFromTextFile(strVocFile)) {
            throw Failure(std::string("Failed to load vocabulary at: ") + strVocFile);
      }

    } else {

      if ( !mVocabulary.loadFromBinaryFile(strVocFile)) {
            throw Failure(std::string("Failed to load vocabulary at: ") + strVocFile);
      }

    }
    cout << "Vocabulary loaded in: " << ((double)(clock() - tStart)/CLOCKS_PER_SEC) << " s" << endl;

    mpKeyFrameDatabase = make_unique<KeyFrameDatabase>(mVocabulary);
    mpMap = make_unique<Map>();
    mpTracker = make_unique<Tracking>(&mVocabulary, mpMap.get(), mpKeyFrameDatabase.get(), strSettingsFile, mSensor);
    mpLocalMapper = make_unique<LocalMapping>(mpMap.get(), mSensor == MONOCULAR);
    mpLoopCloser = make_unique<LoopClosing>(mpMap.get(), mpKeyFrameDatabase.get(), &mVocabulary, mSensor != MONOCULAR);
}

System::GenericBuilder::~GenericBuilder() { }

System::eSensor   System::GenericBuilder::GetSensorType() { return mSensor; }
ORBVocabulary*    System::GenericBuilder::GetVocabulary() { return &mVocabulary; }
KeyFrameDatabase* System::GenericBuilder::GetKeyFrameDatabase() { return mpKeyFrameDatabase.get(); }
Map*              System::GenericBuilder::GetMap() { return mpMap.get(); }
Tracking*         System::GenericBuilder::GetTracker() { return mpTracker.get(); }
LocalMapping*     System::GenericBuilder::GetLocalMapper() { return mpLocalMapper.get(); }
LoopClosing*      System::GenericBuilder::GetLoopCloser() { return mpLoopCloser.get(); }


System::System(std::unique_ptr<IBuilder> builder)
    : mpBuilder(std::move(builder)),
      mSensor(mpBuilder->GetSensorType()),
      mbStarted(false), mbReset(false),
      mbActivateLocalizationMode(false),
      mbDeactivateLocalizationMode(false)
{
    if (!mpBuilder)
        throw std::invalid_argument("builder shouldn't be null");

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

    mpVocabulary = mpBuilder->GetVocabulary();
    mpKeyFrameDatabase = mpBuilder->GetKeyFrameDatabase();
    mpMap = mpBuilder->GetMap();
    mpTracker = mpBuilder->GetTracker();
    mpLocalMapper = mpBuilder->GetLocalMapper();
    mpLoopCloser = mpBuilder->GetLoopCloser();
    mpPublisher = mpBuilder->GetPublisher();

    // added to publisher
    mpPublisher->SetLoopCloser(mpLoopCloser);

    mpTracker->SetPublisherThread(mpPublisher);

    mpPublisher->SetSystem(this);
	
    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);
}

void System::Start()
{
    if (mbStarted)
        return;

    mtLocalMapping = std::thread(&ORB_SLAM2::LocalMapping::Run, mpLocalMapper);
    mtLoopClosing = std::thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);
    mtPublisher = std::thread(&ORB_SLAM2::IPublisherThread::Run, mpPublisher);
    // mtCamPublisher = std::thread(&ORB_SLAM2::IPublisherThread::RunCam, mpPublisher);
    mbStarted = true;
}

cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
{

    if(mSensor!=STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
        std::exit(-1);
    }   

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();
            // Wait until Local Mapping has effectively stopped
            // !!!!!!!!!!!! mtLocalMapping.join(); CRASH HERE
            std::this_thread::sleep_for(std::chrono::milliseconds(1500));

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
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft,imRight,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
{
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        std::exit(-1);
    }    

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            // !!!!!!!!!!!! mtLocalMapping.join(); CRASH HERE
            std::this_thread::sleep_for(std::chrono::milliseconds(1500));

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
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageRGBD(im,depthmap,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
    if(mSensor!=MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        std::exit(-1);
    }
    // cout << "System.cc - TrackMonocular" << endl;
    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            // cout << "System.cc - if | 1" << endl;
            mpLocalMapper->RequestStop();
            // cout << "System.cc - if | 2" << endl;
            // Wait until Local Mapping has effectively stopped
            // !!!!!!!!!!!! mtLocalMapping.join(); CRASH HERE
            std::this_thread::sleep_for(std::chrono::milliseconds(1500));
            // cout << "System.cc - if | 3" << endl;

            mpTracker->InformOnlyTracking(true);
            // cout << "System.cc - if | 4" << endl;
            mbActivateLocalizationMode = false;
            // cout << "System.cc - if | 5" << endl;
        }
        // cout << "System.cc - after if" << endl;
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
        // cout << "System.cc - after deact" << endl;
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        // cout << "System.cc - bef mpTracker->Reset()" << endl;
        mpTracker->Reset();
        mbReset = false;
    }
    }

    // cout << "System.cc - bef GrabImgMono" << endl;
    cv::Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    // cout << "System.cc - bef return Tcw" << endl;
    return Tcw;
}

void System::ActivateLocalizationMode()
{
    cout << "Localization mode activated!" << endl;
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    cout << "Mapping again!" << endl;
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

bool System::MapChanged()
{
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
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::Shutdown()
{
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    mpPublisher->RequestFinish();

    // Wait until all thread have effectively stopped
    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished()  ||
          !mpPublisher->isFinished()      || mpLoopCloser->isRunningGBA())
    {
        usleep(5000);
    }
}

void System::SaveTrajectoryTUM(const string &filename)
{
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
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveTrajectoryKITTI(const string &filename)
{
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
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

vector<MapPoint*> System::GetTrackedMapPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}


} //namespace ORB_SLAM
