/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include "MapPoint.h"
#include "ORBmatcher.h"

#include<mutex>
#include <chrono>
#include <ctime>
#include<random>

namespace ORB_SLAM3
{

long unsigned int MapPoint::nNextId=0;
mutex MapPoint::mGlobalMutex;

MapPoint::MapPoint():
    mnFirstKFid(0), mnFirstFrame(0), nObs(0), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(static_cast<MapPoint*>(NULL)),
    mnUpdateCounter(0), mUpdateTimeStamp(0)
{
    ++nNextId;
    mpReplaced = static_cast<MapPoint*>(NULL);
    mpRefKF = static_cast<KeyFrame*>(NULL);
    mpHostKF = static_cast<KeyFrame*>(NULL);
}

MapPoint::MapPoint(const Eigen::Vector3f &Pos, KeyFrame *pRefKF, Map* pMap):
    mnFirstKFid(pRefKF->mnId), mpHostKF(static_cast<KeyFrame*>(NULL)), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(static_cast<MapPoint*>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap),
    mnOriginMapId(pMap->GetId()),
    mnUpdateCounter(0), mUpdateTimeStamp(0)
{
    SetWorldPos(Pos);

    mNormalVector.setZero();

    mbTrackInViewR = false;
    mbTrackInView = false;

    mpHostKF = static_cast<KeyFrame*>(NULL);
    
    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;
    
    // Get the current time point
    //auto currentTimePoint = std::chrono::system_clock::now();
    // Convert the time point to a time_t (seconds since epoch)
    //std::time_t currentTime = std::chrono::system_clock::to_time_t(currentTimePoint);
    // Convert time_t to an integer
    //int currentTimestamp = static_cast<int>(currentTime);   

    const char* cSystemId = std::getenv("SLAM_SYSTEM_ID");
    std::string strSystemId(cSystemId);
    mstrHexId=createHashId("", mnId);
    //std::cout << "MP Constructor0: Hash=" << mstrHexId << ", " << createHashId("main", mnId) << std::endl;
    pMap->notifyNewMapPointCreated(this);
}

MapPoint::MapPoint(const double invDepth, cv::Point2f uv_init, KeyFrame* pRefKF, KeyFrame* pHostKF, Map* pMap):
    mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(static_cast<MapPoint*>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap),
    mnOriginMapId(pMap->GetId()),
    mnUpdateCounter(0), mUpdateTimeStamp(0)
{
    mInvDepth=invDepth;
    mInitU=(double)uv_init.x;
    mInitV=(double)uv_init.y;
    mpHostKF = (pHostKF) ? pHostKF : static_cast<KeyFrame*>(NULL);

    mNormalVector.setZero();
    
    
    // Worldpos is not set
    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;
    
    // Get the current time point
    //auto currentTimePoint = std::chrono::system_clock::now();
    // Convert the time point to a time_t (seconds since epoch)
    //std::time_t currentTime = std::chrono::system_clock::to_time_t(currentTimePoint);
    // Convert time_t to an integer
    //int currentTimestamp = static_cast<int>(currentTime);   
    
    const char* cSystemId = std::getenv("SLAM_SYSTEM_ID");
    std::string strSystemId(cSystemId);
    mstrHexId=createHashId("", mnId);
    //std::cout << "MP Constructor1: Hash=" << mstrHexId << std::endl;
    pMap->notifyNewMapPointCreated(this);

}

MapPoint::MapPoint(const Eigen::Vector3f &Pos, Map* pMap, Frame* pFrame, const int &idxF):
    mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mnTrackReferenceForFrame(0), mnLastFrameSeen(0),
    mnBALocalForKF(0), mnFuseCandidateForKF(0),mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(static_cast<KeyFrame*>(NULL)), mnVisible(1),
    mnFound(1), mbBad(false), mpReplaced(NULL), mpMap(pMap), mnOriginMapId(pMap->GetId()),
    mnUpdateCounter(0), mUpdateTimeStamp(0)
{
    SetWorldPos(Pos);

    Eigen::Vector3f Ow;
    if(pFrame -> Nleft == -1 || idxF < pFrame -> Nleft){
        Ow = pFrame->GetCameraCenter();
    }
    else{
        Eigen::Matrix3f Rwl = pFrame->GetRwc();
        Eigen::Vector3f tlr = pFrame->GetRelativePoseTlr().translation();
        Eigen::Vector3f twl = pFrame->GetOw();

        Ow = Rwl * tlr + twl;
    }
    mNormalVector = mWorldPos - Ow;
    mNormalVector = mNormalVector / mNormalVector.norm();

    Eigen::Vector3f PC = mWorldPos - Ow;
    const float dist = PC.norm();
    const int level = (pFrame -> Nleft == -1) ? pFrame->mvKeysUn[idxF].octave
                                              : (idxF < pFrame -> Nleft) ? pFrame->mvKeys[idxF].octave
                                                                         : pFrame -> mvKeysRight[idxF].octave;
    const float levelScaleFactor =  pFrame->mvScaleFactors[level];
    const int nLevels = pFrame->mnScaleLevels;

    mfMaxDistance = dist*levelScaleFactor;
    mfMinDistance = mfMaxDistance/pFrame->mvScaleFactors[nLevels-1];

    pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

    mpHostKF = static_cast<KeyFrame*>(NULL);
    
    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++; 
    
    // Get the current time point
    //auto currentTimePoint = std::chrono::system_clock::now();
    // Convert the time point to a time_t (seconds since epoch)
    //std::time_t currentTime = std::chrono::system_clock::to_time_t(currentTimePoint);
    // Convert time_t to an integer
    //int currentTimestamp = static_cast<int>(currentTime);   
    
    const char* cSystemId = std::getenv("SLAM_SYSTEM_ID");
    std::string strSystemId(cSystemId);
    mstrHexId=createHashId("", mnId);
    //std::cout << "MP Constructor2: Hash=" << mstrHexId << std::endl;
    pMap->notifyNewMapPointCreated(this);
    
}

// Copy constructor
MapPoint::MapPoint(const MapPoint& mp):   
    mnId(mp.mnId), mstrHexId(mp.mstrHexId), mnFirstKFid(mp.mnFirstKFid), mnFirstFrame(mp.mnFirstFrame), nObs(mp.nObs), mTrackProjX(mp.mTrackProjX), 
    mTrackProjY(mp.mTrackProjY), mTrackDepth(mp.mTrackDepth), mTrackDepthR(mp.mTrackDepthR), mTrackProjXR(mp.mTrackProjXR), mTrackProjYR(mp.mTrackProjYR), 
    mbTrackInView(mp.mbTrackInView), mbTrackInViewR(mp.mbTrackInViewR), mnTrackScaleLevel(mp.mnTrackScaleLevel), mnTrackScaleLevelR(mp.mnTrackScaleLevelR), 
    mTrackViewCos(mp.mTrackViewCos), mTrackViewCosR(mp.mTrackViewCosR), mnTrackReferenceForFrame(mp.mnTrackReferenceForFrame), mnLastFrameSeen(mp.mnLastFrameSeen), 
    mnBALocalForKF(mp.mnBALocalForKF), mnFuseCandidateForKF(mp.mnFuseCandidateForKF), mnLoopPointForKF(mp.mnLoopPointForKF), mnCorrectedByKF(mp.mnCorrectedByKF), 
    mnCorrectedReference(mp.mnCorrectedReference), mPosGBA(mp.mPosGBA), mnBAGlobalForKF(mp.mnBAGlobalForKF), mnBALocalForMerge(mp.mnBALocalForMerge), 
    mPosMerge(mp.mPosMerge), mNormalVectorMerge(mp.mNormalVectorMerge), mInvDepth(mp.mInvDepth), mInitU(mp.mInitU), mInitV(mp.mInitV), mpHostKF(mp.mpHostKF), 
    mBackupHostKFId(mp.mBackupHostKFId), mnOriginMapId(mp.mnOriginMapId), mWorldPos(mp.mWorldPos), /*mObservations(mp.mObservations),*/ 
    /*mBackupObservationsId1(mp.mBackupObservationsId1), mBackupObservationsId2(mp.mBackupObservationsId2),*/ mNormalVector(mp.mNormalVector), 
    /*mDescriptor(mp.mDescriptor.clone()),*/ mpRefKF(mp.mpRefKF), mBackupRefKFId(mp.mBackupRefKFId), mnVisible(mp.mnVisible), mnFound(mp.mnFound), mbBad(mp.mbBad), 
    mpReplaced(mp.mpReplaced), /*mBackupReplacedId(mp.mBackupReplacedId),*/ mBackupReplacedStrId(mp.mBackupReplacedStrId), 
    mfMinDistance(mp.mfMinDistance), mfMaxDistance(mp.mfMaxDistance), mpMap(mp.mpMap), mnLastModule(mp.mnLastModule)
{
    //unique_lock<mutex> lock1(mMutexModule);
    //unique_lock<mutex> lock3(mMutexMap);
    //unique_lock<mutex> lock4(mMutexPos);
    //unique_lock<mutex> lock4(mGlobalMutex,std::defer_lock);
    
    //lock(lock1, lock2, lock3, lock4);

    {
        //unique_lock<mutex> lock2(mMutexFeatures);
        map<KeyFrame*, std::tuple<int,int>> mObs(mp.mObservations);
        for(map<KeyFrame*, std::tuple<int,int>>::const_iterator it = mObs.begin(), end = mObs.end(); it != end; ++it)
        {
          //std::cout << it->first->mnId << "," << get<0>(it->second) << std::endl;
          if(it->first)
          {
              mObservations[it->first] = it->second;
              mBackupObservationsId1[it->first->mnId] = get<0>(it->second); 
              mBackupObservationsId2[it->first->mnId] = get<1>(it->second); 
          }
        }

        if(!mp.mDescriptor.empty())
            mDescriptor=mp.mDescriptor.clone();

    }

}

MapPoint::MapPoint(const long long int mnId_, const std::string mstrHexId, const long int mnFirstKFid, const long int mnFirstFrame, const int nObs, /*const float mTrackProjX,
    const float mTrackProjY, const float mTrackDepth, const float mTrackDepthR, const float mTrackProjXR, const float mTrackProjYR, const bool mbTrackInView, const bool mbTrackInViewR,
    const int mnTrackScaleLevel, const int mnTrackScaleLevelR, const float mTrackViewCos, const float mTrackViewCosR, const long unsigned int mnTrackReferenceForFrame,
    const long unsigned int mnLastFrameSeen, const long unsigned int mnBALocalForKF, const long unsigned int mnFuseCandidateForKF, const long unsigned int mnLoopPointForKF,
    const long unsigned int mnCorrectedByKF, const long unsigned int mnCorrectedReference, const Eigen::Vector3f mPosGBA, const long unsigned int mnBAGlobalForKF,
    const long unsigned int mnBALocalForMerge, const Eigen::Vector3f mPosMerge, const Eigen::Vector3f mNormalVectorMerge,*/ const double mInvDepth, const double mInitU,
    const double mInitV, /*KeyFrame* mpHostKF = nullptr,*/ const long long int mBackupHostKFId, const unsigned int mnOriginMapId, const Eigen::Vector3f mWorldPos,
    /*std::map<KeyFrame*,std::tuple<int,int> > mObservations = std::map<KeyFrame*,std::tuple<int,int> >(),*/
    const std::map<long unsigned int, int>& mBackupObservationsId1, const std::map<long unsigned int, int>& mBackupObservationsId2, const Eigen::Vector3f mNormalVector,
    /*const cv::Mat& mDescriptor,*/ /*KeyFrame* mpRefKF = nullptr,*/ const long long int mBackupRefKFId, const int mnVisible, const int mnFound, const bool mbBad,
    /*MapPoint* mpReplaced = nullptr,*/ /*long long int mBackupReplacedId = -1,*/ const std::string mBackupReplacedStrId, const float mfMinDistance,
    const float mfMaxDistance, /*Map* mpMap = nullptr*/ const unsigned int mnLastModule):
    /*mnId(mnId),*/ mstrHexId(mstrHexId), mnFirstKFid(mnFirstKFid), mnFirstFrame(mnFirstFrame), nObs(nObs), /*mTrackProjX(mTrackProjX), 
    mTrackProjY(mTrackProjY), mTrackDepth(mTrackDepth), mTrackDepthR(mTrackDepthR), mTrackProjXR(mTrackProjXR), mTrackProjYR(mTrackProjYR), 
    mbTrackInView(mbTrackInView), mbTrackInViewR(mbTrackInViewR), mnTrackScaleLevel(mnTrackScaleLevel), mnTrackScaleLevelR(mnTrackScaleLevelR), 
    mTrackViewCos(mTrackViewCos), mTrackViewCosR(mTrackViewCosR), mnTrackReferenceForFrame(mnTrackReferenceForFrame), mnLastFrameSeen(mnLastFrameSeen), 
    mnBALocalForKF(mnBALocalForKF), mnFuseCandidateForKF(mnFuseCandidateForKF), mnLoopPointForKF(mnLoopPointForKF), mnCorrectedByKF(mnCorrectedByKF), 
    mnCorrectedReference(mnCorrectedReference), mPosGBA(mPosGBA), mnBAGlobalForKF(mnBAGlobalForKF), mnBALocalForMerge(mnBALocalForMerge), 
    mPosMerge(mPosMerge), mNormalVectorMerge(mNormalVectorMerge),*/ mInvDepth(mInvDepth), mInitU(mInitU), mInitV(mInitV), /*mpHostKF(mpHostKF),*/ 
    mBackupHostKFId(mBackupHostKFId), mnOriginMapId(mnOriginMapId), mWorldPos(mWorldPos), /*mObservations(mObservations),*/ 
    mBackupObservationsId1(mBackupObservationsId1), mBackupObservationsId2(mBackupObservationsId2), mNormalVector(mNormalVector), 
    /*mDescriptor(mDescriptor.clone()),*/ /*mpRefKF(mpRefKF),*/ mBackupRefKFId(mBackupRefKFId), mnVisible(mnVisible), mnFound(mnFound), mbBad(mbBad), 
    /*mpReplaced(mpReplaced),*/ /*mBackupReplacedId(mBackupReplacedId),*/ mBackupReplacedStrId(mBackupReplacedStrId), 
    mfMinDistance(mfMinDistance), mfMaxDistance(mfMaxDistance) /*mpMap(mpMap)*/, mnLastModule(mnLastModule),
    mnUpdateCounter(0), mUpdateTimeStamp(0)
{
    //SetWorldPos(Pos);
    
    mpReplaced = static_cast<MapPoint*>(NULL);
    mpHostKF = static_cast<KeyFrame*>(NULL);
    mpRefKF= static_cast<KeyFrame*>(NULL);
    mpMap = static_cast<Map*>(NULL);
    //mNormalVector.setZero();

    //mbTrackInViewR = false;
    //mbTrackInView = false;
    
    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    //unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    if(mnId_ > nNextId)
    {
      nNextId=mnId_;
      ++nNextId;
    }
    
    mnId=mnId_;
    
    //mnId=nNextId++;
    //mstrHexId=createHashId("sub", nNextId)
    //std::cout << "MP ROS Constructor: nNextId=" << nNextId << "," << mnId << ", Hash=" << mstrHexId << std::endl;
}

void MapPoint::UpdateMapPoint(const long int mnFirstKFid_, const long int mnFirstFrame_, const int nObs_, /*float mTrackProjX_, float mTrackProjY_, float mTrackDepth_,*/
    /* float mTrackDepthR_, float mTrackProjXR_, float mTrackProjYR_, bool mbTrackInView_, bool mbTrackInViewR_, int mnTrackScaleLevel_,*/
    /* int mnTrackScaleLevelR_, float mTrackViewCos_, float mTrackViewCosR_, long unsigned int mnTrackReferenceForFrame_, long unsigned int mnLastFrameSeen_,*/ 
    /*const long unsigned int mnBALocalForKF_, const long unsigned int mnFuseCandidateForKF_, const long unsigned int mnLoopPointForKF_, const long unsigned int mnCorrectedByKF_, 
    const long unsigned int mnCorrectedReference_, const Eigen::Vector3f mPosGBA_, const long unsigned int mnBAGlobalForKF_, const long unsigned int mnBALocalForMerge_, 
    const Eigen::Vector3f mPosMerge_, const Eigen::Vector3f mNormalVectorMerge_,*/ const double mInvDepth_, const double mInitU_, const double mInitV_, /*KeyFrame* mpHostKF = nullptr, */
    const long long int mBackupHostKFId_, const unsigned int mnOriginMapId_, const Eigen::Vector3f mWorldPos_, 
    /*std::map<KeyFrame*,std::tuple<int,int> > mObservations = std::map<KeyFrame*,std::tuple<int,int> >(), */const std::map<long unsigned int, int>& mBackupObservationsId1_, 
    const std::map<long unsigned int, int>& mBackupObservationsId2_, const Eigen::Vector3f mNormalVector_, /*const cv::Mat& mDescriptor_,*/ /*KeyFrame* mpRefKF = nullptr,*/
    const long long int mBackupRefKFId_,const int mnVisible_,const int mnFound_,const bool mbBad_, /*MapPoint* mpReplaced = nullptr, */ /*long long int mBackupReplacedId = -1, */
    const std::string mBackupReplacedStrId_, const float mfMinDistance_, const float mfMaxDistance_ /*Map* mpMap = nullptr */, const unsigned int mnLastModule_) {
    
    SetWorldPos(mWorldPos_);

    //:unique_lock<mutex> lock1(mMutexModule);
    //unique_lock<mutex> lock3(mMutexMap);
    //unique_lock<mutex> lock4(mMutexPos);
    //unique_lock<mutex> lock4(mGlobalMutex, std::defer_lock);
    
    //lock(lock1, /*lock2,*/ lock3, lock4);

    //mnFirstKFid               = mnFirstKFid_;
    //mnFirstFrame              = mnFirstFrame_;
    nObs                      = nObs_;
    //mTrackProjX               = mTrackProjX_;
    //mTrackProjY               = mTrackProjY_;
    //mTrackDepth               = mTrackDepth_;
    //mTrackDepthR              = mTrackDepthR_;
    //mTrackProjXR              = mTrackProjXR_;
    //mTrackProjYR              = mTrackProjYR_;
    //mbTrackInView             = mbTrackInView_;
    //mbTrackInViewR            = mbTrackInViewR_;
    //mnTrackScaleLevel         = mnTrackScaleLevel_;
    //mnTrackScaleLevelR        = mnTrackScaleLevelR_;
    //mTrackViewCos             = mTrackViewCos_;
    //mTrackViewCosR            = mTrackViewCosR_;
    //mnTrackReferenceForFrame  = mnTrackReferenceForFrame_;
    //mnLastFrameSeen           = mnLastFrameSeen_;
    //mnBALocalForKF            = mnBALocalForKF_;
    //mnFuseCandidateForKF      = mnFuseCandidateForKF_;
    //mnLoopPointForKF          = mnLoopPointForKF_;
    //mnCorrectedByKF           = mnCorrectedByKF_;
    //mnCorrectedReference      = mnCorrectedReference_;
    //mPosGBA                   = mPosGBA_;
    //mnBAGlobalForKF           = mnBAGlobalForKF_;
    //mnBALocalForMerge         = mnBALocalForMerge_;
    //mPosMerge                 = mPosMerge_;
    //mNormalVectorMerge        = mNormalVectorMerge_;
    //mInvDepth                 = mInvDepth_;
    //mInitU                    = mInitU_;
    //mInitV                    = mInitV_;
    if(!mpHostKF)
        mpHostKF                  = static_cast<KeyFrame*>(NULL);
    mBackupHostKFId           = mBackupHostKFId_;
    mnOriginMapId             = mnOriginMapId_;
    //std::map<KeyFrame*,std::tuple<int,int> > mObservations = std::map<KeyFrame*,std::tuple<int,int> >(),
    mBackupObservationsId1    = mBackupObservationsId1_;
    mBackupObservationsId2    = mBackupObservationsId2_;
    mNormalVector             = mNormalVector_;
    //mDescriptor               = mDescriptor_.clone();
    if(!mpRefKF)
        mpRefKF                   = static_cast<KeyFrame*>(NULL);
    mBackupRefKFId            = mBackupRefKFId_;
    mnVisible                 = mnVisible_;
    mnFound                   = mnFound_;
    mbBad                     = mbBad_;
    if(mpReplaced)
        mpReplaced                = static_cast<MapPoint*>(NULL);
    //long long int mBackupReplacedId = -1,
    mBackupReplacedStrId      = mBackupReplacedStrId_;
    mfMinDistance             = mfMinDistance_;
    mfMaxDistance             = mfMaxDistance_;
    if(mpMap)
        mpMap                     = static_cast<Map*>(NULL);
    mnLastModule              = mnLastModule_;



}

void MapPoint::UpdateMapPoint(const int nObs_, /*KeyFrame* mpHostKF = nullptr, */const long long int mBackupHostKFId_, 
    const unsigned int mnOriginMapId_, const Eigen::Vector3f mWorldPos_, /*std::map<KeyFrame*,std::tuple<int,int> > mObservations = std::map<KeyFrame*,std::tuple<int,int> >(), */
    const std::map<long unsigned int, int>& mBackupObservationsId1_, const std::map<long unsigned int, int>& mBackupObservationsId2_, 
    const Eigen::Vector3f mNormalVector_, /*const cv::Mat& mDescriptor_,*/ /*KeyFrame* mpRefKF = nullptr,*/const long long int mBackupRefKFId_,
    const int mnVisible_,const int mnFound_,const bool mbBad_, /*MapPoint* mpReplaced = nullptr, */ /*long long int mBackupReplacedId = -1, */
    const std::string mBackupReplacedStrId_, const float mfMinDistance_, const float mfMaxDistance_ /*Map* mpMap = nullptr */, const unsigned int mnLastModule_)
{
    
    SetWorldPos(mWorldPos_);
    unique_lock<mutex> lock1(mMutexModule);
    unique_lock<mutex> lock3(mMutexMap);
    unique_lock<mutex> lock4(mMutexPos);
    //unique_lock<mutex> lock4(mGlobalMutex, std::defer_lock);
    
    lock(lock1, /*lock2,*/ lock3, lock4);
    
    nObs                      = nObs_;
    //mpHostKF                  = static_cast<KeyFrame*>(NULL);
    mBackupHostKFId           = mBackupHostKFId_;
    mnOriginMapId             = mnOriginMapId_;
    //std::map<KeyFrame*,std::tuple<int,int> > mObservations = std::map<KeyFrame*,std::tuple<int,int> >(),
    mBackupObservationsId1    = mBackupObservationsId1_;
    mBackupObservationsId2    = mBackupObservationsId2_;
    mNormalVector             = mNormalVector_;
    //mDescriptor               = mDescriptor_.clone();
    //mpRefKF                   = static_cast<KeyFrame*>(NULL);
    mBackupRefKFId            = mBackupRefKFId_;
    mnVisible                 = mnVisible_;
    mnFound                   = mnFound_;
    mbBad                     = mbBad_;
    //mpReplaced                = static_cast<MapPoint*>(NULL);
    //long long int mBackupReplacedId = -1,
    mBackupReplacedStrId      = mBackupReplacedStrId_;
    mfMinDistance             = mfMinDistance_;
    mfMaxDistance             = mfMaxDistance_;
    //mpMap                     = static_cast<Map*>(NULL);
    mnLastModule              = mnLastModule_;
}


void MapPoint::UpdateMapPoint(const MapPoint& mp) 
{
    
    //SetWorldPos(mp.mWorldPos);

    {
        //unique_lock<mutex> lock1(mMutexModule);
        unique_lock<mutex> lock1(mMutexMap, std::defer_lock);
        unique_lock<mutex> lock3(mMutexPos,std::defer_lock);
        //unique_lock<mutex> lock4(mGlobalMutex, std::defer_lock);
        //
        lock(lock1, /*lock2,*/ lock3/*, lock4*/);
        
        mWorldPos = mp.mWorldPos;

        mnFirstKFid               = mp.mnFirstKFid;
        mnFirstFrame              = mp.mnFirstFrame;
        nObs                      = mp.nObs;
        //mTrackProjX               = mTrackProjX_;
        //mTrackProjY               = mTrackProjY_;
        //mTrackDepth               = mTrackDepth_;
        //mTrackDepthR              = mTrackDepthR_;
        //mTrackProjXR              = mTrackProjXR_;
        //mTrackProjYR              = mTrackProjYR_;
        //mbTrackInView             = mbTrackInView_;
        //mbTrackInViewR            = mbTrackInViewR_;
        //mnTrackScaleLevel         = mnTrackScaleLevel_;
        //mnTrackScaleLevelR        = mnTrackScaleLevelR_;
        //mTrackViewCos             = mTrackViewCos_;
        //mTrackViewCosR            = mTrackViewCosR_;
        //mnTrackReferenceForFrame  = mnTrackReferenceForFrame_;
        //mnLastFrameSeen           = mnLastFrameSeen_;
        //mnBALocalForKF            = mp.mnBALocalForKF;
        //mnFuseCandidateForKF      = mp.mnFuseCandidateForKF;
        //mnLoopPointForKF          = mp.mnLoopPointForKF;
        //mnCorrectedByKF           = mp.mnCorrectedByKF;
        //mnCorrectedReference      = mp.mnCorrectedReference;
        //mPosGBA                   = mp.mPosGBA;
        //mnBAGlobalForKF           = mp.mnBAGlobalForKF;
        //mnBALocalForMerge         = mp.mnBALocalForMerge;
        //mPosMerge                 = mp.mPosMerge;
        //mNormalVectorMerge        = mp.mNormalVectorMerge;
        mInvDepth                 = mp.mInvDepth;
        mInitU                    = mp.mInitU;
        mInitV                    = mp.mInitV;
        mpHostKF                  = mp.mpHostKF;
        //mBackupHostKFId           = mp->mBackupHostKFId;
        mnOriginMapId             = mp.mnOriginMapId;
        //mObservations             = mp.mObservations;
        if(!mp.mObservations.empty()){
            unique_lock<mutex> lock2(mMutexFeatures);
            mObservations.clear();
            mBackupObservationsId1.clear();
            mBackupObservationsId2.clear();

            map<KeyFrame*, std::tuple<int,int>> mObs(mp.mObservations);
            for(map<KeyFrame*, std::tuple<int,int>>::const_iterator it = mObs.begin(), end = mObs.end(); it != end; ++it)
            {
              //std::cout << it->first->mnId << "," << get<0>(it->second) << std::endl;
              if(it->first)
              {
                  mObservations[it->first] = it->second;
                  mBackupObservationsId1[it->first->mnId] = get<0>(it->second); 
                  mBackupObservationsId2[it->first->mnId] = get<1>(it->second); 
              }
            }
            mDescriptor               = mp.mDescriptor.clone();
        }


        mNormalVector             = mp.mNormalVector;
        mpRefKF                   = mp.mpRefKF;
        //mBackupRefKFId            = mp->mBackupRefKFId;
        mnVisible                 = mp.mnVisible;
        mnFound                   = mp.mnFound;
        mbBad                     = mp.mbBad;
        mpReplaced                = mp.mpReplaced;
        //long long int mBackupReplacedId = -1,
        //mBackupReplacedStrId      = mp->mBackupReplacedStrId;
        mfMinDistance             = mp.mfMinDistance;
        mfMaxDistance             = mp.mfMaxDistance;
        mpMap                     = mp.mpMap;
        mnLastModule              = mp.mnLastModule;

    }

}



void MapPoint::SetWorldPos(const Eigen::Vector3f &Pos) {
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
    mWorldPos = Pos;
}

Eigen::Vector3f MapPoint::GetWorldPos() {
    unique_lock<mutex> lock(mMutexPos);
    return mWorldPos;
}

Eigen::Vector3f MapPoint::GetNormal() {
    unique_lock<mutex> lock(mMutexPos);
    return mNormalVector;
}


KeyFrame* MapPoint::GetReferenceKeyFrame()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mpRefKF;
}

void MapPoint::SetReferenceKeyFrame(KeyFrame* mpRef)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mpRefKF = mpRef;
    mBackupRefKFId = mpRef->mnId;
}

long int MapPoint::GetRefBackup()
{
  return mBackupRefKFId;
}
void MapPoint::AddObservation(KeyFrame* pKF, int idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    tuple<int,int> indexes;

    if(mObservations.count(pKF)){
        indexes = mObservations[pKF];
    }
    else{
        indexes = tuple<int,int>(-1,-1);
    }

    if(pKF -> NLeft != -1 && idx >= pKF -> NLeft){
        get<1>(indexes) = idx;
    }
    else{
        get<0>(indexes) = idx;
    }

    mObservations[pKF]=indexes;

    if(!pKF->mpCamera2 && pKF->mvuRight[idx]>=0)
        nObs+=2;
    else
        nObs++;
}

void MapPoint::EraseObservation(KeyFrame* pKF)
{
    bool bBad=false;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mObservations.count(pKF))
        {
            tuple<int,int> indexes = mObservations[pKF];
            int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

            if(leftIndex != -1){
                if(!pKF->mpCamera2 && pKF->mvuRight[leftIndex]>=0)
                    nObs-=2;
                else
                    nObs--;
            }
            if(rightIndex != -1){
                nObs--;
            }

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


std::map<KeyFrame*, std::tuple<int,int>>  MapPoint::GetObservations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mObservations;
}

int MapPoint::Observations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return nObs;
}

void MapPoint::SetBadFlag()
{
    
    map<KeyFrame*, tuple<int,int>> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        mbBad=true;
        obs = mObservations;
        mObservations.clear();
    }
    for(map<KeyFrame*, tuple<int,int>>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        int leftIndex = get<0>(mit -> second), rightIndex = get<1>(mit -> second);
        if(leftIndex != -1){
            pKF->EraseMapPointMatch(leftIndex);
        }
        //TODO: Uncomment this
        //if(rightIndex != -1){
        //    pKF->EraseMapPointMatch(rightIndex);
        //}
    }
    
    if(mpMap)
        mpMap->EraseMapPoint(this);
        
}

MapPoint* MapPoint::GetReplaced()
{
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mpReplaced;
}

void MapPoint::Replace(MapPoint* pMP)
{
    if(!pMP || pMP->isBad())
        return;
    if(pMP->mstrHexId==this->mstrHexId)
        return;

    
    int nvisible, nfound;
    map<KeyFrame*,tuple<int,int>> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        obs=mObservations;
        mObservations.clear();
        mbBad=true;
        nvisible = mnVisible;
        nfound = mnFound;
        mpReplaced = pMP;
    }

    for(map<KeyFrame*,tuple<int,int>>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        // Replace measurement in keyframe
        KeyFrame* pKF = mit->first;

        tuple<int,int> indexes = mit -> second;
        int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

        if(!pMP->IsInKeyFrame(pKF))
        {
            if(leftIndex != -1){
                pKF->ReplaceMapPointMatch(leftIndex, pMP);
                pMP->AddObservation(pKF,leftIndex);
            }
            if(rightIndex != -1){
                pKF->ReplaceMapPointMatch(rightIndex, pMP);
                pMP->AddObservation(pKF,rightIndex);
            }
        }
        else
        {
            if(leftIndex != -1){
                pKF->EraseMapPointMatch(leftIndex);
            }
            if(rightIndex != -1){
                pKF->EraseMapPointMatch(rightIndex);
            }
        }
    }
    pMP->IncreaseFound(nfound);
    pMP->IncreaseVisible(nvisible);
    pMP->ComputeDistinctiveDescriptors();
    
    if(!mpMap)
        return;
    
    mpMap->EraseMapPoint(this);
}

bool MapPoint::isBad()
{
    unique_lock<mutex> lock1(mMutexFeatures,std::defer_lock);
    unique_lock<mutex> lock2(mMutexPos,std::defer_lock);
    lock(lock1, lock2);

    return mbBad;
}

void MapPoint::IncreaseVisible(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnVisible+=n;
}

void MapPoint::IncreaseFound(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnFound+=n;
}

float MapPoint::GetFoundRatio()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return static_cast<float>(mnFound)/mnVisible;
}
int MapPoint::GetFound() 
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mnFound;
}
int MapPoint::GetVisible() 
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mnVisible;
}
void MapPoint::ComputeDistinctiveDescriptors()
{
    // Retrieve all observed descriptors
    vector<cv::Mat> vDescriptors;

    map<KeyFrame*,tuple<int,int>> observations;

    {
        unique_lock<mutex> lock1(mMutexFeatures);
        if(mbBad)
            return;
        observations=mObservations;
    }

    if(observations.empty())
        return;
    

    vDescriptors.reserve(observations.size());

    for(map<KeyFrame*,tuple<int,int>>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;

        if(pKF && !pKF->isBad()){
            tuple<int,int> indexes = mit -> second;
            int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
            //std::cout << leftIndex << ", " << rightIndex << std::endl;
            if(leftIndex >= 0){
                vDescriptors.push_back(pKF->mDescriptors.row(leftIndex));
            }
            // fix: removed in monocular, add back later. Randomly value is overflown = 1060103539
            //if(rightIndex >= 0 && rightIndex < INT_MAX){
            //    vDescriptors.push_back(pKF->mDescriptors.row(rightIndex));
            //}
        }
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
        unique_lock<mutex> lock(mMutexFeatures);
        mDescriptor = vDescriptors[BestIdx].clone();
    }
}

cv::Mat MapPoint::GetDescriptor()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mDescriptor.clone();
}

tuple<int,int> MapPoint::GetIndexInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return mObservations[pKF];
    else
        return tuple<int,int>(-1,-1);
}

bool MapPoint::IsInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return (mObservations.count(pKF));
}

void MapPoint::UpdateNormalAndDepth()
{
    map<KeyFrame*,tuple<int,int>> observations;
    KeyFrame* pRefKF;
    Eigen::Vector3f Pos;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        if(mbBad)
            return;
        observations = mObservations;
        if(!mpRefKF)
          return;
        pRefKF = mpRefKF;
        Pos = mWorldPos;
    }

    if(observations.empty())
        return;

    
    Eigen::Vector3f normal;
    normal.setZero();
    int n=0;
    for(map<KeyFrame*,tuple<int,int>>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        
        // Fix this later
        if(!pKF || pKF->isBad())
          continue;

        tuple<int,int> indexes = mit -> second;
        int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

        if(leftIndex != -1){
            Eigen::Vector3f Owi = pKF->GetCameraCenter();
            //std::cout << "after getting the translation" << std::endl;
            Eigen::Vector3f normali = Pos - Owi;
            normal = normal + normali / normali.norm();
            n++;
        }
        if(rightIndex != -1){
            Eigen::Vector3f Owi = pKF->GetRightCameraCenter();
            Eigen::Vector3f normali = Pos - Owi;
            normal = normal + normali / normali.norm();
            n++;
        }
    }
    
    //std::cout << "Get Camera Center of Reference KF" << std::endl;
    Eigen::Vector3f PC = Pos - pRefKF->GetCameraCenter();
    const float dist = PC.norm();

    tuple<int ,int> indexes = observations[pRefKF];
    int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
    int level;
    if(pRefKF -> NLeft == -1){
        level = pRefKF->mvKeysUn[leftIndex].octave;
    }
    else if(leftIndex != -1){
        level = pRefKF -> mvKeys[leftIndex].octave;
    }
    else{
        level = pRefKF -> mvKeysRight[rightIndex - pRefKF -> NLeft].octave;
    }

    //const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
    const float levelScaleFactor =  pRefKF->mvScaleFactors[level];
    const int nLevels = pRefKF->mnScaleLevels;

    {
        unique_lock<mutex> lock3(mMutexPos);
        mfMaxDistance = dist*levelScaleFactor;
        mfMinDistance = mfMaxDistance/pRefKF->mvScaleFactors[nLevels-1];
        mNormalVector = normal/n;
    }
}

void MapPoint::SetNormalVector(const Eigen::Vector3f& normal)
{
    unique_lock<mutex> lock3(mMutexPos);
    mNormalVector = normal;
}

float MapPoint::GetMinDistance()
{
    unique_lock<mutex> lock(mMutexPos);
    return mfMinDistance;
}

float MapPoint::GetMaxDistance()
{
    unique_lock<mutex> lock(mMutexPos);
    return mfMaxDistance;
}

float MapPoint::GetMinDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 0.8f * mfMinDistance;
}

float MapPoint::GetMaxDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 1.2f * mfMaxDistance;
}

int MapPoint::PredictScale(const float &currentDist, KeyFrame* pKF)
{
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
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
        unique_lock<mutex> lock(mMutexPos);
        ratio = mfMaxDistance/currentDist;
    }

    int nScale = ceil(log(ratio)/pF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pF->mnScaleLevels)
        nScale = pF->mnScaleLevels-1;

    return nScale;
}

void MapPoint::PrintObservations()
{
    cout << "MP_OBS: MP " << mstrHexId << endl;
    for(map<KeyFrame*,tuple<int,int>>::iterator mit=mObservations.begin(), mend=mObservations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKFi = mit->first;
        tuple<int,int> indexes = mit->second;
        int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
        cout << "--OBS in KF " << pKFi->mnId << " in map " << pKFi->GetMap()->GetId() << endl;
    }
}

Map* MapPoint::GetMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mpMap;
}

void MapPoint::UpdateMap(Map* pMap)
{
    unique_lock<mutex> lock(mMutexMap);
    mpMap = pMap;
}

unsigned int MapPoint::GetLastModule() 
{
    unique_lock<mutex> lock(mMutexModule);
    return mnLastModule;
}

void MapPoint::SetLastModule(unsigned int mnId) 
{
    unique_lock<mutex> lock(mMutexModule);
    mnLastModule = mnId;
}

std::string MapPoint::createHashId(const std::string& strSystemId, unsigned long int mnMPId)
{
  // Combine string and number
  std::string combinedInput = strSystemId + std::to_string(mnMPId);

  // Basic hash function using std::hash
  size_t hashValue = std::hash<std::string>{}(combinedInput);

  // Convert hash value to a 6-character hexadecimal string
  std::stringstream hexStream;
  hexStream << std::hex << std::setw(6) << std::setfill('0') << (hashValue % 0xFFFFFF);
  std::string hexHash = hexStream.str();

  return hexHash;
}


void MapPoint::PreSave()//(set<KeyFrame*>& spKF,set<MapPoint*>& spMP)
{
    unique_lock<mutex> lock1(mMutexPos,std::defer_lock);
    unique_lock<mutex> lock2(mMutexFeatures,std::defer_lock);
    unique_lock<mutex> lock3(mMutexMap,std::defer_lock);
    unique_lock<mutex> lock4(mGlobalMutex, std::defer_lock);
    
    lock(lock1, lock2, lock3, lock4);
    
    //unique_lock<mutex> lock2(mMutexMap);
    //std::cout << "start of a presave" << std::endl;
    mBackupReplacedStrId = "";
    if(mpReplaced)
    {
        mBackupReplacedStrId = mpReplaced->mstrHexId;
    } else {
        mpReplaced = static_cast<MapPoint*>(NULL);
        mBackupReplacedStrId="";
    }

    mBackupObservationsId1.clear();
    mBackupObservationsId2.clear();
    std::set<KeyFrame*> mvpEraseObservationsKF;
    //std::cout << "before observations, nObs=" << nObs << std::endl;
    
    //if(mObservations.size() > 0)
    //std::cout << "mObservations" << std::endl;

    // Save the id and position in each KF who view it
    for(std::map<KeyFrame*,std::tuple<int,int> >::const_iterator it = mObservations.begin(), end = mObservations.end(); it != end; ++it)
    {
        //std::cout << "iteration ";
        KeyFrame* pKFi = it->first;
        if(!pKFi)
            continue;
        mBackupObservationsId1[it->first->mnId] = get<0>(it->second);
        mBackupObservationsId2[it->first->mnId] = get<1>(it->second);
        // Memory problem, fix!
        //else {
        //    EraseObservation(pKFi);
        //}

        //std::cout << " end." << std::endl;
    }

    //for(const auto& eraseKF : mvpEraseObservationsKF)
    //{
    //    EraseObservation(eraseKF);
    //}
    
    //std::cout << "after observations" << std::endl;
    // Save the id of the reference KF
    mBackupRefKFId = -1;
    if(mpRefKF)
    {
        mBackupRefKFId = mpRefKF->mnId;
    } 
    //else {
    //    mpHostKF = static_cast<KeyFrame*>(NULL);
    //}


    // Save the id of the reference KF
    mBackupHostKFId = -1;
    if(mpHostKF)
    {
        mBackupHostKFId = mpHostKF->mnId;
    } 
    //else {
    //    mpHostKF = static_cast<KeyFrame*>(NULL);
    //}

}


void MapPoint::PostLoad(map<long unsigned int, KeyFrame*>& mpKFid, std::unordered_map<std::string, MapPoint*>& mpMPid, bool* bUnprocessed, std::set<unsigned long int> mspUnprocKFids)
{
    
    unique_lock<mutex> lock1(mMutexPos,std::defer_lock);
    unique_lock<mutex> lock2(mMutexFeatures,std::defer_lock);
    unique_lock<mutex> lock3(mMutexMap,std::defer_lock);
    unique_lock<mutex> lock4(mGlobalMutex, std::defer_lock);
    
    lock(lock1, lock2, lock3, lock4);
    //if(mspUnprocKFids.find(mBackupRefKFId) != mspUnprocKFids.end() && !mspUnprocKFids.empty())
    //{
    //  *bUnprocessed = true; 
    //  std::cout << "Required KF=" << mBackupRefKFId << " is not processed yet (mBackupRefKFId)" << std::endl;
    //  return;
    //}

    // Check if map point ptr can be found, if not, return
    if(mBackupRefKFId > -1 && mBackupRefKFId < 10000 && mpKFid.find(mBackupRefKFId) == mpKFid.end()) {
      //if(mBackupRefKFId < mpKFid.rbegin()->first)
      //{
      //  mBackupRefKFId = -1;
      //  mpRefKF=static_cast<KeyFrame*>(NULL);
      //} 
      mBackupRefKFId = -1;
      mpRefKF=static_cast<KeyFrame*>(NULL);
      //else {

      //  *bUnprocessed = true; 
      //  std::cout << "ref kf id " << mBackupRefKFId << std::endl;
      //  return;
      //}
    } else {
      mpRefKF = mpKFid[mBackupRefKFId];
    }
    
    if(!mpRefKF)
    {
        mpRefKF=static_cast<KeyFrame*>(NULL);
        //cout << "ERROR: MP without KF reference " << mBackupRefKFId << "; Num obs: " << nObs << endl;
    }
    

    //if(mspUnprocKFids.find(mBackupHostKFId) != mspUnprocKFids.end() && !mspUnprocKFids.empty())
    //{
    //  *bUnprocessed = true; 
    //  std::cout << "Required KF=" << mBackupHostKFId << " is not processed yet (mBackupHostKFId)" << std::endl;
    //  return;
    //}
    // Check if map point ptr can be found, if not, return
    if(mBackupHostKFId > -1 && mBackupHostKFId < 10000 && mpKFid.find(mBackupHostKFId) == mpKFid.end()) {
      mBackupHostKFId = -1;
      mpHostKF=static_cast<KeyFrame*>(NULL);
      //if(mBackupHostKFId < mpKFid.rbegin()->first)
      //{
      //  mBackupHostKFId = -1;
      //  mpHostKF=static_cast<KeyFrame*>(NULL);
      //} else {
      //
      //  *bUnprocessed = true; 
      //  std::cout << "host kf id " << mBackupHostKFId << std::endl;
      //  return;
      //}
    } else {
      mpHostKF = mpKFid[mBackupHostKFId];
    }

    if(!mpHostKF)
    {
        mpHostKF = static_cast<KeyFrame*>(NULL);   
        //cout << "ERROR: MP without KF HOST " << mBackupHostKFId << "; Num obs: " << nObs << endl;
    }
    
    // TODO: Here we need to check the local mapping id and tracking id
    mpReplaced = static_cast<MapPoint*>(NULL);
    if(mBackupReplacedStrId!="" && mBackupReplacedStrId.length() == 6)
    {
        unordered_map<std::string, MapPoint*>::iterator it = mpMPid.find(mBackupReplacedStrId);
        if(it != mpMPid.end())
        {
          if(it->second)
          {
              mpReplaced = it->second;
          }
        }
        //else {
        //  *bUnprocessed = true; 
        //  std::cout << "replace mp id " << mBackupReplacedStrId << std::endl;
        //  return;
        //}
    }
    //std::cout << "after mp replaced" << std::endl;
    mObservations.clear();

    //std::cout << "before obs " << mBackupObservationsId1.size() << ", " << mBackupObservationsId2.size() << std::endl;
    for(map<long unsigned int, int>::const_iterator it = mBackupObservationsId1.begin(), end = mBackupObservationsId1.end(); it != end; ++it)
    {
        //std::cout << "observations stuff " << it->first << std::endl;
        //if(mspUnprocKFids.find(it->first) != mspUnprocKFids.end() && !mspUnprocKFids.empty())
        //{
        //  *bUnprocessed = true; 
        //  std::cout << "Required KF=" << it->first << " is not processed yet (observations)" << std::endl;
        //  return;
        //}
        
        // Check if map point ptr can be found, if not, return
        ORB_SLAM3::KeyFrame* pKFi = mpKFid[it->first];
        if(!pKFi || (pKFi && pKFi->GetMap() && pKFi->GetMap()->GetId() != this->mpMap->GetId())) {
          continue;
          //if(it->first < mpKFid.rbegin()->first)
          //{
          //  continue;
          //} else {
          //  *bUnprocessed = true;
          //  std::cout << "obs kf id " << it->first << std::endl;
          //  return;
          //}
        } else {

          map<long unsigned int, int>::const_iterator it2 = mBackupObservationsId2.find(it->first);
          std::tuple<int, int> indexes = tuple<int,int>(it->second,it2->second);
          mObservations[pKFi] = indexes;
        }
    }

    //mBackupObservationsId1.clear();
    //mBackupObservationsId2.clear();
    //std::cout << "end of postload" << std::endl;
}

void MapPoint::PostLoad(map<long unsigned int, KeyFrame*>& mpKFid, map<std::string, MapPoint*>& mpMPid, bool* bUnprocessed, std::set<unsigned long int> mspUnprocKFids)
{
    
    //unique_lock<mutex> lock1(mMutexPos,std::defer_lock);
    //unique_lock<mutex> lock2(mMutexFeatures,std::defer_lock);
    //unique_lock<mutex> lock3(mMutexMap,std::defer_lock);
    //unique_lock<mutex> lock4(mGlobalMutex, std::defer_lock);
    //
    //lock(lock1, lock2, lock3, lock4);
    //if(mspUnprocKFids.find(mBackupRefKFId) != mspUnprocKFids.end() && !mspUnprocKFids.empty())
    //{
    //  *bUnprocessed = true; 
    //  std::cout << "Required KF=" << mBackupRefKFId << " is not processed yet (mBackupRefKFId)" << std::endl;
    //  return;
    //}

    // Check if map point ptr can be found, if not, return
    if(mpKFid[mBackupRefKFId] && (mpKFid[mBackupRefKFId]->GetMap() && mpKFid[mBackupRefKFId]->GetMap()->GetId() == this->mpMap->GetId())) {
      //if(mBackupRefKFId < mpKFid.rbegin()->first)
      //{
      //  mBackupRefKFId = -1;
      //  mpRefKF=static_cast<KeyFrame*>(NULL);
      //} 
      //else {
      
      mpRefKF = mpKFid[mBackupRefKFId];
      //  *bUnprocessed = true; 
      //  std::cout << "ref kf id " << mBackupRefKFId << std::endl;
      //  return;
      //}
    } else {
      mBackupRefKFId = -1;
      mpRefKF=static_cast<KeyFrame*>(NULL);
    }
    
    if(!mpRefKF)
    {
        mpRefKF=static_cast<KeyFrame*>(NULL);
        //cout << "ERROR: MP without KF reference " << mBackupRefKFId << "; Num obs: " << nObs << endl;
    }
    

    //if(mspUnprocKFids.find(mBackupHostKFId) != mspUnprocKFids.end() && !mspUnprocKFids.empty())
    //{
    //  *bUnprocessed = true; 
    //  std::cout << "Required KF=" << mBackupHostKFId << " is not processed yet (mBackupHostKFId)" << std::endl;
    //  return;
    //}
    // Check if map point ptr can be found, if not, return
    if(mpKFid[mBackupHostKFId] && (mpKFid[mBackupRefKFId]->GetMap() && mpKFid[mBackupHostKFId]->GetMap()->GetId() == this->mpMap->GetId())) {
      mpHostKF = mpKFid[mBackupHostKFId];
      //if(mBackupHostKFId < mpKFid.rbegin()->first)
      //{
      //  mBackupHostKFId = -1;
      //  mpHostKF=static_cast<KeyFrame*>(NULL);
      //} else {
      //
      //  *bUnprocessed = true; 
      //  std::cout << "host kf id " << mBackupHostKFId << std::endl;
      //  return;
      //}
    } else {
      mBackupHostKFId = -1;
      mpHostKF=static_cast<KeyFrame*>(NULL);
    }

    if(!mpHostKF)
    {
        mpHostKF = static_cast<KeyFrame*>(NULL);   
        //cout << "ERROR: MP without KF HOST " << mBackupHostKFId << "; Num obs: " << nObs << endl;
    }
    
    // TODO: Here we need to check the local mapping id and tracking id
    mpReplaced = static_cast<MapPoint*>(NULL);
    if(mBackupReplacedStrId!="" && mBackupReplacedStrId.length() == 6)
    {
        ORB_SLAM3::MapPoint* tempMP = mpMPid[mBackupReplacedStrId];
        if (tempMP && (tempMP->GetMap() && tempMP->GetMap()->GetId() == this->mpMap->GetId()))
        {
          mpReplaced = tempMP;
        }
        //else {
        //  *bUnprocessed = true; 
        //  std::cout << "replace mp id " << mBackupReplacedStrId << std::endl;
        //  return;
        //}
    }
    //std::cout << "after mp replaced" << std::endl;
    mObservations.clear();

    //std::cout << "before obs " << mBackupObservationsId1.size() << ", " << mBackupObservationsId2.size() << std::endl;
    for(map<long unsigned int, int>::const_iterator it = mBackupObservationsId1.begin(), end = mBackupObservationsId1.end(); it != end; ++it)
    {
        //std::cout << "observations stuff " << it->first << std::endl;
        //if(mspUnprocKFids.find(it->first) != mspUnprocKFids.end() && !mspUnprocKFids.empty())
        //{
        //  *bUnprocessed = true; 
        //  std::cout << "Required KF=" << it->first << " is not processed yet (observations)" << std::endl;
        //  return;
        //}
        
        // Check if map point ptr can be found, if not, return
        ORB_SLAM3::KeyFrame* tempKF = mpKFid[it->first];
        if(!tempKF || (tempKF->GetMap() && tempKF->GetMap()->GetId() != this->mpMap->GetId())) 
          continue;
        //if(it->first < mpKFid.rbegin()->first)
        //{
        //  continue;
        //} else {
        //  *bUnprocessed = true;
        //  std::cout << "obs kf id " << it->first << std::endl;
        //  return;
        //}

        map<long unsigned int, int>::const_iterator it2 = mBackupObservationsId2.find(it->first);
        std::tuple<int, int> indexes = tuple<int,int>(it->second,it2->second);
        mObservations[tempKF] = indexes;
    }

    //mBackupObservationsId1.clear();
    //mBackupObservationsId2.clear();
    //std::cout << "end of postload" << std::endl;
}

std::map<long unsigned int, int> MapPoint::GetObservationsBackup1() {
  unique_lock<mutex> lock(mMutexFeatures);
  //mBackupObservationsId1.clear();
  //for(std::map<KeyFrame*,std::tuple<int,int> >::const_iterator it = mObservations.begin(), end = mObservations.end(); it != end; ++it)
  //{
  //    KeyFrame* pKFi = it->first;
  //    if(!pKFi)
  //        continue;
  //    mBackupObservationsId1[it->first->mnId] = get<0>(it->second);
  //}
  return mBackupObservationsId1; 
}

std::map<long unsigned int, int> MapPoint::GetObservationsBackup2() {
  unique_lock<mutex> lock(mMutexFeatures);
  //mBackupObservationsId2.clear();
  //for(std::map<KeyFrame*,std::tuple<int,int> >::const_iterator it = mObservations.begin(), end = mObservations.end(); it != end; ++it)
  //{
  //    KeyFrame* pKFi = it->first;
  //    if(!pKFi)
  //        continue;
  //    mBackupObservationsId2[it->first->mnId] = get<1>(it->second);
  //}
  return mBackupObservationsId2; 
}
} //namespace ORB_SLAM
