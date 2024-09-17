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


#include "Map.h"

#include<mutex>

namespace ORB_SLAM3
{

long unsigned int Map::nNextId=0;

Map::Map():mnMaxKFid(0),mnBigChangeIdx(0), mbImuInitialized(false), mnMapChange(0), mpFirstRegionKF(static_cast<KeyFrame*>(NULL)),
mbFail(false), mIsInUse(false), mHasTumbnail(false), mbBad(false), mnMapChangeNotified(0), mbIsInertial(false), mbIMU_BA1(false), mbIMU_BA2(false)
{
    std::cout << "Thread1=Map::Map : Create a new map without KFid" << std::endl;
    mnId=nNextId++;
    mThumbnail = static_cast<GLubyte*>(NULL);
}

Map::Map(int initKFid):mnInitKFid(initKFid), mnMaxKFid(initKFid),/*mnLastLoopKFid(initKFid),*/ mnBigChangeIdx(0), mIsInUse(false),
                       mHasTumbnail(false), mbBad(false), mbImuInitialized(false), mpFirstRegionKF(static_cast<KeyFrame*>(NULL)),
                       mnMapChange(0), mbFail(false), mnMapChangeNotified(0), mbIsInertial(false), mbIMU_BA1(false), mbIMU_BA2(false)
{
    std::cout << "Thread1=Map::Map : Create a new map with KFid" << std::endl;
    mnId=nNextId++;
    mThumbnail = static_cast<GLubyte*>(NULL);
}


Map::Map(const bool mbFail, const std::set<long unsigned int>& msOptKFs, const std::set<long unsigned int>& msFixedKFs, const long unsigned int mnId, 
    const std::vector<std::string>& mvpBackupMapPointsId, const std::vector<unsigned long int>& mvpBackupKeyFramesId, 
    const std::vector<unsigned long int>& mvBackupKeyFrameOriginsId, const unsigned long int mnBackupKFinitialID, const unsigned long int mnBackupKFlowerID, 
    const std::vector<std::string>& mvpBackupReferenceMapPointsId, const bool mbImuInitialized, const int mnMapChange, 
    const int mnMapChangeNotified, const long unsigned int mnInitKFid, const long unsigned int mnMaxKFid, const int mnBigChangeIdx, 
    const bool mIsInUse, const bool mHasTumbnail, const bool mbBad, const bool mbIsInertial, const bool mbIMU_BA1, const bool mbIMU_BA2, 
    const std::set<unsigned long int>& msErasedKFIds, const std::set<std::string>& mspErasedMPIds):
    mbFail(mbFail), msOptKFs(msOptKFs), msFixedKFs(msFixedKFs), mnId(mnId), mvpBackupMapPointsId(mvpBackupMapPointsId), 
    mvpBackupKeyFramesId(mvpBackupKeyFramesId), mvBackupKeyFrameOriginsId(mvBackupKeyFrameOriginsId), 
    mnBackupKFinitialID(mnBackupKFinitialID), mnBackupKFlowerID(mnBackupKFlowerID), 
    mvpBackupReferenceMapPointsId(mvpBackupReferenceMapPointsId), mbImuInitialized(mbImuInitialized), mnMapChange(mnMapChange), 
    mnMapChangeNotified(mnMapChangeNotified), mnInitKFid(mnInitKFid), mnMaxKFid(mnMaxKFid), 
    /*mnBigChangeIdx(mnBigChangeIdx),*/ mIsInUse(mIsInUse), mHasTumbnail(mHasTumbnail), mbBad(mbBad), 
    mbIsInertial(mbIsInertial), mbIMU_BA1(mbIMU_BA1), mbIMU_BA2(mbIMU_BA2), mspErasedMapPointIds(mspErasedMPIds), mspErasedKeyFrameIds(msErasedKFIds)
{
    mThumbnail = static_cast<GLubyte*>(NULL);

    mpKFinitial = static_cast<KeyFrame*>(NULL);           
    mpKFlowerID = static_cast<KeyFrame*>(NULL);             
    mspMapPoints = std::set<MapPoint*>();
    mspKeyFrames = std::set<KeyFrame*>();

    mvpBackupMapPoints = std::vector<MapPoint*>();
    mvpBackupKeyFrames = std::vector<KeyFrame*>();

    mvpReferenceMapPoints = std::vector<MapPoint*>();
    std::cout << "Thread1=Map::Map : Create a new map from ROS" << std::endl;
}

Map::~Map()
{
    //TODO: erase all points from memory
    mspMapPoints.clear();

    //TODO: erase all keyframes from memory
    mspKeyFrames.clear();

    if(mThumbnail)
        delete mThumbnail;
    mThumbnail = static_cast<GLubyte*>(NULL);

    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}


void Map::UpdateMap(const Map &tempMap, const int nFromModule)
{
    {
        unique_lock<mutex> lock(mMutexMap);
        mbFail = tempMap.mbFail; 
        msOptKFs = tempMap.msOptKFs; 
        msFixedKFs = tempMap.msFixedKFs;

        //mvpReferenceMapPoints.clear();
        //for(ORB_SLAM3::MapPoint* tempMP : tempMap.mvpReferenceMapPoints)
        //{
        //  mvpReferenceMapPoints.push_back(tempMP);
        //}

        //mvpReferenceMapPoints = tempMap.mvpReferenceMapPoints;

        //mvpBackupReferenceMapPointsId = tempMap.mvpBackupReferenceMapPointsId;
        //mvpBackupReferenceMapPointsId.clear();
        //for(std::string tempMPid : tempMap.mvpBackupReferenceMapPointsId)
        //{
        //  mvpBackupReferenceMapPointsId.push_back(tempMPid);
        //}

        mspErasedMapPointIds = tempMap.mspErasedMapPointIds;
        for(const auto id : tempMap.mspErasedKeyFrameIds)
        {
            mspErasedKeyFrameIds.insert(id);
        }

        //for(const auto id : tempMap.mspErasedMapPointIds)
        //{
        //    mspErasedMapPointIds.insert(id);
        //}

        mbImuInitialized = tempMap.mbImuInitialized; 
        mnMapChange = tempMap.mnMapChange; 
        mnMapChangeNotified = tempMap.mnMapChangeNotified; 
        mnBigChangeIdx = tempMap.mnBigChangeIdx;
        mIsInUse = tempMap.mIsInUse; 
        mHasTumbnail = tempMap.mHasTumbnail; 
        mbBad = tempMap.mbBad; 
        mbIsInertial = tempMap.mbIsInertial; 
        mbIMU_BA1 = tempMap.mbIMU_BA1;
        mbIMU_BA2 = tempMap.mbIMU_BA2;
            

        mnInitKFid = tempMap.mnInitKFid; 

        mvBackupKeyFrameOriginsId = tempMap.mvBackupKeyFrameOriginsId; 
        mpKFinitial = tempMap.mpKFinitial;
        mpKFlowerID = tempMap.mpKFlowerID;
        mnBackupKFinitialID = tempMap.mnBackupKFinitialID; 
        mnBackupKFlowerID = tempMap.mnBackupKFlowerID; 

    }
    if(nFromModule != 3)
    {
        {
            unique_lock<mutex> lock(mMutexMap);
            mvpReferenceMapPoints=tempMap.mvpReferenceMapPoints;
            mnMaxKFid = tempMap.mnMaxKFid; 
            //mvpBackupMapPointsId = tempMap.mvpBackupMapPointsId;
            mvpBackupKeyFramesId = tempMap.mvpBackupKeyFramesId;

            //mspMapPoints = tempMap.mspMapPoints;
            mspKeyFrames = tempMap.mspKeyFrames;


        }
        for(const auto& pMP : mspMapPoints)
        {
          pMP->UpdateMap(this);
        }

        for(const auto& pKF : mspKeyFrames)
        {
          pKF->UpdateMap(this);
        }

    }
}


void Map::UpdateMap(const bool mbFail_, const std::set<long unsigned int>& msOptKFs_, const std::set<long unsigned int>& msFixedKFs_, 
    const long unsigned int mnId_, const std::vector<std::string>& mvpBackupMapPointsId_, const std::vector<unsigned long int>& mvpBackupKeyFramesId_, 
    const std::set<unsigned long int>& msUpdatedKFIds, const std::set<std::string>& msUpdatedMPIds, const std::vector<unsigned long int>& mvBackupKeyFrameOriginsId_, 
    const unsigned long int mnBackupKFinitialID_, const unsigned long int mnBackupKFlowerID_, const std::vector<std::string>& mvpBackupReferenceMapPointsId_, 
    const bool mbImuInitialized_, const int mnMapChange_, const int mnMapChangeNotified_, const long unsigned int mnInitKFid_, 
    const long unsigned int mnMaxKFid_, const int mnBigChangeIdx_, const bool mIsInUse_, /*const bool mHasTumbnail,*/ 
    const bool mbBad_ /*, const bool mbIsInertial, const bool mbIMU_BA1, const bool mbIMU_BA2*/, const std::set<unsigned long int>& msErasedKFIds, 
    const std::set<std::string>& mspErasedMPIds)
{
    unique_lock<mutex> lock(mMutexMap);
    mbFail                        =  mbFail_;                        
    msOptKFs                      =  msOptKFs_;                      
    msFixedKFs                    =  msFixedKFs_;                    
    //mnId                          =  mnId_;

    mspUpdatedMapPointIds = msUpdatedMPIds;
    mspUpdatedKeyFrameIds = msUpdatedKFIds;
    
    //for(const auto& mpId : mvpBackupMapPointsId_)
    //{
    //  auto it = std::find(mvpBackupMapPointsId.begin(), mvpBackupMapPointsId.end(), mpId);
    //  if(it == mvpBackupMapPointsId.end())
    //    mvpBackupMapPointsId.push_back(mpId);
    //  
    //}
    mvpBackupMapPointsId          =  mvpBackupMapPointsId_;          
    
    //for(const auto& kfId : mvpBackupKeyFramesId_)
    //{
    //  auto it = std::find(mvpBackupKeyFramesId.begin(), mvpBackupKeyFramesId.end(), kfId);
    //  if(it == mvpBackupKeyFramesId.end())
    //    mvpBackupKeyFramesId.push_back(kfId); 
    //}

    mvpBackupKeyFramesId          =  mvpBackupKeyFramesId_;          
    //mnBackupKFinitialID           =  mnBackupKFinitialID_;           
    //mnBackupKFlowerID             =  mnBackupKFlowerID_;             
    //for(const auto& mpId : mvpBackupReferenceMapPointsId_)
    //{
    //  auto it = std::find(mvpBackupReferenceMapPointsId.begin(), mvpBackupReferenceMapPointsId.end(), mpId);
    //  if(it == mvpBackupReferenceMapPointsId.end())
    //    mvpBackupReferenceMapPointsId.push_back(mpId); 
    //}
    mvpBackupReferenceMapPointsId = mvpBackupReferenceMapPointsId_;

    mvBackupKeyFrameOriginsId = mvBackupKeyFrameOriginsId_;
    //mvpBackupReferenceMapPointsId =  mvpBackupReferenceMapPointsId_; 
    //mbImuInitialized              =  mbImuInitialized_;              
    mnMapChange                   =  mnMapChange_;                   
    mnMapChangeNotified           =  mnMapChangeNotified_;           
    //mnInitKFid                    =  mnInitKFid_;                    
    mnMaxKFid                     =  mnMaxKFid_;                     
    mnBigChangeIdx                =  mnBigChangeIdx_;                
    mIsInUse                      =  mIsInUse_;                      
    //mHasTumbnail                  =  mHasTumbnail_;                  
    mbBad                         =  mbBad_;                         
    //mbIsInertial                  =  mbIsInertial_;                  
    //mbIMU_BA1                     =  mbIMU_BA1_;                     
    //mbIMU_BA2                     =  mbIMU_BA2_;                     
    mspErasedKeyFrameIds = msErasedKFIds;
    mspErasedMapPointIds = mspErasedMPIds;
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    //cout << "Thread1=Map::AddKeyFrame : Insert KF to map." << endl;
    unique_lock<mutex> lock(mMutexMap);
    if(mspKeyFrames.empty()){
        //cout << "Thread1=Map::AddKeyFrame : First KF:" << pKF->mnId << "; Map init KF:" << mnInitKFid << endl;
        mnInitKFid = pKF->mnId;
        mpKFinitial = pKF;
        mpKFlowerID = pKF;
    }
    notifyNewKeyFrameAdded(pKF);
    mspKeyFrames.insert(pKF);

    if(pKF->mnId>mnMaxKFid)
    {
        mnMaxKFid=pKF->mnId;
    }
    if(pKF->mnId<mpKFlowerID->mnId)
    {
        mpKFlowerID = pKF;
    }
}

void Map::AddMapPoint(MapPoint *pMP)
{
    {
        unique_lock<mutex> lock(mMutexMap);
        mspMapPoints.insert(pMP);
    }
    notifyNewMapPointCreated(pMP);
}

bool Map::CheckIfMapPointInMap(std::string mnTargetId)
{
    unique_lock<mutex> lock(mMutexMap);
    auto it = std::find_if(mspMapPoints.begin(), mspMapPoints.end(), [mnTargetId](const MapPoint* point) {
        return point->mstrHexId == mnTargetId;
    });

    if(it != mspMapPoints.end()) {
      return true;
    }

    return false;
}

void Map::SetImuInitialized()
{
    unique_lock<mutex> lock(mMutexMap);
    mbImuInitialized = true;
}

bool Map::isImuInitialized()
{
    unique_lock<mutex> lock(mMutexMap);
    return mbImuInitialized;
}

std::set<unsigned long int> Map::GetErasedKFIds()
{ 
    unique_lock<mutex> lock(mMutexMap);
    return mspErasedKeyFrameIds;
}


std::set<std::string>& Map::GetErasedMPIds()
{ 
    unique_lock<mutex> lock(mMutexMap);
    return mspErasedMapPointIds;
}

void Map::ClearErasedData()
{
    unique_lock<mutex> lock(mMutexMap);
    mspErasedMapPointIds.clear();
    mspErasedKeyFrameIds.clear();
}



std::set<unsigned long int> Map::GetUpdatedKFIds()
{ 
    unique_lock<mutex> lock(mMutexMap);
    return mspUpdatedKeyFrameIds;
}

void Map::ClearUpdatedKFIds()
{ 
    unique_lock<mutex> lock(mMutexMap);
    mspUpdatedKeyFrameIds.clear();
}


void Map::AddUpdatedKFId(unsigned long int id)
{ 
    unique_lock<mutex> lock(mMutexMap);
    mspUpdatedKeyFrameIds.insert(id);
}

std::set<std::string> Map::GetUpdatedMPIds()
{ 
    unique_lock<mutex> lock(mMutexMap);
    return mspUpdatedMapPointIds;
}

void Map::ClearUpdatedMPIds()
{ 
    unique_lock<mutex> lock(mMutexMap);
    mspUpdatedMapPointIds.clear();
}


void Map::AddUpdatedMPId(std::string id)
{ 
    unique_lock<mutex> lock(mMutexMap);
    mspUpdatedMapPointIds.insert(id);
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    if(!pMP)
    {
        mspMapPoints.erase(pMP);
        pMP = static_cast<MapPoint*>(NULL);
        return;
    }
    
    
    mspErasedMapPointIds.insert(pMP->mstrHexId);
    
    std::string idToRemove = pMP->mstrHexId;

    for(size_t i = 0; i < mvpReferenceMapPoints.size(); ++i)
    {
      MapPoint* mp = mvpReferenceMapPoints[i];
      if(mp)
      {
        if(mp->mstrHexId==idToRemove)
        {
          mvpReferenceMapPoints.erase(mvpReferenceMapPoints.begin() + i);
          break;
        }
      }
    }
    
    mspMapPoints.erase(pMP);
    pMP = static_cast<MapPoint*>(NULL);
    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(unsigned long int mnId)
{
    unique_lock<mutex> lock(mMutexMap);
    //mspKeyFrames.erase(pKF);
    mspErasedKeyFrameIds.insert(mnId);
    if(mspKeyFrames.size()>0)
    {
        if(mnId == mpKFlowerID->mnId)
        {
            vector<KeyFrame*> vpKFs = vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
            sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);
            mpKFlowerID = vpKFs[0];
        }
    }
    else
    {
        mpKFlowerID = 0;
    }

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}


void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);
    mspErasedKeyFrameIds.insert(pKF->mnId);
    if(mspKeyFrames.size()>0)
    {
        if(mpKFlowerID)
        {
            if(pKF->mnId == mpKFlowerID->mnId)
            {
                vector<KeyFrame*> vpKFs = vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
                sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);
                mpKFlowerID = vpKFs[0];
            }
        }
        else {
            vector<KeyFrame*> vpKFs = vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
            sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);
            mpKFlowerID = vpKFs[0];
        }
    }
    else
    {
        mpKFlowerID = 0;
    }

    pKF = static_cast<KeyFrame*>(NULL);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetId()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnId;
}
long unsigned int Map::GetInitKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnInitKFid;
}

void Map::SetInitKFid(long unsigned int initKFif)
{
    unique_lock<mutex> lock(mMutexMap);
    mnInitKFid = initKFif;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

KeyFrame* Map::GetOriginKF()
{
    return mpKFinitial;
}

void Map::SetCurrentMap()
{
    mIsInUse = true;
}

void Map::SetStoredMap()
{
    mIsInUse = false;
}

void Map::clear()
{
//    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
//        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
    {
        KeyFrame* pKF = *sit;
        pKF->UpdateMap(static_cast<Map*>(NULL));
//        delete *sit;
    }

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = mnInitKFid;
    mbImuInitialized = false;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
    mbIMU_BA1 = false;
    mbIMU_BA2 = false;
}

bool Map::IsInUse()
{
    return mIsInUse;
}

void Map::SetBad()
{
    mbBad = true;
}

bool Map::IsBad()
{
    return mbBad;
}


void Map::ApplyScaledRotation(const Sophus::SE3f &T, const float s, const bool bScaledVel)
{
    unique_lock<mutex> lock(mMutexMap);

    // Body position (IMU) of first keyframe is fixed to (0,0,0)
    Sophus::SE3f Tyw = T;
    Eigen::Matrix3f Ryw = Tyw.rotationMatrix();
    Eigen::Vector3f tyw = Tyw.translation();

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(); sit!=mspKeyFrames.end(); sit++)
    {
        KeyFrame* pKF = *sit;
        Sophus::SE3f Twc = pKF->GetPoseInverse();
        Twc.translation() *= s;
        Sophus::SE3f Tyc = Tyw*Twc;
        Sophus::SE3f Tcy = Tyc.inverse();
        pKF->SetPose(Tcy);
        Eigen::Vector3f Vw = pKF->GetVelocity();
        if(!bScaledVel)
            pKF->SetVelocity(Ryw*Vw);
        else
            pKF->SetVelocity(Ryw*Vw*s);

    }
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(); sit!=mspMapPoints.end(); sit++)
    {
        MapPoint* pMP = *sit;
        pMP->SetWorldPos(s * Ryw * pMP->GetWorldPos() + tyw);
        pMP->UpdateNormalAndDepth();
    }
    mnMapChange++;
}

void Map::SetInertialSensor()
{
    unique_lock<mutex> lock(mMutexMap);
    mbIsInertial = true;
}

bool Map::IsInertial()
{
    unique_lock<mutex> lock(mMutexMap);
    return mbIsInertial;
}

void Map::SetIniertialBA1()
{
    unique_lock<mutex> lock(mMutexMap);
    mbIMU_BA1 = true;
}

void Map::SetIniertialBA2()
{
    unique_lock<mutex> lock(mMutexMap);
    mbIMU_BA2 = true;
}

bool Map::GetIniertialBA1()
{
    unique_lock<mutex> lock(mMutexMap);
    return mbIMU_BA1;
}

bool Map::GetIniertialBA2()
{
    unique_lock<mutex> lock(mMutexMap);
    return mbIMU_BA2;
}

void Map::ChangeId(long unsigned int nId)
{
    mnId = nId;
}

unsigned int Map::GetLowerKFID()
{
    unique_lock<mutex> lock(mMutexMap);
    if (mpKFlowerID) {
        return mpKFlowerID->mnId;
    }
    return 0;
}

int Map::GetMapChangeIndex()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMapChange;
}

void Map::IncreaseChangeIndex()
{
    unique_lock<mutex> lock(mMutexMap);
    mnMapChange++;
}

int Map::GetLastMapChange()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMapChangeNotified;
}

void Map::SetLastMapChange(int currentChangeId)
{
    unique_lock<mutex> lock(mMutexMap);
    mnMapChangeNotified = currentChangeId;
}

void Map::PreSave(std::set<GeometricCamera*> &spCams)
{
    // Without lock, it crashes. SetBadFlag() is removed from EraseObservation() function
    // since its using the same lock which causes deadlock. Try to figure out a way to 
    // lock this without causing the deadlock.
    int nMPWithoutObs = 0;

    //for(MapPoint* pMPi : mspMapPoints)
    //for(std::set<ORB_SLAM3::MapPoint*>::iterator it = mspMapPoints.begin(); it != mspMapPoints.end(); ++it)
    //{
    //    ORB_SLAM3::MapPoint* pMPi = *it;
    //    if(!pMPi)
    //        continue;
    //    //std::cout << pMPi->mstrHexId << std::endl;
    //    if(pMPi->isBad())
    //        continue;
    //    
    //    if(pMPi->GetObservations().size() == 0)
    //    {
    //        nMPWithoutObs++;
    //    }

    //    map<KeyFrame*, std::tuple<int,int>> mpObs = pMPi->GetObservations();
    //    for(map<KeyFrame*, std::tuple<int,int>>::iterator it= mpObs.begin(), end=mpObs.end(); it!=end; ++it)
    //    {
    //        if(it->first->GetMap() != this || it->first->isBad())
    //        {
    //            pMPi->EraseObservation(it->first);
    //        }
    //    }
    //}




    // Saves the id of KF origins
    mvBackupKeyFrameOriginsId.clear();
    mvBackupKeyFrameOriginsId.reserve(mvpKeyFrameOrigins.size());
    for(int i = 0, numEl = mvpKeyFrameOrigins.size(); i < numEl; ++i)
    {
        mvBackupKeyFrameOriginsId.push_back(mvpKeyFrameOrigins[i]->mnId);
    }


    // Backup of MapPoints
    mvpBackupMapPoints.clear();
    mvpBackupMapPointsId.clear();
    
    for(std::set<ORB_SLAM3::MapPoint*>::iterator it = mspMapPoints.begin(); it != mspMapPoints.end(); ++it)
    {
        ORB_SLAM3::MapPoint* pMPi = *it;
        //std::cout << "MP->presave()" << std::endl;
        if(!pMPi) 
            continue;
        //ustd::cout << "pMPi exists " << pMPi->mstrHexId << std::endl;

        //if(!mspUpdatedMapPointIds.empty() && mspUpdatedMapPointIds.find(pMPi->mstrHexId) == mspUpdatedMapPointIds.end())
        //    continue;
        mvpBackupMapPoints.push_back(pMPi);
        mvpBackupMapPointsId.push_back(pMPi->mstrHexId);
        //pMPi->PreSave(mspKeyFrames,mspMapPoints);
    }

    mvpBackupReferenceMapPointsId.clear();
    for(MapPoint* pMPi : mvpReferenceMapPoints)
    {
      if(pMPi)
        mvpBackupReferenceMapPointsId.push_back(pMPi->mstrHexId);
    }

    // Backup of KeyFrames
    //std::cout << "before mspKeyFrames" << std::endl;
    mvpBackupKeyFrames.clear();
    mvpBackupKeyFramesId.clear();
    for(KeyFrame* pKFi : mspKeyFrames)
    {
        if(!pKFi)
            continue;
        //if(!mspUpdatedKeyFrameIds.empty() && mspUpdatedKeyFrameIds.find(pKFi->mnId) == mspUpdatedKeyFrameIds.end())
        //    continue;
        mvpBackupKeyFrames.push_back(pKFi);
        mvpBackupKeyFramesId.push_back(pKFi->mnId);
        //pKFi->PreSave(mspKeyFrames,mspMapPoints, spCams);
    }

    mnBackupKFinitialID = -1;
    if(mpKFinitial)
    {
        mnBackupKFinitialID = mpKFinitial->mnId;
    }

    mnBackupKFlowerID = -1;
    if(mpKFlowerID)
    {
        mnBackupKFlowerID = mpKFlowerID->mnId;
    }

}

void Map::PostLoad(KeyFrameDatabase* pKFDB, ORBVocabulary* pORBVoc, map<long unsigned int, KeyFrame*>& mpKeyFrameId, unordered_map<std::string, MapPoint*>& mpMapPointId, map<unsigned int, GeometricCamera*> &mpCams, bool* bUnprocessed, std::set<unsigned long int> mspUnprocKFids)
{
    lock_guard<mutex> lock2(mMutexLMUpdate);
    //if(mpKeyFrameId.empty() && mpMapPointId.empty())
    //{
    //  std::copy(mvpBackupMapPoints.begin(), mvpBackupMapPoints.end(), std::inserter(mspMapPoints, mspMapPoints.begin()));
    //  std::copy(mvpBackupKeyFrames.begin(), mvpBackupKeyFrames.end(), std::inserter(mspKeyFrames, mspKeyFrames.begin()));
    //  
    //  //map<long unsigned int,MapPoint*> mpMapPointId;
    //  for(MapPoint* pMPi : mspMapPoints)
    //  {
    //      if(!pMPi || pMPi->isBad())
    //          continue;

    //      pMPi->UpdateMap(this);
    //      mpMapPointId[pMPi->mstrHexId] = pMPi;
    //  }

    //  //map<long unsigned int, KeyFrame*> mpKeyFrameId;
    //  for(KeyFrame* pKFi : mspKeyFrames)
    //  {
    //      if(!pKFi || pKFi->isBad())
    //          continue; 

    //      pKFi->UpdateMap(this);
    //      pKFi->SetORBVocabulary(pORBVoc);
    //      pKFi->SetKeyFrameDatabase(pKFDB);
    //      mpKeyFrameId[pKFi->mnId] = pKFi;
    //  }
    //}
    //
    ////std::cout << "Insert KFs to map in postload=";
    //for(const auto& id : mvpBackupKeyFramesId)
    //{
    //  if(mpKeyFrameId[id])
    //  {
    //    //std::cout << id << ", ";
    //    mspKeyFrames.insert(mpKeyFrameId[id]);
    //  }
    //}
    ////std::cout << std::endl;

    ////std::cout << "before mps" << std::endl;
    //// TODO: Here we need to check the local mapping id and tracking id
    //for(const auto& id : mvpBackupMapPointsId)
    //{
    //  mspMapPoints.insert(mpMapPointId[id]);
    //}

    //std::cout << "before ref mps" << std::endl;
    mvpReferenceMapPoints.clear();
    for (const auto& id : mvpBackupReferenceMapPointsId)
    {
      if(!mpMapPointId[id] || mpMapPointId[id]->isBad())
          continue;
      mvpReferenceMapPoints.push_back(mpMapPointId[id]);
    }

    //std::cout << "Map::PostLoad() == #MPs=" <<mspUpdatedMapPointIds.size() << "/" << mspMapPoints.size() << ", #KFs=" << mspUpdatedKeyFrameIds.size() << "/" << mspKeyFrames.size() << std::endl;
    //// References reconstruction between different instances
    //for(MapPoint* pMPi : mspMapPoints)
    //{
    //    if(!pMPi || pMPi->isBad())
    //    {
    //      continue;
    //    }
    //    
    //    if(!mspUpdatedMapPointIds.empty() && mspUpdatedMapPointIds.find(pMPi->mstrHexId) == mspUpdatedMapPointIds.end())
    //      continue;
    //    bool mbTempUnprocessed = false;
    //    pMPi->PostLoad(mpKeyFrameId, mpMapPointId, &mbTempUnprocessed, mspUnprocKFids);
    //    pMPi->UpdateMap(this);

    //    *bUnprocessed = mbTempUnprocessed;
    //    if(mbTempUnprocessed)
    //      std::cout << "pMPi " << pMPi->mstrHexId << " is unprocessed" << std::endl;
    //}

    //for(KeyFrame* pKFi : mspKeyFrames)
    //{
    //    
    //    if(!pKFi || pKFi->isBad())
    //    {
    //      pKFDB->erase(pKFi);
    //      continue;
    //    }  
    //    
    //    if(!mspUpdatedKeyFrameIds.empty() && mspUpdatedKeyFrameIds.find(pKFi->mnId) == mspUpdatedKeyFrameIds.end())
    //      continue;
    //    
    //    bool mbTempUnprocessed = false;
    //    pKFi->PostLoad(mpKeyFrameId, mpMapPointId, mpCams, &mbTempUnprocessed, mspUnprocKFids);
    //    pKFi->UpdateMap(this);
    //    //pKFi->UpdateConnections();
    //    //pKFi->UpdateBestCovisibles();

    //    //pKFDB->add(pKFi);
    //    
    //    *bUnprocessed = mbTempUnprocessed;
    //    if(mbTempUnprocessed)
    //      std::cout << "pKFi " << pKFi->mnId << " is unprocessed" << std::endl;
    //}


    if(mnBackupKFinitialID != -1)
    {
        if(mpKeyFrameId.find(mnBackupKFinitialID) != mpKeyFrameId.end())
        {
          if(mpKeyFrameId[mnBackupKFinitialID])
              mpKFinitial = mpKeyFrameId[mnBackupKFinitialID];
        } else {
          mpKFinitial = mpKeyFrameId.begin()->second;
          mnBackupKFinitialID = mpKeyFrameId.begin()->first;
        }
    } else {
      mpKFinitial = mpKeyFrameId.begin()->second;
      mnBackupKFinitialID = mpKeyFrameId.begin()->first;
    }

    vector<KeyFrame*> vpKFs = vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);
    mpKFlowerID = vpKFs[0];
    mnBackupKFlowerID = mpKFlowerID->mnId;

    //if(mnBackupKFlowerID != -1)
    //{
    //    if(mpKeyFrameId.find(mnBackupKFlowerID) != mpKeyFrameId.end())
    //    {
    //      if(mpKeyFrameId[mnBackupKFlowerID])
    //          mpKFlowerID = mpKeyFrameId[mnBackupKFlowerID];
    //    } else {
    //      mpKFlowerID = mpKeyFrameId.begin()->second;
    //      mnBackupKFlowerID = mpKeyFrameId.begin()->first;
    //    }
    //} else {
    //  mpKFlowerID = mpKeyFrameId.begin()->second;
    //  mnBackupKFlowerID = mpKeyFrameId.begin()->first;
    //}

    mvpKeyFrameOrigins.clear();
    mvpKeyFrameOrigins.reserve(mvBackupKeyFrameOriginsId.size());
    for(int i = 0; i < mvBackupKeyFrameOriginsId.size(); ++i)
    {
        mvpKeyFrameOrigins.push_back(mpKeyFrameId[mvBackupKeyFrameOriginsId[i]]);
    }

    mvpBackupMapPoints.clear();
    ClearUpdatedKFIds();
    ClearUpdatedMPIds();
}

void Map::PostLoad(KeyFrameDatabase* pKFDB, ORBVocabulary* pORBVoc, map<long unsigned int, KeyFrame*>& mpKeyFrameId, map<std::string, MapPoint*>& mpMapPointId, map<unsigned int, GeometricCamera*> &mpCams, bool* bUnprocessed, std::set<unsigned long int> mspUnprocKFids)
{
    lock_guard<mutex> lock2(mMutexLMUpdate);
    //if(mpKeyFrameId.empty() && mpMapPointId.empty())
    //{
    //  std::copy(mvpBackupMapPoints.begin(), mvpBackupMapPoints.end(), std::inserter(mspMapPoints, mspMapPoints.begin()));
    //  std::copy(mvpBackupKeyFrames.begin(), mvpBackupKeyFrames.end(), std::inserter(mspKeyFrames, mspKeyFrames.begin()));
    //  
    //  //map<long unsigned int,MapPoint*> mpMapPointId;
    //  for(MapPoint* pMPi : mspMapPoints)
    //  {
    //      if(!pMPi || pMPi->isBad())
    //          continue;

    //      pMPi->UpdateMap(this);
    //      mpMapPointId[pMPi->mstrHexId] = pMPi;
    //  }

    //  //map<long unsigned int, KeyFrame*> mpKeyFrameId;
    //  for(KeyFrame* pKFi : mspKeyFrames)
    //  {
    //      if(!pKFi || pKFi->isBad())
    //          continue; 

    //      pKFi->UpdateMap(this);
    //      pKFi->SetORBVocabulary(pORBVoc);
    //      pKFi->SetKeyFrameDatabase(pKFDB);
    //      mpKeyFrameId[pKFi->mnId] = pKFi;
    //  }
    //}
    //
    ////std::cout << "Insert KFs to map in postload=";
    mspKeyFrames.clear();
    for(const auto& id : mvpBackupKeyFramesId)
    {
      if(mpKeyFrameId[id] && !mpKeyFrameId[id]->isBad() )
      {
        //std::cout << id << ", ";
        if(id>mnMaxKFid)
        {
          mnMaxKFid=id;
        }

        if(id<mnBackupKFinitialID)
        {
          mnBackupKFinitialID = id;
        }
        
        //if(mpKeyFrameId[id]->GetMap()->GetId() != this->mnId)
        //    mpKeyFrameId[id]->UpdateMap(this);

        mspKeyFrames.insert(mpKeyFrameId[id]);
            
      }
    }
    //std::cout << std::endl;

    ////std::cout << "before mps" << std::endl;
    //// TODO: Here we need to check the local mapping id and tracking id
    //mspMapPoints.clear();
    //for(const auto& id : mvpBackupMapPointsId)
    //{
    //  if(mpMapPointId[id] && !mpMapPointId[id]->isBad())
    //  {

    //    //if(mpMapPointId[id]->GetMap()->GetId() != this->mnId)
    //    //    mpMapPointId[id]->UpdateMap(this);
    //    
    //    mspMapPoints.insert(mpMapPointId[id]);
    //  }
    //}

    //std::cout << "before ref mps" << std::endl;
    mvpReferenceMapPoints.clear();
    for (const auto& id : mvpBackupReferenceMapPointsId)
    {
      if(!mpMapPointId[id] || mpMapPointId[id]->isBad() || mpMapPointId[id]->GetMap()->GetId() != this->mnId)
          continue;
      mvpReferenceMapPoints.push_back(mpMapPointId[id]);
    }

    //std::cout << "Map::PostLoad() == #MPs=" <<mspUpdatedMapPointIds.size() << "/" << mspMapPoints.size() << ", #KFs=" << mspUpdatedKeyFrameIds.size() << "/" << mspKeyFrames.size() << std::endl;
    //// References reconstruction between different instances
    //for(MapPoint* pMPi : mspMapPoints)
    //{
    //    if(!pMPi || pMPi->isBad())
    //    {
    //      continue;
    //    }
    //    
    //    if(!mspUpdatedMapPointIds.empty() && mspUpdatedMapPointIds.find(pMPi->mstrHexId) == mspUpdatedMapPointIds.end())
    //      continue;
    //    bool mbTempUnprocessed = false;
    //    pMPi->PostLoad(mpKeyFrameId, mpMapPointId, &mbTempUnprocessed, mspUnprocKFids);
    //    pMPi->UpdateMap(this);

    //    *bUnprocessed = mbTempUnprocessed;
    //    if(mbTempUnprocessed)
    //      std::cout << "pMPi " << pMPi->mstrHexId << " is unprocessed" << std::endl;
    //}

    //for(KeyFrame* pKFi : mspKeyFrames)
    //{
    //    
    //    if(!pKFi || pKFi->isBad())
    //    {
    //      pKFDB->erase(pKFi);
    //      continue;
    //    }  
    //    
    //    if(!mspUpdatedKeyFrameIds.empty() && mspUpdatedKeyFrameIds.find(pKFi->mnId) == mspUpdatedKeyFrameIds.end())
    //      continue;
    //    
    //    bool mbTempUnprocessed = false;
    //    pKFi->PostLoad(mpKeyFrameId, mpMapPointId, mpCams, &mbTempUnprocessed, mspUnprocKFids);
    //    pKFi->UpdateMap(this);
    //    //pKFi->UpdateConnections();
    //    //pKFi->UpdateBestCovisibles();

    //    //pKFDB->add(pKFi);
    //    
    //    *bUnprocessed = mbTempUnprocessed;
    //    if(mbTempUnprocessed)
    //      std::cout << "pKFi " << pKFi->mnId << " is unprocessed" << std::endl;
    //}


    if(mnBackupKFinitialID != -1)
    {
        if(mpKeyFrameId[mnBackupKFinitialID] && !mpKeyFrameId[mnBackupKFinitialID]->isBad() && (mpKeyFrameId[mnBackupKFinitialID]->GetMap() && mpKeyFrameId[mnBackupKFinitialID]->GetMap()->GetId() == this->mnId || !mpKeyFrameId[mnBackupKFinitialID]->GetMap()))
        {
              mpKFinitial = mpKeyFrameId[mnBackupKFinitialID];
        } else {
          for(std::map<unsigned long int, ORB_SLAM3::KeyFrame*>::iterator it = mpKeyFrameId.begin(); it!=mpKeyFrameId.end(); ++it)
          {
              if(it->second && !it->second->isBad() && it->second->GetMap() && it->second->GetMap()->GetId() == this->mnId)
              {
                  mpKFinitial = mpKeyFrameId.begin()->second;
                  mnBackupKFinitialID = mpKeyFrameId.begin()->first;
                  break;
              }
          }
        }
    } else {
        for(std::map<unsigned long int, ORB_SLAM3::KeyFrame*>::iterator it = mpKeyFrameId.begin(); it!=mpKeyFrameId.end(); ++it)
        {
            if(it->second && !it->second->isBad() &&it->second->GetMap()&& it->second->GetMap()->GetId() == this->mnId)
            {
                mpKFinitial = mpKeyFrameId.begin()->second;
                mnBackupKFinitialID = mpKeyFrameId.begin()->first;
                break;
            }
        }
    }

    vector<KeyFrame*> vpKFs = vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    mpKFinitial = static_cast<KeyFrame*>(NULL);
    mnBackupKFinitialID = -1;
    mpKFlowerID = static_cast<KeyFrame*>(NULL);
    mnBackupKFlowerID = -1;
    if(vpKFs.size() > 0)
    {
        for(auto const& pKF : vpKFs)
        {
            if(pKF && !pKF->isBad() && pKF->GetMap()->GetId() == this->mnId)
            {
                mpKFinitial = pKF;
                mnBackupKFinitialID = mpKFinitial->mnId;
                mpKFlowerID = pKF;
                mnBackupKFlowerID = mpKFlowerID->mnId;
                break;
            }
            
        }
    }
    //if(mnBackupKFlowerID != -1)
    //{
    //    if(mpKeyFrameId.find(mnBackupKFlowerID) != mpKeyFrameId.end())
    //    {
    //      if(mpKeyFrameId[mnBackupKFlowerID])
    //          mpKFlowerID = mpKeyFrameId[mnBackupKFlowerID];
    //    } else {
    //      mpKFlowerID = mpKeyFrameId.begin()->second;
    //      mnBackupKFlowerID = mpKeyFrameId.begin()->first;
    //    }
    //} else {
    //  mpKFlowerID = mpKeyFrameId.begin()->second;
    //  mnBackupKFlowerID = mpKeyFrameId.begin()->first;
    //}

    mvpKeyFrameOrigins.clear();
    mvpKeyFrameOrigins.reserve(mvBackupKeyFrameOriginsId.size());
    for(int i = 0; i < mvBackupKeyFrameOriginsId.size(); ++i)
    {
        if(mpKeyFrameId[mvBackupKeyFrameOriginsId[i]] && !mpKeyFrameId[mvBackupKeyFrameOriginsId[i]]->isBad() && mpKeyFrameId[mvBackupKeyFrameOriginsId[i]]->GetMap()->GetId() == this->mnId)
            mvpKeyFrameOrigins.push_back(mpKeyFrameId[mvBackupKeyFrameOriginsId[i]]);
    }

    mvpBackupMapPoints.clear();
    //ClearUpdatedKFIds();
    //ClearUpdatedMPIds();
}

std::vector<std::string> Map::GetBackupMapPointsId() {
  unique_lock<mutex> lock(mMutexMap);
  std::vector<std::string> ids;
  for(MapPoint* mp : mspMapPoints) {
    if(mp)
      ids.push_back(mp->mstrHexId);
  }
  return ids;
}
std::vector<long unsigned int> Map::GetBackupKeyFrames() {
  unique_lock<mutex> lock(mMutexMap);
  std::vector<long unsigned int> ids;
  for(KeyFrame* kf : mspKeyFrames) {
    ids.push_back(kf->mnId);
  }
  return ids;
}

long int Map::GetBackupKFInitialID() {
  unique_lock<mutex> lock(mMutexMap);
  return (mpKFinitial) ? mpKFinitial->mnId : -1; 
}

long int Map::GetBackupKFLowerID() {
  unique_lock<mutex> lock(mMutexMap);
 return (mpKFlowerID) ? mpKFlowerID->mnId : -1;
}

std::vector<std::string> Map::GetBackupReferenceMapPointsId() {
  unique_lock<mutex> lock(mMutexMap);
  std::set<std::string> ids;
  for(MapPoint* mp : mvpReferenceMapPoints) {
    if(!mp || mp->isBad())
        continue;
    ids.insert(mp->mstrHexId);
  } 

  std::vector<std::string> mvIds(ids.begin(), ids.end());
  return mvIds;
}

MapPoint* Map::RetrieveMapPoint(std::string id)
{
    //for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
    for(MapPoint* pMP : mspMapPoints)
    {
      std::string current_id = pMP->mstrHexId; //(isTracking)?(*sit)->mnId:(*sit)->lmMnId;
      if (current_id == id)
      {
          std::cout << " returning mp, ";
          return pMP;
      }
    }

    std::cout << " returning null, ";
    return NULL;
}

} //namespace ORB_SLAM3
