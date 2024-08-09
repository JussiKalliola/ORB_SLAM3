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


#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include "Distributor.h"

#include <set>
#include <pangolin/pangolin.h>
#include <mutex>

#include <boost/serialization/base_object.hpp>


namespace ORB_SLAM3
{

class MapPoint;
class KeyFrame;
class Atlas;
class KeyFrameDatabase;
class Distributor;

class Map
{
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar & mnId;
        ar & mnInitKFid;
        ar & mnMaxKFid;
        ar & mnBigChangeIdx;

        // Save/load a set structure, the set structure is broken in libboost 1.58 for ubuntu 16.04, a vector is serializated
        //ar & mspKeyFrames;
        //ar & mspMapPoints;
        //ar & mvpBackupKeyFrames;
        //ar & mvpBackupMapPoints;

        ar & mvpBackupReferenceMapPointsId;
        ar & mvpBackupMapPointsId;
        ar & mvpBackupKeyFramesId;

        ar & mvBackupKeyFrameOriginsId;

        ar & mnBackupKFinitialID;
        ar & mnBackupKFlowerID;

        ar & mbImuInitialized;
        ar & mbIsInertial;
        ar & mbIMU_BA1;
        ar & mbIMU_BA2;
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Map();
    Map(int initKFid);
    Map(const bool mbFail, const std::set<long unsigned int>& msOptKFs, const std::set<long unsigned int>& msFixedKFs, const long unsigned int mnId, 
        const std::vector<std::string>& mvpBackupMapPointsId, const std::vector<unsigned long int>& mvpBackupKeyFramesId, 
        const std::vector<unsigned long int>& mvBackupKeyFrameOriginsId, const unsigned long int mnBackupKFinitialID, const unsigned long int mnBackupKFlowerID, 
        const std::vector<std::string>& mvpBackupReferenceMapPointsId, const bool mbImuInitialized, const int mnMapChange, 
        const int mnMapChangeNotified, const long unsigned int mnInitKFid, const long unsigned int mnMaxKFid, const int mnBigChangeIdx, 
        const bool mIsInUse, const bool mHasTumbnail, const bool mbBad, const bool mbIsInertial, const bool mbIMU_BA1, const bool mbIMU_BA2, 
        const std::set<unsigned long int>& msErasedKFIds, const std::set<std::string>& mspErasedMPIds);
    ~Map();

    void UpdateMap(const Map &tempMap, const int nFromModule);
    void UpdateMap(const bool mbFail_, const std::set<long unsigned int>& msOptKFs_, const std::set<long unsigned int>& msFixedKFs_, 
        const long unsigned int mnId_, const std::vector<std::string>& mvpBackupMapPointsId_, const std::vector<unsigned long int>& mvpBackupKeyFramesId_, 
        const std::set<unsigned long int>& msUpdatedKFIds, const std::set<std::string>& msUpdatedMPIds, const std::vector<unsigned long int>& mvBackupKeyFrameOriginsId_, 
        const unsigned long int mnBackupKFinitialID_, const unsigned long int mnBackupKFlowerID_, const std::vector<std::string>& mvpBackupReferenceMapPointsId_, 
        const bool mbImuInitialized_, const int mnMapChange_, const int mnMapChangeNotified_, const long unsigned int mnInitKFid_, 
        const long unsigned int mnMaxKFid_, const int mnBigChangeIdx_, const bool mIsInUse_, /*const bool mHasTumbnail,*/ 
        const bool mbBad_ /*, const bool mbIsInertial, const bool mbIMU_BA1, const bool mbIMU_BA2*/, const std::set<unsigned long int>& msErasedKFIds, 
        const std::set<std::string>& mspErasedMPIds);
    void AddKeyFrame(KeyFrame* pKF);
    void AddMapPoint(MapPoint* pMP);
    void EraseMapPoint(MapPoint* pMP);
    void EraseKeyFrame(KeyFrame* pKF);
    void EraseKeyFrame(unsigned long int mnId);
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);
    void InformNewBigChange();
    int GetLastBigChangeIdx();

    std::vector<KeyFrame*> GetAllKeyFrames();
    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<MapPoint*> GetReferenceMapPoints();
    
    std::set<unsigned long int> GetErasedKFIds();
    std::set<std::string>& GetErasedMPIds();
    
    void ClearErasedData();

    long unsigned int MapPointsInMap();
    long unsigned  KeyFramesInMap();

    bool CheckIfMapPointInMap(std::string mnTargetId);

    long unsigned int GetId();

    long unsigned int GetInitKFid();
    void SetInitKFid(long unsigned int initKFif);
    long unsigned int GetMaxKFid();

    KeyFrame* GetOriginKF();

    void SetCurrentMap();
    void SetStoredMap();

    inline bool HasThumbnail() {
      return mHasTumbnail;
    }

    bool IsInUse();

    void SetBad();
    bool IsBad();

    void clear();

    int GetMapChangeIndex();
    void IncreaseChangeIndex();
    int GetLastMapChange();
    void SetLastMapChange(int currentChangeId);

    void SetImuInitialized();
    bool isImuInitialized();

    void ApplyScaledRotation(const Sophus::SE3f &T, const float s, const bool bScaledVel=false);

    void SetInertialSensor();
    bool IsInertial();
    void SetIniertialBA1();
    void SetIniertialBA2();
    bool GetIniertialBA1();
    bool GetIniertialBA2();

    void PrintEssentialGraph();
    bool CheckEssentialGraph();
    void ChangeId(long unsigned int nId);

    unsigned int GetLowerKFID();

    void PreSave(std::set<GeometricCamera*> &spCams);
    void PostLoad(KeyFrameDatabase* pKFDB, ORBVocabulary* pORBVoc, map<long unsigned int, KeyFrame*>& mpKeyFrameId, map<std::string, MapPoint*>& mpMapPointId, map<unsigned int, GeometricCamera*> &mpCams, bool* bUnprocessed, std::set<unsigned long int> mspUnprocKFids=std::set<unsigned long int>());
    void PostLoad(KeyFrameDatabase* pKFDB, ORBVocabulary* pORBVoc, map<long unsigned int, KeyFrame*>& mpKeyFrameId, unordered_map<std::string, MapPoint*>& mpMapPointId, map<unsigned int, GeometricCamera*> &mpCams, bool* bUnprocessed, std::set<unsigned long int> mspUnprocKFids=std::set<unsigned long int>());

    void printReprojectionError(list<KeyFrame*> &lpLocalWindowKFs, KeyFrame* mpCurrentKF, string &name, string &name_folder);

    vector<KeyFrame*> mvpKeyFrameOrigins;
    vector<unsigned long int> mvBackupKeyFrameOriginsId;
    KeyFrame* mpFirstRegionKF;
    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;

    bool mbFail;

    // Size of the thumbnail (always in power of 2)
    static const int THUMB_WIDTH = 512;
    static const int THUMB_HEIGHT = 512;

    static long unsigned int nNextId;

    // DEBUG: show KFs which are used in LBA
    std::set<long unsigned int> msOptKFs;
    std::set<long unsigned int> msFixedKFs;

    std::set<unsigned long int> GetUpdatedKFIds();
    void AddUpdatedKFId(unsigned long int id);
    void ClearUpdatedKFIds();
    std::set<std::string> GetUpdatedMPIds();
    void AddUpdatedMPId(std::string id);
    void ClearUpdatedMPIds();

    inline std::vector<long unsigned int> GetOptKFs() {
      std::vector<long unsigned int> ids(msOptKFs.begin(), msOptKFs.end());
      return ids;
    }

    std::vector<long unsigned int> GetFixedKFs() {
      std::vector<long unsigned int> ids(msFixedKFs.begin(), msFixedKFs.end());
      return ids;
    }
    
    std::vector<std::string> GetBackupMapPointsId();
    std::vector<long unsigned int> GetBackupKeyFrames();
    long int GetBackupKFInitialID();
    long int GetBackupKFLowerID();
    std::vector<std::string> GetBackupReferenceMapPointsId();

    inline bool GetImuInitialized() {
      return mbImuInitialized;
    }

    inline int GetMapChange() {
      return mnMapChange;
    }

    inline int GetMapChangeNotified() {
      return mnMapChangeNotified;
    }

    inline int GetBigChangeIdx() {
      return mnBigChangeIdx;
    }

    inline bool GetIsBad() {
      return mbBad;
    }

    void attachDistributor(std::shared_ptr<Distributor> distributor) {
      distributor_ = distributor;
    }
    
    std::shared_ptr<Distributor> GetDistributor() {
      return distributor_;
    }
    
    void UpdateMapPoints(std::set<MapPoint*> mspMPs)
    {
      unique_lock<mutex> lock(mMutexMap);
      mspMapPoints=mspMPs;
      // UPDATE BACKUP IDS
    }

    MapPoint* RetrieveMapPoint(std::string id);

    void UpdateKeyFrames(std::set<KeyFrame*> mspKFs)
    {
      unique_lock<mutex> lock(mMutexMap);
      mspKeyFrames=mspKFs;
      // UPDATE BACKUP IDS
    }

    void UpdateInitialKF(KeyFrame* pKF)
    {
      unique_lock<mutex> lock(mMutexMap);
      mpKFinitial=pKF;
      // UPDATE BACKUP IDS
    }

    void UpdateLowerKF(KeyFrame* pKF)
    {
      unique_lock<mutex> lock(mMutexMap);
      mpKFlowerID=pKF;
      // UPDATE BACKUP IDS
    }

    void UpdateReferenceMapPoints(std::vector<MapPoint*> mvpMPs)
    {
      unique_lock<mutex> lock(mMutexMap);
      mvpReferenceMapPoints=mvpMPs;
      // UPDATE BACKUP IDS
    }




    void UpdateMap(Map* pM) {
      
      mbImuInitialized = pM->mbImuInitialized;

      mnMapChange = pM->mnMapChange;
      mnMapChangeNotified = pM->mnMapChangeNotified;

      mnInitKFid = pM->mnInitKFid;
      mnMaxKFid = pM->mnMaxKFid;
    //long unsigned int mnLastLoopKFid;

    // Index related to a big change in the map (loop closure, global BA)
      mnBigChangeIdx = pM->mnBigChangeIdx;


      mIsInUse = pM->mIsInUse;
      mHasTumbnail = pM->mHasTumbnail;
      mbBad = pM->mbBad;

      mbIsInertial = pM->mbIsInertial;
      mbIMU_BA1 = pM->mbIMU_BA1;
      mbIMU_BA2 = pM->mbIMU_BA2;
      // HERE UPDATE BACKUP IDS
    }

protected:

    void notifyNewMapPointCreated(ORB_SLAM3::MapPoint* pMP)
    {
      if (distributor_) {
        distributor_->onNewMapPoint(pMP);
      }

    }
    
    void notifyNewKeyFrameAdded(ORB_SLAM3::KeyFrame* pKF)
    {
      if (distributor_) {
        distributor_->onNewKeyFrame(pKF);
      }

    }

    std::shared_ptr<Distributor> distributor_;
    
    long unsigned int mnId;

    std::set<MapPoint*> mspMapPoints;
    std::set<KeyFrame*> mspKeyFrames;

    // Save/load, the set structure is broken in libboost 1.58 for ubuntu 16.04, a vector is serializated
    std::vector<MapPoint*> mvpBackupMapPoints;
    std::vector<KeyFrame*> mvpBackupKeyFrames;

    std::set<unsigned long int> mspErasedKeyFrameIds;
    std::set<std::string> mspErasedMapPointIds;

    std::set<unsigned long int> mspUpdatedKeyFrameIds;
    std::set<std::string> mspUpdatedMapPointIds;

    std::vector<std::string> mvpBackupMapPointsId;
    std::vector<unsigned long int> mvpBackupKeyFramesId;

    KeyFrame* mpKFinitial;
    KeyFrame* mpKFlowerID;

    unsigned long int mnBackupKFinitialID;
    unsigned long int mnBackupKFlowerID;

    std::vector<MapPoint*> mvpReferenceMapPoints;
    std::vector<std::string> mvpBackupReferenceMapPointsId;

    bool mbImuInitialized;

    int mnMapChange;
    int mnMapChangeNotified;

    long unsigned int mnInitKFid;
    long unsigned int mnMaxKFid;
    //long unsigned int mnLastLoopKFid;

    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;


    // View of the map in aerial sight (for the AtlasViewer)
    GLubyte* mThumbnail;

    bool mIsInUse;
    bool mHasTumbnail;
    bool mbBad = false;

    bool mbIsInertial;
    bool mbIMU_BA1;
    bool mbIMU_BA2;

    // Mutex
    std::mutex mMutexMap;
    std::mutex mMutexMapPreSave;
    std::mutex mMutexLMUpdate;
};

} //namespace ORB_SLAM3

#endif // MAP_H
