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


#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "KeyFrame.h"
#include "Frame.h"
#include "Map.h"
#include "Converter.h"

#include "SerializationUtils.h"

#include <opencv2/core/core.hpp>
#include <mutex>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/array.hpp>
#include <boost/serialization/map.hpp>

namespace ORB_SLAM3
{

class KeyFrame;
class Map;
class Frame;

class MapPoint
{

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        //ar & mnId;
        ar & mstrHexId;
        ar & mnFirstKFid;
        ar & mnFirstFrame;
        ar & nObs;
        // Variables used by the tracking
        //ar & mTrackProjX;
        //ar & mTrackProjY;
        //ar & mTrackDepth;
        //ar & mTrackDepthR;
        //ar & mTrackProjXR;
        //ar & mTrackProjYR;
        //ar & mbTrackInView;
        //ar & mbTrackInViewR;
        //ar & mnTrackScaleLevel;
        //ar & mnTrackScaleLevelR;
        //ar & mTrackViewCos;
        //ar & mTrackViewCosR;
        //ar & mnTrackReferenceForFrame;
        //ar & mnLastFrameSeen;

        // Variables used by local mapping
        //ar & mnBALocalForKF;
        //ar & mnFuseCandidateForKF;

        // Variables used by loop closing and merging
        //ar & mnLoopPointForKF;
        //ar & mnCorrectedByKF;
        //ar & mnCorrectedReference;
        //serializeMatrix(ar,mPosGBA,version);
        //ar & mnBAGlobalForKF;
        //ar & mnBALocalForMerge;
        //serializeMatrix(ar,mPosMerge,version);
        //serializeMatrix(ar,mNormalVectorMerge,version);

        // Protected variables
        ar & boost::serialization::make_array(mWorldPos.data(), mWorldPos.size());
        ar & boost::serialization::make_array(mNormalVector.data(), mNormalVector.size());
        //ar & BOOST_SERIALIZATION_NVP(mBackupObservationsId);
        //ar & mObservations;
        ar & mBackupObservationsId1;
        ar & mBackupObservationsId2;
        serializeMatrix(ar,mDescriptor,version);
        ar & mBackupRefKFId;
        //ar & mnVisible;
        //ar & mnFound;

        ar & mbBad;
        ar & mBackupReplacedStrId;

        ar & mfMinDistance;
        ar & mfMaxDistance;

    }


public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MapPoint();

    MapPoint(const Eigen::Vector3f &Pos, KeyFrame* pRefKF, Map* pMap);
    MapPoint(const double invDepth, cv::Point2f uv_init, KeyFrame* pRefKF, KeyFrame* pHostKF, Map* pMap);
    MapPoint(const Eigen::Vector3f &Pos,  Map* pMap, Frame* pFrame, const int &idxF);
    MapPoint(/*long unsigned int mnId,*/ std::string mstrHexId, long int mnFirstKFid, long int mnFirstFrame, int nObs, float mTrackProjX, float mTrackProjY, float mTrackDepth, float mTrackDepthR, float mTrackProjXR, float mTrackProjYR, bool mbTrackInView, bool mbTrackInViewR, int mnTrackScaleLevel, int mnTrackScaleLevelR,float mTrackViewCos, float mTrackViewCosR,long unsigned int mnTrackReferenceForFrame, long unsigned int mnLastFrameSeen,long unsigned int mnBALocalForKF,long unsigned int mnFuseCandidateForKF,long unsigned int mnLoopPointForKF, long unsigned int mnCorrectedByKF, long unsigned int mnCorrectedReference, Eigen::Vector3f mPosGBA, long unsigned int mnBAGlobalForKF, long unsigned int mnBALocalForMerge, Eigen::Vector3f mPosMerge, Eigen::Vector3f mNormalVectorMerge, double mInvDepth,double mInitU, double mInitV, /*KeyFrame* mpHostKF,*/ long long int mBackupHostKFId, unsigned int mnOriginMapId, Eigen::Vector3f mWorldPos, /*std::map<KeyFrame*,std::tuple<int,int> > mObservations,*/ std::map<long unsigned int, int> mBackupObservationsId1, std::map<long unsigned int, int> mBackupObservationsId2, Eigen::Vector3f mNormalVector, cv::Mat mDescriptor, /*KeyFrame* mpRefKF,*/ long long int mBackupRefKFId, int mnVisible, int mnFound, bool mbBad, /*MapPoint* mpReplaced,*/ /*long long int mBackupReplacedId,*/std::string mBackupReplacedStrId, float mfMinDistance, float mfMaxDistance /*Map* mpMap*/);

    void SetWorldPos(const Eigen::Vector3f &Pos, bool fromRos=false);
    Eigen::Vector3f GetWorldPos();

    Eigen::Vector3f GetNormal();
    void SetNormalVector(const Eigen::Vector3f& normal, bool fromRos=false);

    KeyFrame* GetReferenceKeyFrame();

    std::map<KeyFrame*,std::tuple<int,int>> GetObservations();
    std::map<long unsigned int, int> GetObservationsBackup1();
    std::map<long unsigned int, int> GetObservationsBackup2();
    int Observations();

    void AddObservation(KeyFrame* pKF,int idx, bool fromRos=false);
    void EraseObservation(KeyFrame* pKF, bool fromRos=false);

    std::tuple<int,int> GetIndexInKeyFrame(KeyFrame* pKF);
    bool IsInKeyFrame(KeyFrame* pKF);

    void SetBadFlag(bool fromRos=false);
    bool isBad();

    void Replace(MapPoint* pMP, bool fromRos=false);    
    MapPoint* GetReplaced();

    void IncreaseVisible(int n=1, bool fromRos=false);
    void IncreaseFound(int n=1, bool fromRos=false);
    float GetFoundRatio();
    inline int GetVisible(){
        return mnVisible;
    }
    inline int GetFound(){
        return mnFound;
    }
    //inline long int GetReplacedBackup()
    inline std::string GetReplacedBackup()
    {
      return mBackupReplacedStrId;
    }

    inline long int GetRefBackup()
    {
      return static_cast<long int>(mBackupRefKFId);
    }
    void ComputeDistinctiveDescriptors(bool fromRos=false);

    cv::Mat GetDescriptor();

    void UpdateNormalAndDepth(bool fromRos=false);

    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();
    int PredictScale(const float &currentDist, KeyFrame*pKF);
    int PredictScale(const float &currentDist, Frame* pF);

    Map* GetMap();
    void UpdateMap(Map* pMap, bool fromRos=false);

    void PrintObservations();
    std::string createHashId(const std::string& strSystemId, unsigned long int mnMPId);
    void PreSave(set<KeyFrame*>& spKF,set<MapPoint*>& spMP);
    void PostLoad(map<long unsigned int, KeyFrame*>& mpKFid, map<std::string, MapPoint*>& mpMPid, bool* bUnprocessed );
    
    void attachObserver(std::shared_ptr<Observer> observer) {
      observer_ = observer;
    }

public:
    //long unsigned int mnId;
    std::string mstrHexId;
    
    static long unsigned int nNextId;
    long int mnFirstKFid;
    long int mnFirstFrame;
    int nObs;

    // Variables used by the tracking
    float mTrackProjX;
    float mTrackProjY;
    float mTrackDepth;
    float mTrackDepthR;
    float mTrackProjXR;
    float mTrackProjYR;
    bool mbTrackInView, mbTrackInViewR;
    int mnTrackScaleLevel, mnTrackScaleLevelR;
    float mTrackViewCos, mTrackViewCosR;
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnLastFrameSeen;

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF;

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;    
    Eigen::Vector3f mPosGBA;
    long unsigned int mnBAGlobalForKF;
    long unsigned int mnBALocalForMerge;

    // Variable used by merging
    Eigen::Vector3f mPosMerge;
    Eigen::Vector3f mNormalVectorMerge;


    // Fopr inverse depth optimization
    double mInvDepth;
    double mInitU;
    double mInitV;
    KeyFrame* mpHostKF;
    long long int mBackupHostKFId;

    static std::mutex mGlobalMutex;

    unsigned int mnOriginMapId;

    inline void SetMpRefKF(KeyFrame* mpKF) {
      mpRefKF = mpKF;
    }

protected:    

     // Position in absolute coordinates
     Eigen::Vector3f mWorldPos;

     // Keyframes observing the point and associated index in keyframe
     std::map<KeyFrame*,std::tuple<int,int> > mObservations;
     // For save relation without pointer, this is necessary for save/load function
     std::map<long unsigned int, int> mBackupObservationsId1;
     std::map<long unsigned int, int> mBackupObservationsId2;

     // Mean viewing direction
     Eigen::Vector3f mNormalVector;

     // Best descriptor to fast matching
     cv::Mat mDescriptor;

     // Reference KeyFrame
     KeyFrame* mpRefKF;
     long long int mBackupRefKFId;

     // Tracking counters
     int mnVisible;
     int mnFound;

     // Bad flag (we do not currently erase MapPoint from memory)
     bool mbBad;
     MapPoint* mpReplaced;
     // For save relation without pointer, this is necessary for save/load function
     //long long int mBackupReplacedId;
     std::string mBackupReplacedStrId;

     // Scale invariance distances
     float mfMinDistance;
     float mfMaxDistance;

     Map* mpMap;

     // Mutex
     std::mutex mMutexPos;
     std::mutex mMutexFeatures;
     std::mutex mMutexMap;

    std::shared_ptr<Observer> observer_;
    
    void notifyObserverMapPointAdded(MapPoint* pMp) {
      if (observer_) {
        observer_->onMapPointAdded(pMp);
      }
    }
    

    void notifyObserverMapPointAction(unsigned long int hostMpId, int actionId, bool boolAction) {
      if (observer_) {
        observer_->onMapPointAction(hostMpId, actionId, boolAction);
      }
    }
    void notifyObserverMapPointAction(unsigned long int hostMpId, int actionId, unsigned long int id) {
      if (observer_) {
        observer_->onMapPointAction(hostMpId, actionId, id);
      }
    }
    void notifyObserverMapPointAction(unsigned long int hostMpId, int actionId, unsigned long int id, int idx) {
      if (observer_) {
        observer_->onMapPointAction(hostMpId, actionId, id, idx);
      }
    }
    void notifyObserverMapPointAction(unsigned long int hostMpId, int actionId, int n) {
      if (observer_) {
        observer_->onMapPointAction(hostMpId, actionId, n);
      }
    }
    void notifyObserverMapPointAction(unsigned long int hostMpId, int actionId, Eigen::Vector3f vec) {
      if (observer_) {
        observer_->onMapPointAction(hostMpId, actionId, vec);
      }
    }
};

} //namespace ORB_SLAM

#endif // MAPPOINT_H
