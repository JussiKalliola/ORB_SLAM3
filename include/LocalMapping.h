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


#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "KeyFrame.h"
#include "Atlas.h"
#include "LoopClosing.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"
#include "Settings.h"

#include <mutex>


namespace ORB_SLAM3
{

class System;
class Tracking;
class LoopClosing;
class Atlas;

class LocalMapping
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LocalMapping(System* pSys, Atlas* pAtlas, const float bMonocular, bool bInertial, const string &_strSeqName=std::string());

    void SetLoopCloser(LoopClosing* pLoopCloser);
    
    void AllowLocalMapping(bool mbAllow);

    void SetTracker(Tracking* pTracker);

    // Main function
    void Run();

    void InsertKeyFrame(KeyFrame* pKF);
    void EmptyQueue();

    // Thread Synch
    void RequestStop();
    void RequestReset();
    void RequestResetActiveMap(Map* pMap);
    bool Stop();
    void Release();
    bool isStopped();
    bool stopRequested();
    bool AcceptKeyFrames();
    void SetAcceptKeyFrames(bool flag);
    bool SetNotStop(bool flag);
    void SetLocalMappingActive(bool flag);
    bool IsLMRunning();

    void InterruptBA();

    void RequestFinish();
    bool isFinished();

    int KeyframesInQueue(){
        unique_lock<std::mutex> lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }

    bool IsInitializing();
    double GetCurrKFTime();
    KeyFrame* GetCurrKF();

    std::mutex mMutexImuInit;

    Eigen::MatrixXd mcovInertial;
    Eigen::Matrix3d mRwg;
    Eigen::Vector3d mbg;
    Eigen::Vector3d mba;
    double mScale;
    double mInitTime;
    double mCostTime;

    unsigned int mInitSect;
    unsigned int mIdxInit;
    unsigned int mnKFs;
    double mFirstTs;
    int mnMatchesInliers;

    // For debugging (erase in normal mode)
    int mInitFr;
    int mIdxIteration;
    string strSequence;

    bool mbNotBA1;
    bool mbNotBA2;
    bool mbBadImu;

    bool mbWriteStats;

    // not consider far points (clouds)
    bool mbFarPoints;
    float mThFarPoints;

    bool mbGBARunning;

#ifdef REGISTER_TIMES
    vector<double> vdKFInsert_ms;
    vector<double> vdMPCulling_ms;
    vector<double> vdMPCreation_ms;
    vector<double> vdLBA_ms;
    vector<double> vdKFCulling_ms;
    vector<double> vdLMTotal_ms;

    vector<std::chrono::steady_clock::time_point> vtStartTimeLM_ms;
    vector<std::chrono::steady_clock::time_point> vtStartTimeLBA_ms;

    vector<double> vdLBASync_ms;
    vector<double> vdKFCullingSync_ms;
    vector<int> vnLBA_edges;
    vector<int> vnLBA_KFopt;
    vector<int> vnLBA_KFfixed;
    vector<int> vnLBA_MPs;
    int nLBA_exec;
    int nLBA_abort;
#endif


    void InsertKeyframeFromRos(KeyFrame* pKF);
    bool NeedNewKeyFrame(KeyFrame* pKF);

    void attachDistributor(std::shared_ptr<Distributor> distributor) {
      distributor_ = distributor;
    } 
    
    void notifyDistributorAddKeyframe(KeyFrame* pKF, std::set<std::string> msNewMapPointIds) {
      if (distributor_) {
        distributor_->onKeyframeAdded(pKF, msNewMapPointIds);
      }
    }

    void notifyStopRequest() {
      if (distributor_) {
        distributor_->onLMStopRequest();
      }
    }

    void notifyDistributorLocalMapUpdated(Map* pM) {
      unique_lock<std::mutex> lock(mMutexNewKFs);
      if (distributor_) { 
        distributor_->onLocalMapUpdated(pM);
      }
    }

    //void notifyDistributorLMActive(bool bActive) {
    //  //unique_lock<std::mutex> lock(mMutexNewKFs);
    //  if (distributor_) { 
    //    distributor_->onChangeLMActive(bActive);
    //  }
    //}


protected:

    bool CheckNewKeyFrames();
    void ProcessNewKeyFrame();
    void CreateNewMapPoints();

    void MapPointCulling();
    void SearchInNeighbors();
    void KeyFrameCulling();

    System *mpSystem;

    bool mbMonocular;
    bool mbInertial;

    void ResetIfRequested();
    bool mbResetRequested;
    bool mbResetRequestedActiveMap;
    Map* mpMapToReset;
    std::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    Atlas* mpAtlas;

    LoopClosing* mpLoopCloser;
    Tracking* mpTracker;

    std::list<KeyFrame*> mlNewKeyFrames;

    KeyFrame* mpCurrentKeyFrame;

    std::list<MapPoint*> mlpRecentAddedMapPoints;
    std::set<std::string> msNewMapPointIds;

    std::mutex mMutexNewKFs;
    
    bool mbAllowLM;
    
    bool mbAbortBA;

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    std::mutex mMutexStop;

    bool mbAcceptKeyFrames;
    bool mbIsRunning;
    std::mutex mMutexAccept;
    
    std::mutex mMutexProcessKFs;
    void InitializeIMU(float priorG = 1e2, float priorA = 1e6, bool bFirst = false);
    void ScaleRefinement();

    bool bInitializing;

    Eigen::MatrixXd infoInertial;
    int mNumLM;
    int mNumKFCulling;

    float mTinit;

    int countRefinement;

    //DEBUG
    ofstream f_lm;

    std::chrono::high_resolution_clock::time_point msLastMUStart;
    std::chrono::high_resolution_clock::time_point msLastMUStop;
    
    unsigned long int mnLastKeyFrameId;
    bool mbKFsAfterMapUpdate;

    int MAP_FREQ;  // Set to: after how many ms from last map update, a new map update should be sent
    int KF_NUM;    // Set to: how many keyframes should a map update consist of
    int CONN_KF;   // Set to: for every keyframe, how many connected keyframes should be included in a map update

private:
    std::shared_ptr<Distributor> distributor_;


};

} //namespace ORB_SLAM

#endif // LOCALMAPPING_H
