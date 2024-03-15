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


#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"
#include "ImuTypes.h"
#include "Observer.h"

#include "GeometricCamera.h"
#include "SerializationUtils.h"

#include <mutex>

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>


namespace ORB_SLAM3
{

class Map;
class MapPoint;
class Frame;
class KeyFrameDatabase;
class Observer;

class GeometricCamera;

class KeyFrame
{
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        //ar & mnId;
        //ar & const_cast<long unsigned int&>(mnFrameId);
        //ar & const_cast<double&>(mTimeStamp);
        //// Grid
        //ar & const_cast<int&>(mnGridCols);
        //ar & const_cast<int&>(mnGridRows);
        //ar & const_cast<float&>(mfGridElementWidthInv);
        //ar & const_cast<float&>(mfGridElementHeightInv);

        //// Variables of tracking
        //ar & mnTrackReferenceForFrame;
        //ar & mnFuseTargetForKF;
        //// Variables of local mapping
        //ar & mnBALocalForKF;
        //ar & mnBAFixedForKF;
        //ar & mnNumberOfOpt;
        //// Variables used by KeyFrameDatabase
        //ar & mnLoopQuery;
        //ar & mnLoopWords;
        //ar & mLoopScore;
        //ar & mnRelocQuery;
        //ar & mnRelocWords;
        //ar & mRelocScore;
        //ar & mnMergeQuery;
        //ar & mnMergeWords;
        //ar & mMergeScore;
        //ar & mnPlaceRecognitionQuery;
        //ar & mnPlaceRecognitionWords;
        //ar & mPlaceRecognitionScore;
        //ar & mbCurrentPlaceRecognition;
        //// Variables of loop closing
        //serializeSophusSE3<Archive>(ar,mTcwGBA,version);
        //serializeSophusSE3<Archive>(ar,mTcwBefGBA,version);
        ////ar & boost::serialization::make_array(mVwbGBA.data(), mVwbGBA.size());
        ////ar & boost::serialization::make_array(mVwbBefGBA.data(), mVwbBefGBA.size());
        //ar & mBiasGBA;
        //ar & mnBAGlobalForKF;
        //// Variables of Merging
        //serializeSophusSE3<Archive>(ar,mTcwMerge,version);
        //serializeSophusSE3<Archive>(ar,mTcwBefMerge,version);
        //serializeSophusSE3<Archive>(ar,mTwcBefMerge,version);
        ////ar & boost::serialization::make_array(mVwbMerge.data(), mVwbMerge.size());
        ////ar & boost::serialization::make_array(mVwbBefMerge.data(), mVwbBefMerge.size());
        //ar & mBiasMerge;
        //ar & mnMergeCorrectedForKF;
        //ar & mnMergeForKF;
        //ar & mfScaleMerge;
        //ar & mnBALocalForMerge;

        //// Scale
        //ar & mfScale;
        //// Calibration parameters
        //ar & const_cast<float&>(fx);
        //ar & const_cast<float&>(fy);
        //ar & const_cast<float&>(invfx);
        //ar & const_cast<float&>(invfy);
        //ar & const_cast<float&>(cx);
        //ar & const_cast<float&>(cy);
        //ar & const_cast<float&>(mbf);
        //ar & const_cast<float&>(mb);
        //ar & const_cast<float&>(mThDepth);
        //serializeMatrix(ar, mDistCoef, version);
        //// Number of Keypoints
        //ar & const_cast<int&>(N);
        //// KeyPoints
        serializeVectorKeyPoints<Archive>(ar, mvKeys, version);
        //serializeVectorKeyPoints<Archive>(ar, mvKeysUn, version);
        //ar & const_cast<vector<float>& >(mvuRight);
        //ar & const_cast<vector<float>& >(mvDepth);
        serializeMatrix<Archive>(ar,mDescriptors,version);
        // BOW
        //ar & mBowVec;
        //ar & mFeatVec;
        // Pose relative to parent
        //serializeSophusSE3<Archive>(ar, mTcp, version);
        //// Scale
        //ar & const_cast<int&>(mnScaleLevels);
        //ar & const_cast<float&>(mfScaleFactor);
        //ar & const_cast<float&>(mfLogScaleFactor);
        //ar & const_cast<vector<float>& >(mvScaleFactors);
        //ar & const_cast<vector<float>& >(mvLevelSigma2);
        //ar & const_cast<vector<float>& >(mvInvLevelSigma2);
        //// Image bounds and calibration
        //ar & const_cast<int&>(mnMinX);
        //ar & const_cast<int&>(mnMinY);
        //ar & const_cast<int&>(mnMaxX);
        //ar & const_cast<int&>(mnMaxY);
        ////ar & boost::serialization::make_array(mK_.data(), mK_.size());
        //// Pose
        //serializeSophusSE3<Archive>(ar, mTcw, version);
        //// MapPointsId associated to keypoints
        //ar & mvBackupMapPointsId;
        //// Grid
        //ar & mGrid;
        //// Connected KeyFrameWeight
        //ar & mBackupConnectedKeyFrameIdWeights;
        //// Spanning Tree and Loop Edges
        //ar & mbFirstConnection;
        //ar & mBackupParentId;
        //ar & mvBackupChildrensId;
        //ar & mvBackupLoopEdgesId;
        //ar & mvBackupMergeEdgesId;
        //// Bad flags
        //ar & mbNotErase;
        //ar & mbToBeErased;
        //ar & mbBad;

        //ar & mHalfBaseline;

        //ar & mnOriginMapId;

        // Camera variables
        //ar & mnBackupIdCamera;
        //ar & mnBackupIdCamera2;

        // Fisheye variables
        //ar & mvLeftToRightMatch;
        //ar & mvRightToLeftMatch;
        //ar & const_cast<int&>(NLeft);
        //ar & const_cast<int&>(NRight);
        //serializeSophusSE3<Archive>(ar, mTlr, version);
        //serializeVectorKeyPoints<Archive>(ar, mvKeysRight, version);
        //ar & mGridRight;

        // Inertial variables
        //ar & mImuBias;
        //ar & mBackupImuPreintegrated;
        //ar & mImuCalib;
        //ar & mBackupPrevKFId;
        //ar & mBackupNextKFId;
        //ar & bImu;
        //ar & boost::serialization::make_array(mVw.data(), mVw.size());
        //ar & boost::serialization::make_array(mOwb.data(), mOwb.size());
        //ar & mbHasVelocity;
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    KeyFrame();
    KeyFrame(Frame &F, Map* pMap, KeyFrameDatabase* pKFDB);

    // Constructor for ROS message
    KeyFrame(bool bImu, long unsigned int nNextId, long unsigned int mnId, const long unsigned int mnFrameId, const double mTimeStamp, const int mnGridCols, const int mnGridRows, const float mfGridElementWidthInv, const float mfGridElementHeightInv, long unsigned int mnTrackReferenceForFrame, long unsigned int mnFuseTargetForKF, long unsigned int mnBALocalForKF, long unsigned int mnBAFixedForKF, long unsigned int mnNumberOfOpt, long unsigned int mnLoopQuery, int mnLoopWords, float mLoopScore, long unsigned int mnRelocQuery, int mnRelocWords, float mRelocScore, long unsigned int mnMergeQuery, int mnMergeWords, float mMergeScore, long unsigned int mnPlaceRecognitionQuery, int mnPlaceRecognitionWords, float mPlaceRecognitionScore, bool mbCurrentPlaceRecognition, Sophus::SE3f mTcwGBA, Sophus::SE3f mTcwBefGBA, Eigen::Vector3f mVwbGBA, Eigen::Vector3f mVwbBefGBA, IMU::Bias mBiasGBA, long unsigned int mnBAGlobalForKF, Sophus::SE3f mTcwMerge, Sophus::SE3f mTcwBefMerge, Sophus::SE3f mTwcBefMerge, Eigen::Vector3f mVwbMerge, Eigen::Vector3f mVwbBefMerge, IMU::Bias mBiasMerge, long unsigned int mnMergeCorrectedForKF, long unsigned int mnMergeForKF, float mfScaleMerge, long unsigned int mnBALocalForMerge, float mfScale, const float fx, const float fy, const float cx, const float cy, const float invfx, const float invfy, const float mbf, const float mb, const float mThDepth, cv::Mat mDistCoef, const int N, const std::vector<cv::KeyPoint> mvKeys, const std::vector<cv::KeyPoint> mvKeysUn, const std::vector<float> mvuRight, const std::vector<float> mvDepth, const cv::Mat mDescriptors, /*DBoW2::BowVector mBowVec, DBoW2::FeatureVector mFeatVec, */ Sophus::SE3f mTcp, const int mnScaleLevels, const float mfScaleFactor, const float mfLogScaleFactor, const std::vector<float> mvScaleFactors, const std::vector<float> mvLevelSigma2, const std::vector<float> mvInvLevelSigma2, const int mnMinX, const int mnMinY, const int mnMaxX, const int mnMaxY, /*KeyFrame* mPrevKF, KeyFrame* mNextKF,*/ 
        //IMU::Preintegrated* mpImuPreintegrated, 
        IMU::Calib mImuCalib, unsigned int mnOriginMapId, string mNameFile, int mnDataset, /*std::vector <KeyFrame*> mvpLoopCandKFs, std::vector <KeyFrame*> mvpMergeCandKFs,*/ std::vector <unsigned long int> mvLoopCandKFIds, std::vector <unsigned long int> mvMergeCandKFIds ,Sophus::SE3<float> mTcw, /*Eigen::Matrix3f mRcw, Sophus::SE3<float> mTwc, Eigen::Matrix3f mRwc, Eigen::Vector3f mOwb, Eigen::Vector3f mVw,*/ bool mbHasVelocity, Sophus::SE3<float> mTlr, Sophus::SE3<float> mTrl, IMU::Bias mImuBias, /*std::vector<MapPoint*> mvpMapPoints,*/ std::vector<std::string> mvBackupMapPointsId, /*KeyFrameDatabase* mpKeyFrameDB, ORBVocabulary* mpORBvocabulary,*/ std::vector< std::vector <std::vector<size_t> > > mGrid, /*std::map<KeyFrame*,int> mConnectedKeyFrameWeights, std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames,*/ std::vector<int> mvOrderedWeights, std::map<long unsigned int, int> mBackupConnectedKeyFrameIdWeights, bool mbFirstConnection, /*KeyFrame* mpParent, std::set<KeyFrame*> mspChildrens, std::set<KeyFrame*> mspLoopEdges, std::set<KeyFrame*> mspMergeEdges,*/ long long int mBackupParentId, std::vector<long unsigned int> mvBackupChildrensId, std::vector<long unsigned int> mvBackupLoopEdgesId, std::vector<long unsigned int> mvBackupMergeEdgesId, bool mbNotErase, bool mbToBeErased, bool mbBad, float mHalfBaseline, /*Map* mpMap,*/ long long int mBackupPrevKFId, long long int mBackupNextKFId, 
        //IMU::Preintegrated* mBackupImuPreintegrated, 
        unsigned int mnBackupIdCamera, unsigned int mnBackupIdCamera2, Eigen::Matrix3f mK_, unsigned int mnLastModule, /*GeometricCamera* mpCamera, GeometricCamera* mpCamera2,*/ std::vector<int> mvLeftToRightMatch, std::vector<int> mvRightToLeftMatch, const std::vector<cv::KeyPoint> mvKeysRight, const int NLeft, const int NRight, std::vector< std::vector <std::vector<size_t> > > mGridRight);
    

    void UpdateKeyFrame( bool bImu_, /*const long unsigned int mnFrameId_,*/ /*const double mTimeStamp_,*//*const int mnGridCols_, *//*const int mnGridRows_, *//*const float mfGridElementWidthInv_, *//*const float mfGridElementHeightInv_,*/ /*long unsigned int mnTrackReferenceForFrame_,long unsigned int mnFuseTargetForKF_,*/ long unsigned int mnBALocalForKF_, long unsigned int mnBAFixedForKF_, long unsigned int mnNumberOfOpt_, long unsigned int mnLoopQuery_, int mnLoopWords_, float mLoopScore_, long unsigned int mnRelocQuery_, int mnRelocWords_, float mRelocScore_, long unsigned int mnMergeQuery_, int mnMergeWords_, float mMergeScore_, long unsigned int mnPlaceRecognitionQuery_, int mnPlaceRecognitionWords_, float mPlaceRecognitionScore_, bool mbCurrentPlaceRecognition_, Sophus::SE3f mTcwGBA_, Sophus::SE3f mTcwBefGBA_, Eigen::Vector3f mVwbGBA_, Eigen::Vector3f mVwbBefGBA_, IMU::Bias mBiasGBA_, long unsigned int mnBAGlobalForKF_, Sophus::SE3f mTcwMerge_, Sophus::SE3f mTcwBefMerge_, Sophus::SE3f mTwcBefMerge_, Eigen::Vector3f mVwbMerge_, Eigen::Vector3f mVwbBefMerge_, IMU::Bias mBiasMerge_, long unsigned int mnMergeCorrectedForKF_, long unsigned int mnMergeForKF_, float mfScaleMerge_, long unsigned int mnBALocalForMerge_, float mfScale_, 
    /*const float fx_, *//*const float fy_, *//*const float cx_, *//*const float cy_, *//*const float invfx_, *//*const float invfy_, *//*const float mbf_, *//*const float mb_, *//*const float mThDepth_,*/ /*cv::Mat mDistCoef_,*/ /*const int N_, *//*const std::vector<cv::KeyPoint> mvKeys_, *//*const std::vector<cv::KeyPoint> mvKeysUn_, *//*const std::vector<float> mvuRight_, *//*const std::vector<float> mvDepth_, *//*const cv::Mat mDescriptors_, *//*DBoW2::BowVector mBowVec_, DBoW2::FeatureVector mFeatVec_,*/ Sophus::SE3f mTcp_, /*const int mnScaleLevels_, *//*const float mfScaleFactor_, *//*const float mfLogScaleFactor_, *//*const std::vector<float> mvScaleFactors_, *//*const std::vector<float> mvLevelSigma2_, *//*const std::vector<float> mvInvLevelSigma2_, *//*const int mnMinX_, *//*const int mnMinY_, *//*const int mnMaxX_, *//*const int mnMaxY_, *//*KeyFrame* mPrevKF = nullptr, *//*KeyFrame* mNextKF = nullptr, *//*IMU::Preintegrated* mpImuPreintegrated = nullptr, */ /*IMU::Calib mImuCalib_, unsigned int mnOriginMapId_, string mNameFile_, int mnDataset_,*/ /*std::vector <KeyFrame*> mvpLoopCandKFs = std::vector <KeyFrame*>(), */ /*std::vector <KeyFrame*> mvpMergeCandKFs = std::vector <KeyFrame*>(), */ std::vector <unsigned long int> mvLoopCandKFIds_, std::vector <unsigned long int> mvMergeCandKFIds_, Sophus::SE3<float> mTcw_, /*Eigen::Matrix3f mRcw = Eigen::Matrix3f(), *//*Sophus::SE3<float> mTwc = Sophus::SE3<float>(),*/ /*Eigen::Matrix3f mRwc = Eigen::Matrix3f(), *//*Eigen::Vector3f mOwb = Eigen::Vector3f(), *//*Eigen::Vector3f mVw = Eigen::Vector3f(), */bool mbHasVelocity_, Sophus::SE3<float> mTlr_, Sophus::SE3<float> mTrl_, /*IMU::Bias mImuBias_,*/ /*std::vector<MapPoint*> mvpMapPoints = std::vector<MapPoint*>(), */std::vector<std::string> mvBackupMapPointsId_, /*KeyFrameDatabase* mpKeyFrameDB = nullptr, */ /*ORBVocabulary* mpORBvocabulary = nullptr, */ /*std::vector< std::vector <std::vector<size_t> > > mGrid_, */
    /*std::map<KeyFrame*,int> mConnectedKeyFrameWeights = std::map<KeyFrame*,int>(), */ /*std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames = std::vector<KeyFrame*>(), */std::vector<int> mvOrderedWeights_, std::map<long unsigned int, int> mBackupConnectedKeyFrameIdWeights_, bool mbFirstConnection_,  /*KeyFrame* mpParent = nullptr, */ /*std::set<KeyFrame*> mspChildrens = std::set<KeyFrame*>(), */ /*std::set<KeyFrame*> mspLoopEdges = std::set<KeyFrame*>(), */ /*std::set<KeyFrame*> mspMergeEdges = std::set<KeyFrame*>(), */long long int mBackupParentId_, std::vector<long unsigned int> mvBackupChildrensId_, std::vector<long unsigned int> mvBackupLoopEdgesId_, std::vector<long unsigned int> mvBackupMergeEdgesId_, bool mbNotErase_, bool mbToBeErased_, bool mbBad_, float mHalfBaseline_, /*Map* mpMap = nullptr,*/ long long int mBackupPrevKFId_, long long int mBackupNextKFId_, /*IMU::Preintegrated* mBackupImuPreintegrated = nullptr, */unsigned int mnBackupIdCamera_, unsigned int mnBackupIdCamera2_, /*Eigen::Matrix3f mK__,*/ unsigned int mnLastModule_ /*GeometricCamera* mpCamera = nullptr, */ /*GeometricCamera* mpCamera2 = nullptr, *//*std::vector<int> mvLeftToRightMatch_, std::vector<int> mvRightToLeftMatch_,*/ /*const std::vector<cv::KeyPoint> mvKeysRight_,*/ /*const int NLeft_,*/ /*const int NRight_,*/ /*std::vector< std::vector <std::vector<size_t> > > mGridRight_*/);


    // Pose functions
    void SetPose(const Sophus::SE3f &Tcw);
    void SetVelocity(const Eigen::Vector3f &Vw_);

    Sophus::SE3f GetPose();

    Sophus::SE3f GetPoseInverse();
    Eigen::Vector3f GetCameraCenter();

    Eigen::Vector3f GetImuPosition();
    Eigen::Matrix3f GetImuRotation();
    Sophus::SE3f GetImuPose();
    Eigen::Matrix3f GetRotation();
    Eigen::Vector3f GetTranslation();
    Eigen::Vector3f GetVelocity();
    bool isVelocitySet();

    // Bag of Words Representation
    void ComputeBoW();

    // Covisibility graph functions
    void AddConnection(KeyFrame* pKF, const int &weight);
    void EraseConnection(KeyFrame* pKF);

    void UpdateConnections(bool upParent=true);
    void UpdateBestCovisibles();
    std::map<long unsigned int, int> GetBackupConnectedKeyFrameIdWeights();
    std::set<KeyFrame *> GetConnectedKeyFrames();
    std::vector<KeyFrame* > GetVectorCovisibleKeyFrames();
    std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int &N);
    std::vector<KeyFrame*> GetCovisiblesByWeight(const int &w);
    int GetWeight(KeyFrame* pKF);

    // Spanning tree functions
    void AddChild(KeyFrame* pKF);
    void EraseChild(KeyFrame* pKF);
    void ChangeParent(KeyFrame* pKF);
    std::set<KeyFrame*> GetChilds();
    KeyFrame* GetParent();
    bool hasChild(KeyFrame* pKF);
    void SetFirstConnection(bool bFirst);

    // Loop Edges
    void AddLoopEdge(KeyFrame* pKF);
    std::set<KeyFrame*> GetLoopEdges();

    // Merge Edges
    void AddMergeEdge(KeyFrame* pKF);
    set<KeyFrame*> GetMergeEdges();

    // MapPoint observation functions
    int GetNumberMPs();
    void AddMapPoint(MapPoint* pMP, const size_t &idx);
    void EraseMapPointMatch(const int &idx);
    void EraseMapPointMatch(MapPoint* pMP);
    void ReplaceMapPointMatch(const int &idx, MapPoint* pMP);
    std::set<MapPoint*> GetMapPoints();
    std::vector<MapPoint*> GetMapPointMatches();
    int TrackedMapPoints(const int &minObs);
    MapPoint* GetMapPoint(const size_t &idx);

    // KeyPoint functions
    std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const bool bRight = false) const;
    bool UnprojectStereo(int i, Eigen::Vector3f &x3D);

    // Image
    bool IsInImage(const float &x, const float &y) const;

    // Enable/Disable bad flag changes
    void SetNotErase();
    void SetErase();

    // Set/check bad flag
    void SetBadFlag();
    bool isBad();

    inline bool GetNotErase()
    {
      unique_lock<mutex> lock(mMutexConnections);
      return mbNotErase;
    }

    inline bool GetToBeErased()
    {
      unique_lock<mutex> lock(mMutexConnections);
      return mbToBeErased;
    }

    inline std::vector<std::vector<std::vector<size_t>>> GetMGrid() {
      return mGrid;
    }

    // Compute Scene Depth (q=2 median). Used in monocular.
    float ComputeSceneMedianDepth(const int q);

    static bool weightComp( int a, int b){
        return a>b;
    }

    static bool lId(KeyFrame* pKF1, KeyFrame* pKF2){
        return pKF1->mnId<pKF2->mnId;
    }
    inline bool GetFirstConnection()
    {
      return mbFirstConnection;
    }

    Map* GetMap();
    void UpdateMap(Map* pMap);

    void SetNewBias(const IMU::Bias &b);
    Eigen::Vector3f GetGyroBias();

    Eigen::Vector3f GetAccBias();

    IMU::Bias GetImuBias();

    bool ProjectPointDistort(MapPoint* pMP, cv::Point2f &kp, float &u, float &v);
    bool ProjectPointUnDistort(MapPoint* pMP, cv::Point2f &kp, float &u, float &v);

    void PreSave(set<KeyFrame*>& spKF,set<MapPoint*>& spMP, set<GeometricCamera*>& spCam);
    void PostLoad(map<long unsigned int, KeyFrame*>& mpKFid, map<std::string, MapPoint*>& mpMPid, map<unsigned int, GeometricCamera*>& mpCamId, bool* bUnprocessed, std::set<unsigned long int> mspUnprocKFids=std::set<unsigned long int>());


    void SetORBVocabulary(ORBVocabulary* pORBVoc);
    void SetKeyFrameDatabase(KeyFrameDatabase* pKFDB);

    unsigned int GetLastModule();
    void SetLastModule(unsigned int mnId);

    bool bImu;

    void attachObserver(std::shared_ptr<Observer> observer) {
      observer_ = observer;
    }

    // The following variables are accesed from only 1 thread or never change (no mutex needed).
public:

    static long unsigned int nNextId;
    long unsigned int mnId;
    const long unsigned int mnFrameId;

    const double mTimeStamp;

    // Grid (to speed up feature matching)
    const int mnGridCols;
    const int mnGridRows;
    const float mfGridElementWidthInv;
    const float mfGridElementHeightInv;

    // Variables used by the tracking
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnFuseTargetForKF;

    // Variables used by the local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnBAFixedForKF;

    //Number of optimizations by BA(amount of iterations in BA)
    long unsigned int mnNumberOfOpt;

    // Variables used by the keyframe database
    long unsigned int mnLoopQuery;
    int mnLoopWords;
    float mLoopScore;
    long unsigned int mnRelocQuery;
    int mnRelocWords;
    float mRelocScore;
    long unsigned int mnMergeQuery;
    int mnMergeWords;
    float mMergeScore;
    long unsigned int mnPlaceRecognitionQuery;
    int mnPlaceRecognitionWords;
    float mPlaceRecognitionScore;

    bool mbCurrentPlaceRecognition;


    // Variables used by loop closing
    Sophus::SE3f mTcwGBA;
    Sophus::SE3f mTcwBefGBA;
    Eigen::Vector3f mVwbGBA;
    Eigen::Vector3f mVwbBefGBA;
    IMU::Bias mBiasGBA;
    long unsigned int mnBAGlobalForKF;

    // Variables used by merging
    Sophus::SE3f mTcwMerge;
    Sophus::SE3f mTcwBefMerge;
    Sophus::SE3f mTwcBefMerge;
    Eigen::Vector3f mVwbMerge;
    Eigen::Vector3f mVwbBefMerge;
    IMU::Bias mBiasMerge;
    long unsigned int mnMergeCorrectedForKF;
    long unsigned int mnMergeForKF;
    float mfScaleMerge;
    long unsigned int mnBALocalForMerge;

    float mfScale;

    // Calibration parameters
    const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;
    cv::Mat mDistCoef;

    // Number of KeyPoints
    const int N;

    // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    const std::vector<cv::KeyPoint> mvKeys;
    const std::vector<cv::KeyPoint> mvKeysUn;
    const std::vector<float> mvuRight; // negative value for monocular points
    const std::vector<float> mvDepth; // negative value for monocular points
    const cv::Mat mDescriptors;

    //BoW
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // Pose relative to parent (this is computed when bad flag is activated)
    Sophus::SE3f mTcp;

    // Scale
    const int mnScaleLevels;
    const float mfScaleFactor;
    const float mfLogScaleFactor;
    const std::vector<float> mvScaleFactors;
    const std::vector<float> mvLevelSigma2;
    const std::vector<float> mvInvLevelSigma2;

    // Image bounds and calibration
    const int mnMinX;
    const int mnMinY;
    const int mnMaxX;
    const int mnMaxY;

    // Preintegrated IMU measurements from previous keyframe
    KeyFrame* mPrevKF;
    KeyFrame* mNextKF;

    IMU::Preintegrated* mpImuPreintegrated;
    IMU::Calib mImuCalib;

    unsigned int mnOriginMapId;

    string mNameFile;

    int mnDataset;

    std::vector <KeyFrame*> mvpLoopCandKFs;
    std::vector <KeyFrame*> mvpMergeCandKFs;

    std::vector <unsigned long int> mvLoopCandKFIds;
    std::vector <unsigned long int> mvMergeCandKFIds;
    //bool mbHasHessian;
    //cv::Mat mHessianPose;

    // The following variables need to be accessed trough a mutex to be thread safe.
protected:
    // sophus poses
    Sophus::SE3<float> mTcw;
    Eigen::Matrix3f mRcw;
    Sophus::SE3<float> mTwc;
    Eigen::Matrix3f mRwc;

    // IMU position
    Eigen::Vector3f mOwb;
    // Velocity (Only used for inertial SLAM)
    Eigen::Vector3f mVw;
    bool mbHasVelocity;

    //Transformation matrix between cameras in stereo fisheye
    Sophus::SE3<float> mTlr;
    Sophus::SE3<float> mTrl;

    // Imu bias
    IMU::Bias mImuBias;

    // MapPoints associated to keypoints
    std::vector<MapPoint*> mvpMapPoints;
    // For save relation without pointer, this is necessary for save/load function
    std::vector<std::string> mvBackupMapPointsId;

    // BoW
    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBvocabulary;

    // Grid over the image to speed up feature matching
    std::vector< std::vector <std::vector<size_t> > > mGrid;

    std::map<KeyFrame*,int> mConnectedKeyFrameWeights;
    std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;
    std::vector<int> mvOrderedWeights;
    // For save relation without pointer, this is necessary for save/load function
    std::map<long unsigned int, int> mBackupConnectedKeyFrameIdWeights;

    // Spanning Tree and Loop Edges
    bool mbFirstConnection;
    KeyFrame* mpParent;
    std::set<KeyFrame*> mspChildrens;
    std::set<KeyFrame*> mspLoopEdges;
    std::set<KeyFrame*> mspMergeEdges;
    // For save relation without pointer, this is necessary for save/load function
    long long int mBackupParentId;
    std::vector<long unsigned int> mvBackupChildrensId;
    std::vector<long unsigned int> mvBackupLoopEdgesId;
    std::vector<long unsigned int> mvBackupMergeEdgesId;

    // Bad flags
    bool mbNotErase;
    bool mbToBeErased;
    bool mbBad;    

    float mHalfBaseline; // Only for visualization

    Map* mpMap;

    // Backup variables for inertial
    long long int mBackupPrevKFId;
    long long int mBackupNextKFId;
    IMU::Preintegrated mBackupImuPreintegrated;

    // Backup for Cameras
    unsigned int mnBackupIdCamera, mnBackupIdCamera2;

    // Calibration
    Eigen::Matrix3f mK_;
      
    // Module where object was created/last update
    unsigned int mnLastModule;

    std::shared_ptr<Observer> observer_;

    // Mutex
    std::mutex mMutexPose; // for pose, velocity and biases
    std::mutex mMutexConnections;
    std::mutex mMutexFeatures;
    std::mutex mMutexMap;
    std::mutex mMutexModule;
public:
    GeometricCamera* mpCamera, *mpCamera2;

    //Indexes of stereo observations correspondences
    std::vector<int> mvLeftToRightMatch, mvRightToLeftMatch;

    Sophus::SE3f GetRelativePoseTrl();
    Sophus::SE3f GetRelativePoseTlr();

    //KeyPoints in the right image (for stereo fisheye, coordinates are needed)
    const std::vector<cv::KeyPoint> mvKeysRight;

    const int NLeft, NRight;

    std::vector< std::vector <std::vector<size_t> > > mGridRight;

    Sophus::SE3<float> GetRightPose();
    Sophus::SE3<float> GetRightPoseInverse();

    Eigen::Vector3f GetRightCameraCenter();
    Eigen::Matrix<float,3,3> GetRightRotation();
    Eigen::Vector3f GetRightTranslation();

    inline Eigen::Matrix3f GetCalibrationMatrix() {
      return mK_;
    }

    void PrintPointDistribution(){
        int left = 0, right = 0;
        int Nlim = (NLeft != -1) ? NLeft : N;
        for(int i = 0; i < N; i++){
            if(mvpMapPoints[i]){
                if(i < Nlim) left++;
                else right++;
            }
        }
        cout << "Point distribution in KeyFrame: left-> " << left << " --- right-> " << right << endl;
    }









    // Protected setters
    inline void SetmTcw(Sophus::SE3<float> tcw) {
      // mutex
      mTcw = tcw;
    }

    inline void SetmRcw(Eigen::Matrix3f rcw) {
      // mutex
      mRcw = rcw;
    }

    inline void SetmTwc(Sophus::SE3<float> twc) {
      // mutex
      mTwc = twc;
    }

    inline void SetmRwc(Eigen::Matrix3f rwc) {
      // mutex
      mRwc = rwc;
    }

    inline void SetmOwb(Eigen::Vector3f owb) {
      // mutex
      mOwb = owb;
    }

    inline void SetmVw(Eigen::Vector3f vw) {
      // mutex
      mVw = vw;
    }


    inline void SetmbHasVelocity(bool hasVelocity) {
      // mutex
      mbHasVelocity = hasVelocity;
    }

    inline void SetmTlr(Sophus::SE3<float> tlr) {
      // mutex
      mTlr = tlr;
    }


    inline void SetmTrl(Sophus::SE3<float> trl) {
      // mutex
      mTrl = trl;
    }

    inline void SetMvpMapPoints(std::vector<MapPoint*> mapPoints) {
      // mutex
      mvpMapPoints = mapPoints;
    }
    
    //inline void SetMvBackupMapPointsId(std::vector<long long int> backupMapPointsId) {
      // mutex
    //  mvBackupMapPointsId = backupMapPointsId;
    //}


    inline void SetMpKeyFrameDB(KeyFrameDatabase* keyframeDb) {
      // mutex
      mpKeyFrameDB = keyframeDb;
    }


    inline void SetMpORBvocabulary(ORBVocabulary* orbVocab) {
      // mutex
      mpORBvocabulary = orbVocab;
    }


    inline void SetmGrid(std::vector< std::vector <std::vector<size_t> > > grid) {
      // mutex
      mGrid = grid;
    }


    inline void SetmConnectedKeyFrameWeights(std::map<KeyFrame*,int> connectedKeyFrameWeights) {
      // mutex
      mConnectedKeyFrameWeights = connectedKeyFrameWeights;
    }


    inline void SetMvOrderedConnectedKeyFrames(std::vector<KeyFrame*> orderedConnectedKeyFrames) {
      // mutex
      mvpOrderedConnectedKeyFrames = orderedConnectedKeyFrames;
    }


    inline void SetMvOrderedWeights(std::vector<int> orderedWeights) {
      // mutex
      mvOrderedWeights = orderedWeights;
    }

    inline void SetmBackupConnectedKeyFrameIdWeights(std::map<long unsigned int, int> backupConnectedKeyFrameIdWeights) {
      // mutex
      mBackupConnectedKeyFrameIdWeights = backupConnectedKeyFrameIdWeights;
    }


    inline void SetMbFirstConnection(bool firstConnection) {
      // mutex
      mbFirstConnection = firstConnection;
    }

    inline void SetMpParent(KeyFrame* parent) {
      // mutex
      mpParent = parent;
    }

    inline void SetMspChildrens(std::set<KeyFrame*> childrens) {
      // mutex
      mspChildrens = childrens;
    }

    inline void SetMspLoopEdges(std::set<KeyFrame*> loopEdges) {
      // mutex
      mspLoopEdges = loopEdges;
    }

    inline void SetMspMergeEdges(std::set<KeyFrame*> mergeEdges) {
      // mutex
      mspMergeEdges = mergeEdges;
    }

    inline void SetmBackupParentId(long long int backupParentId) {
      // mutex
      mBackupParentId = backupParentId;
    }

    inline void SetMvBackupChildrensId(std::vector<long unsigned int> backupChildrensId) {
      // mutex
      mvBackupChildrensId = backupChildrensId;
    }

    inline void SetMvBackupLoopEdgesId(std::vector<long unsigned int> backupLoopEdgesId) {
      // mutex
      mvBackupLoopEdgesId = backupLoopEdgesId;
    }

    inline void SetMvBackupParentId(std::vector<long unsigned int> backupMergeEdgesId) {
      // mutex
      mvBackupMergeEdgesId = backupMergeEdgesId;
    }

    inline void SetMbNotErase(bool notErase) {
      // mutex
      mbNotErase = notErase;
    }

    inline void SetMbToBeErased(bool toBeErased) {
      // mutex
      mbToBeErased = toBeErased;
    }

    inline void SetMbBad(bool bad) {
      // mutex
      mbBad = bad;
    }

};

} //namespace ORB_SLAM

#endif // KEYFRAME_H
