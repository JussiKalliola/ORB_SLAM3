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

#include "KeyFrame.h"
#include "Converter.h"
#include "ImuTypes.h"
#include<mutex>

namespace ORB_SLAM3
{

long unsigned int KeyFrame::nNextId=0;

KeyFrame::KeyFrame():
        mnFrameId(0),  mTimeStamp(0), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
        mfGridElementWidthInv(0), mfGridElementHeightInv(0),
        mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0), mnBALocalForMerge(0),
        mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnMergeQuery(0), mnMergeWords(0), mnBAGlobalForKF(0),
        fx(0), fy(0), cx(0), cy(0), invfx(0), invfy(0), mnPlaceRecognitionQuery(0), mnPlaceRecognitionWords(0), mPlaceRecognitionScore(0),
        mbf(0), mb(0), mThDepth(0), N(0), mvKeys(static_cast<vector<cv::KeyPoint> >(NULL)), mvKeysUn(static_cast<vector<cv::KeyPoint> >(NULL)),
        mvuRight(static_cast<vector<float> >(NULL)), mvDepth(static_cast<vector<float> >(NULL)), mnScaleLevels(0), mfScaleFactor(0),
        mfLogScaleFactor(0), mvScaleFactors(0), mvLevelSigma2(0), mvInvLevelSigma2(0), mnMinX(0), mnMinY(0), mnMaxX(0),
        mnMaxY(0), mPrevKF(static_cast<KeyFrame*>(NULL)), mNextKF(static_cast<KeyFrame*>(NULL)), mbFirstConnection(true), mpParent(static_cast<KeyFrame*>(NULL)), mbNotErase(false),
        mbToBeErased(false), mbBad(false), mHalfBaseline(0), mbCurrentPlaceRecognition(false), mnMergeCorrectedForKF(0),
        NLeft(0),NRight(0), mnNumberOfOpt(0), mbHasVelocity(false)
{
    //notifyObserverKeyframeAdded(this);
}

KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB):
    bImu(pMap->isImuInitialized()), mnFrameId(F.mnId),  mTimeStamp(F.mTimeStamp), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
    mfGridElementWidthInv(F.mfGridElementWidthInv), mfGridElementHeightInv(F.mfGridElementHeightInv),
    mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0), mnBALocalForMerge(0),
    mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0), mnPlaceRecognitionQuery(0), mnPlaceRecognitionWords(0), mPlaceRecognitionScore(0),
    fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), invfx(F.invfx), invfy(F.invfy),
    mbf(F.mbf), mb(F.mb), mThDepth(F.mThDepth), N(F.N), mvKeys(F.mvKeys), mvKeysUn(F.mvKeysUn),
    mvuRight(F.mvuRight), mvDepth(F.mvDepth), mDescriptors(F.mDescriptors.clone()),
    mBowVec(F.mBowVec), mFeatVec(F.mFeatVec), mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor),
    mfLogScaleFactor(F.mfLogScaleFactor), mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
    mvInvLevelSigma2(F.mvInvLevelSigma2), mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX),
    mnMaxY(F.mnMaxY), mK_(F.mK_), mPrevKF(static_cast<KeyFrame*>(NULL)), mNextKF(static_cast<KeyFrame*>(NULL)), mpImuPreintegrated(F.mpImuPreintegrated),
    mImuCalib(F.mImuCalib), mvpMapPoints(F.mvpMapPoints), mpKeyFrameDB(pKFDB),
    mpORBvocabulary(F.mpORBvocabulary), mbFirstConnection(true), mpParent(static_cast<KeyFrame*>(NULL)), mDistCoef(F.mDistCoef), mbNotErase(false), mnDataset(F.mnDataset),
    mbToBeErased(false), mbBad(false), mHalfBaseline(F.mb/2), mpMap(pMap), mbCurrentPlaceRecognition(false), mNameFile(F.mNameFile), mnMergeCorrectedForKF(0),
    mpCamera(F.mpCamera), mpCamera2(F.mpCamera2),
    mvLeftToRightMatch(F.mvLeftToRightMatch),mvRightToLeftMatch(F.mvRightToLeftMatch), mTlr(F.GetRelativePoseTlr()),
    mvKeysRight(F.mvKeysRight), NLeft(F.Nleft), NRight(F.Nright), mTrl(F.GetRelativePoseTrl()), mnNumberOfOpt(0), mbHasVelocity(false)
{
    mnId=nNextId++;
    
    attachObserver(mpMap->GetObserver());

    mGrid.resize(mnGridCols);
    if(F.Nleft != -1)  mGridRight.resize(mnGridCols);
    for(int i=0; i<mnGridCols;i++)
    {
        mGrid[i].resize(mnGridRows);
        if(F.Nleft != -1) mGridRight[i].resize(mnGridRows);
        for(int j=0; j<mnGridRows; j++){
            mGrid[i][j] = F.mGrid[i][j];
            if(F.Nleft != -1){
                mGridRight[i][j] = F.mGridRight[i][j];
            }
        }
    }



    if(!F.HasVelocity()) {
        mVw.setZero();
        mbHasVelocity = false;
    }
    else
    {
        mVw = F.GetVelocity();
        mbHasVelocity = true;
    }

    mImuBias = F.mImuBias;
    SetPose(F.GetPose());

    mnOriginMapId = pMap->GetId();
    
    //notifyObserverKeyframeAdded(this);
}

// Constructor for ROS message
KeyFrame::KeyFrame(bool bImu = false, 
    long unsigned int nNextId = 0, 
    long unsigned int mnId = 0, 
    const long unsigned int mnFrameId = 0, 
    const double mTimeStamp = 0.0, 
    const int mnGridCols = 0, 
    const int mnGridRows = 0, 
    const float mfGridElementWidthInv = 0.0, 
    const float mfGridElementHeightInv = 0.0, 
    long unsigned int mnTrackReferenceForFrame = 0,
    long unsigned int mnFuseTargetForKF = 0, 
    long unsigned int mnBALocalForKF = 0, 
    long unsigned int mnBAFixedForKF = 0, 
    long unsigned int mnNumberOfOpt = 0, 
    long unsigned int mnLoopQuery = 0, 
    int mnLoopWords = 0, 
    float mLoopScore = 0.0, 
    long unsigned int mnRelocQuery = 0, 
    int mnRelocWords = 0, 
    float mRelocScore = 0.0, 
    long unsigned int mnMergeQuery = 0, 
    int mnMergeWords = 0, 
    float mMergeScore = 0.0, 
    long unsigned int mnPlaceRecognitionQuery = 0, 
    int mnPlaceRecognitionWords = 0, 
    float mPlaceRecognitionScore = -1.0, 
    bool mbCurrentPlaceRecognition = false, 
    Sophus::SE3f mTcwGBA = Sophus::SE3f(), 
    Sophus::SE3f mTcwBefGBA = Sophus::SE3f(), 
    Eigen::Vector3f mVwbGBA = Eigen::Vector3f(), 
    Eigen::Vector3f mVwbBefGBA = Eigen::Vector3f(), 
    IMU::Bias mBiasGBA = IMU::Bias(), 
    long unsigned int mnBAGlobalForKF = 0, 
    Sophus::SE3f mTcwMerge = Sophus::SE3f(), 
    Sophus::SE3f mTcwBefMerge = Sophus::SE3f(), 
    Sophus::SE3f mTwcBefMerge = Sophus::SE3f(), 
    Eigen::Vector3f mVwbMerge = Eigen::Vector3f(), 
    Eigen::Vector3f mVwbBefMerge = Eigen::Vector3f(), 
    IMU::Bias mBiasMerge = IMU::Bias(), 
    long unsigned int mnMergeCorrectedForKF = 0, 
    long unsigned int mnMergeForKF = 0, 
    float mfScaleMerge = 0.0, 
    long unsigned int mnBALocalForMerge = 0, 
    float mfScale = 0.0, 
    const float fx = 0.0, 
    const float fy = 0.0, 
    const float cx = 0.0, 
    const float cy = 0.0, 
    const float invfx = 0.0, 
    const float invfy = 0.0, 
    const float mbf = 0.0, 
    const float mb = 0.0, 
    const float mThDepth = 0.0, 
    cv::Mat mDistCoef = cv::Mat(), 
    const int N = 0, 
    const std::vector<cv::KeyPoint> mvKeys = std::vector<cv::KeyPoint>(), 
    const std::vector<cv::KeyPoint> mvKeysUn = std::vector<cv::KeyPoint>(), 
    const std::vector<float> mvuRight = std::vector<float>(), 
    const std::vector<float> mvDepth = std::vector<float>(), 
    const cv::Mat mDescriptors = cv::Mat(), 
    DBoW2::BowVector mBowVec = DBoW2::BowVector(), 
    DBoW2::FeatureVector mFeatVec = DBoW2::FeatureVector(), 
    Sophus::SE3f mTcp = Sophus::SE3f(), 
    const int mnScaleLevels = -1, 
    const float mfScaleFactor = -1.0, 
    const float mfLogScaleFactor = -1.0, 
    const std::vector<float> mvScaleFactors = std::vector<float>(), 
    const std::vector<float> mvLevelSigma2 = std::vector<float>(), 
    const std::vector<float> mvInvLevelSigma2 = std::vector<float>(), 
    const int mnMinX = -1, 
    const int mnMinY = -1, 
    const int mnMaxX = -1, 
    const int mnMaxY = -1, 
    //KeyFrame* mPrevKF = nullptr, 
    //KeyFrame* mNextKF = nullptr, 
    //IMU::Preintegrated* mpImuPreintegrated = nullptr, 
    IMU::Calib mImuCalib = IMU::Calib(), 
    unsigned int mnOriginMapId = -1, 
    string mNameFile = "", 
    int mnDataset = 0, 
    //std::vector <KeyFrame*> mvpLoopCandKFs = std::vector <KeyFrame*>(), 
    //std::vector <KeyFrame*> mvpMergeCandKFs = std::vector <KeyFrame*>(), 
    Sophus::SE3<float> mTcw = Sophus::SE3<float>(), 
    Eigen::Matrix3f mRcw = Eigen::Matrix3f(), 
    Sophus::SE3<float> mTwc = Sophus::SE3<float>(), 
    Eigen::Matrix3f mRwc = Eigen::Matrix3f(), 
    Eigen::Vector3f mOwb = Eigen::Vector3f(), 
    Eigen::Vector3f mVw = Eigen::Vector3f(), 
    bool mbHasVelocity = false, 
    Sophus::SE3<float> mTlr = Sophus::SE3<float>(), 
    Sophus::SE3<float> mTrl = Sophus::SE3<float>(), 
    IMU::Bias mImuBias = IMU::Bias(), 
    //std::vector<MapPoint*> mvpMapPoints = std::vector<MapPoint*>(), 
    std::vector<std::string> mvBackupMapPointsId = std::vector<std::string>(), 
    //KeyFrameDatabase* mpKeyFrameDB = nullptr, 
    //ORBVocabulary* mpORBvocabulary = nullptr, 
    std::vector< std::vector <std::vector<size_t> > > mGrid = std::vector< std::vector <std::vector<size_t> > >(), 
    //std::map<KeyFrame*,int> mConnectedKeyFrameWeights = std::map<KeyFrame*,int>(), 
    //std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames = std::vector<KeyFrame*>(), 
    std::vector<int> mvOrderedWeights = std::vector<int>(), 
    std::map<long unsigned int, int> mBackupConnectedKeyFrameIdWeights = std::map<long unsigned int, int>(), 
    bool mbFirstConnection = false, 
    //KeyFrame* mpParent = nullptr, 
    //std::set<KeyFrame*> mspChildrens = std::set<KeyFrame*>(), 
    //std::set<KeyFrame*> mspLoopEdges = std::set<KeyFrame*>(), 
    //std::set<KeyFrame*> mspMergeEdges = std::set<KeyFrame*>(), 
    long long int mBackupParentId = -1, 
    std::vector<long unsigned int> mvBackupChildrensId = std::vector<long unsigned int>(), 
    std::vector<long unsigned int> mvBackupLoopEdgesId = std::vector<long unsigned int>(), 
    std::vector<long unsigned int> mvBackupMergeEdgesId = std::vector<long unsigned int>(), 
    bool mbNotErase = false, 
    bool mbToBeErased = false, 
    bool mbBad = false, 
    float mHalfBaseline = -0.0, 
    /*Map* mpMap = nullptr,*/ 
    long long int mBackupPrevKFId = -1, 
    long long int mBackupNextKFId = -1, 
    //IMU::Preintegrated* mBackupImuPreintegrated = nullptr, 
    unsigned int mnBackupIdCamera = -1, 
    unsigned int mnBackupIdCamera2 = -1, 
    Eigen::Matrix3f mK_ = Eigen::Matrix3f(), 
    //GeometricCamera* mpCamera = nullptr, 
    //GeometricCamera* mpCamera2 = nullptr, 
    std::vector<int> mvLeftToRightMatch = std::vector<int>(), 
    std::vector<int> mvRightToLeftMatch = std::vector<int>(), 
    const std::vector<cv::KeyPoint> mvKeysRight = std::vector<cv::KeyPoint>(), 
    const int NLeft = -1, 
    const int NRight = -1, 
    std::vector< std::vector <std::vector<size_t> > > mGridRight = std::vector< std::vector <std::vector<size_t> > >()):
bImu(bImu), mnId(mnId), mnFrameId(mnFrameId), mTimeStamp(mTimeStamp), mnGridCols(mnGridCols), mnGridRows(mnGridRows), mfGridElementWidthInv(mfGridElementWidthInv), mfGridElementHeightInv(mfGridElementHeightInv), mnTrackReferenceForFrame(mnTrackReferenceForFrame), mnFuseTargetForKF(mnFuseTargetForKF), mnBALocalForKF(mnBALocalForKF), mnBAFixedForKF(mnBAFixedForKF), mnNumberOfOpt(mnNumberOfOpt), mnLoopQuery(mnLoopQuery), mnLoopWords(mnLoopWords), mLoopScore(mLoopScore), mnRelocQuery(mnRelocQuery), mnRelocWords(mnRelocWords), mRelocScore(mRelocScore), mnMergeQuery(mnMergeQuery), mnMergeWords(mnMergeWords), mMergeScore(mMergeScore), mnPlaceRecognitionQuery(mnPlaceRecognitionQuery), mnPlaceRecognitionWords(mnPlaceRecognitionWords), mPlaceRecognitionScore(mPlaceRecognitionScore), mbCurrentPlaceRecognition(mbCurrentPlaceRecognition), mTcwGBA(mTcwGBA), mTcwBefGBA(mTcwBefGBA), mVwbGBA(mVwbGBA), mVwbBefGBA(mVwbBefGBA), mBiasGBA(mBiasGBA), mnBAGlobalForKF(mnBAGlobalForKF), mTcwMerge(mTcwMerge), mTcwBefMerge(mTcwBefMerge), mTwcBefMerge(mTwcBefMerge), mVwbMerge(mVwbMerge), mVwbBefMerge(mVwbBefMerge), mBiasMerge(mBiasMerge), mnMergeCorrectedForKF(mnMergeCorrectedForKF), mnMergeForKF(mnMergeForKF), mfScaleMerge(mfScaleMerge), mnBALocalForMerge(mnBALocalForMerge), mfScale(mfScale), fx(fx), fy(fy), cx(cx), cy(cy), invfx(invfx), invfy(invfy), mbf(mbf), mb(mb), mThDepth(mThDepth), mDistCoef(mDistCoef), N(N), mvKeys(mvKeys), mvKeysUn(mvKeysUn), mvuRight(mvuRight), mvDepth(mvDepth), mDescriptors(mDescriptors), mBowVec(mBowVec), mFeatVec(mFeatVec), mTcp(mTcp), mnScaleLevels(mnScaleLevels), mfScaleFactor(mfScaleFactor), mfLogScaleFactor(mfLogScaleFactor), mvScaleFactors(mvScaleFactors), mvLevelSigma2(mvLevelSigma2), mvInvLevelSigma2(mvInvLevelSigma2), mnMinX(mnMinX), mnMinY(mnMinY), mnMaxX(mnMaxX), mnMaxY(mnMaxY), mPrevKF(static_cast<KeyFrame*>(NULL)), mNextKF(static_cast<KeyFrame*>(NULL)), 
  //mpImuPreintegrated(mpImuPreintegrated), 
  mImuCalib(mImuCalib), mnOriginMapId(mnOriginMapId), mNameFile(mNameFile), mnDataset(mnDataset), /*mvpLoopCandKFs(mvpLoopCandKFs), mvpMergeCandKFs(mvpMergeCandKFs),*/ mTcw(mTcw), mRcw(mRcw), mTwc(mTwc), mRwc(mRwc), mOwb(mOwb), mVw(mVw), mbHasVelocity(mbHasVelocity), mTlr(mTlr), mTrl(mTrl), mImuBias(mImuBias), /*mvpMapPoints(mvpMapPoints),*/ 
  mvBackupMapPointsId(mvBackupMapPointsId), /*mpKeyFrameDB(mpKeyFrameDB), mpORBvocabulary(mpORBvocabulary),*/ mGrid(mGrid), /*mConnectedKeyFrameWeights(mConnectedKeyFrameWeights), mvpOrderedConnectedKeyFrames(mvpOrderedConnectedKeyFrames),*/ mvOrderedWeights(mvOrderedWeights), mBackupConnectedKeyFrameIdWeights(mBackupConnectedKeyFrameIdWeights), mbFirstConnection(mbFirstConnection), mpParent(static_cast<KeyFrame*>(NULL)), /*mspChildrens(mspChildrens), mspLoopEdges(mspLoopEdges), mspMergeEdges(mspMergeEdges),*/ mBackupParentId(mBackupParentId), mvBackupChildrensId(mvBackupChildrensId), mvBackupLoopEdgesId(mvBackupLoopEdgesId), mvBackupMergeEdgesId(mvBackupMergeEdgesId), mbNotErase(mbNotErase), mbToBeErased(mbToBeErased), mbBad(mbBad), mHalfBaseline(mHalfBaseline), /*mpMap(mpMap),*/ mBackupPrevKFId(mBackupPrevKFId), mBackupNextKFId(mBackupNextKFId),
  //mBackupImuPreintegrated(mBackupImuPreintegrated), 
  mnBackupIdCamera(mnBackupIdCamera), mnBackupIdCamera2(mnBackupIdCamera2), mK_(mK_), /*mpCamera(mpCamera),*/ /*mpCamera2(mpCamera2),*/ mvLeftToRightMatch(mvLeftToRightMatch), mvRightToLeftMatch(mvRightToLeftMatch), mvKeysRight(mvKeysRight), NLeft(NLeft), NRight(NRight), mGridRight(mGridRight)
{

    //mnId=nNextId++;
    
    //if(!F.HasVelocity()) {
    //    mVw.setZero();
    //    mbHasVelocity = false;
    //}
    //else
    //{
    //    mVw = F.GetVelocity();
    //    mbHasVelocity = true;
    //}

    //mImuBias = F.mImuBias;
    //SetPose(F.GetPose());

    //mnOriginMapId = pMap->GetId();
    //std::mutex mMutexPose; // for pose, velocity and biases
    //std::mutex mMutexConnections;
    //std::mutex mMutexFeatures;
    //std::mutex mMutexMap;
}



void KeyFrame::ComputeBoW(bool fromRos)
{

    if(!fromRos) {
      notifyObserverKFAction(mnId, 15, true);
    }

    if(mBowVec.empty() || mFeatVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        // Feature vector associate features with nodes in the 4th level (from leaves up)
        // We assume the vocabulary tree has 6 levels, change the 4 otherwise
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}

void KeyFrame::SetPose(const Sophus::SE3f &Tcw, bool fromRos)
{
    unique_lock<mutex> lock(mMutexPose);

    if(!fromRos) {
      notifyObserverKFAction(mnId, 0, Tcw);
    }
    
    mTcw = Tcw;
    mRcw = mTcw.rotationMatrix();
    mTwc = mTcw.inverse();
    mRwc = mTwc.rotationMatrix();

    if (mImuCalib.mbIsSet) // TODO Use a flag instead of the OpenCV matrix
    {
        mOwb = mRwc * mImuCalib.mTcb.translation() + mTwc.translation();
    }
}

void KeyFrame::SetVelocity(const Eigen::Vector3f &Vw, bool fromRos)
{
    unique_lock<mutex> lock(mMutexPose);
    if(!fromRos) {
      notifyObserverKFAction(mnId, 1, Vw);
    }
    mVw = Vw;
    mbHasVelocity = true;
}

Sophus::SE3f KeyFrame::GetPose()
{
    unique_lock<mutex> lock(mMutexPose);
    return mTcw;
}

Sophus::SE3f KeyFrame::GetPoseInverse()
{
    unique_lock<mutex> lock(mMutexPose);
    return mTwc;
}

Eigen::Vector3f KeyFrame::GetCameraCenter(){
    unique_lock<mutex> lock(mMutexPose);
    //std::cout << "this is before returning translation vector" << std::endl;
    return mTwc.translation();
}

Eigen::Vector3f KeyFrame::GetImuPosition()
{
    unique_lock<mutex> lock(mMutexPose);
    return mOwb;
}

Eigen::Matrix3f KeyFrame::GetImuRotation()
{
    unique_lock<mutex> lock(mMutexPose);
    return (mTwc * mImuCalib.mTcb).rotationMatrix();
}

Sophus::SE3f KeyFrame::GetImuPose()
{
    unique_lock<mutex> lock(mMutexPose);
    return mTwc * mImuCalib.mTcb;
}

Eigen::Matrix3f KeyFrame::GetRotation(){
    unique_lock<mutex> lock(mMutexPose);
    return mRcw;
}

Eigen::Vector3f KeyFrame::GetTranslation()
{
    unique_lock<mutex> lock(mMutexPose);
    return mTcw.translation();
}

Eigen::Vector3f KeyFrame::GetVelocity()
{
    unique_lock<mutex> lock(mMutexPose);
    return mVw;
}

bool KeyFrame::isVelocitySet()
{
    unique_lock<mutex> lock(mMutexPose);
    return mbHasVelocity;
}

void KeyFrame::AddConnection(KeyFrame *pKF, const int &weight, bool fromRos)
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        
        if(!fromRos) {
          notifyObserverKFAction(mnId, 2, pKF->mnId, weight);
        }

        if(!mConnectedKeyFrameWeights.count(pKF))
            mConnectedKeyFrameWeights[pKF]=weight;
        else if(mConnectedKeyFrameWeights[pKF]!=weight)
            mConnectedKeyFrameWeights[pKF]=weight;
        else
            return;
    }

    UpdateBestCovisibles();
}

void KeyFrame::UpdateBestCovisibles(bool fromRos)
{
    unique_lock<mutex> lock(mMutexConnections);
    
    if(!fromRos) {
      notifyObserverKFAction(mnId, 16, true);
    }

    vector<pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(mConnectedKeyFrameWeights.size());
    for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
       vPairs.push_back(make_pair(mit->second,mit->first));

    sort(vPairs.begin(),vPairs.end());
    list<KeyFrame*> lKFs;
    list<int> lWs;
    for(size_t i=0, iend=vPairs.size(); i<iend;i++)
    {
        if(!vPairs[i].second->isBad())
        {
            lKFs.push_front(vPairs[i].second);
            lWs.push_front(vPairs[i].first);
        }
    }

    mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
    mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());
}

set<KeyFrame*> KeyFrame::GetConnectedKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    set<KeyFrame*> s;
    for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin();mit!=mConnectedKeyFrameWeights.end();mit++)
        s.insert(mit->first);
    return s;
}

vector<KeyFrame*> KeyFrame::GetVectorCovisibleKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    return mvpOrderedConnectedKeyFrames;
}

vector<KeyFrame*> KeyFrame::GetBestCovisibilityKeyFrames(const int &N)
{
    unique_lock<mutex> lock(mMutexConnections);
    if((int)mvpOrderedConnectedKeyFrames.size()<N)
        return mvpOrderedConnectedKeyFrames;
    else
        return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(),mvpOrderedConnectedKeyFrames.begin()+N);

}

vector<KeyFrame*> KeyFrame::GetCovisiblesByWeight(const int &w)
{
    unique_lock<mutex> lock(mMutexConnections);

    if(mvpOrderedConnectedKeyFrames.empty())
    {
        return vector<KeyFrame*>();
    }

    vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(),mvOrderedWeights.end(),w,KeyFrame::weightComp);

    if(it==mvOrderedWeights.end() && mvOrderedWeights.back() < w)
    {
        return vector<KeyFrame*>();
    }
    else
    {
        int n = it-mvOrderedWeights.begin();
        return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin()+n);
    }
}

int KeyFrame::GetWeight(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexConnections);
    if(mConnectedKeyFrameWeights.count(pKF))
        return mConnectedKeyFrameWeights[pKF];
    else
        return 0;
}

int KeyFrame::GetNumberMPs()
{
    unique_lock<mutex> lock(mMutexFeatures);
    int numberMPs = 0;
    for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
    {
        if(!mvpMapPoints[i])
            continue;
        numberMPs++;
    }
    return numberMPs;
}

void KeyFrame::AddMapPoint(MapPoint *pMP, const size_t &idx, bool fromRos)
{
    unique_lock<mutex> lock(mMutexFeatures);
    //mvpMapPoints.push_back(pMP);
    
    //if(!fromRos) {
    //  notifyObserverKFAction(mnId, 3, pMP->mnId, idx);
    //}

    mvpMapPoints[idx]=pMP;
}

void KeyFrame::EraseMapPointMatch(const int &idx, bool fromRos)
{
    unique_lock<mutex> lock(mMutexFeatures);
    
    if(!fromRos) {
      notifyObserverKFAction(mnId, 4, static_cast<unsigned long int>(idx));
    }

    mvpMapPoints[idx]=static_cast<MapPoint*>(NULL);
}

void KeyFrame::EraseMapPointMatch(MapPoint* pMP, bool fromRos)
{
    
    //if(!fromRos) {
    //  notifyObserverKFAction(mnId, 5, pMP->mnId);
    //}

    tuple<size_t,size_t> indexes = pMP->GetIndexInKeyFrame(this);
    size_t leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
    if(leftIndex != -1)
        mvpMapPoints[leftIndex]=static_cast<MapPoint*>(NULL);
    if(rightIndex != -1)
        mvpMapPoints[rightIndex]=static_cast<MapPoint*>(NULL);
}


void KeyFrame::ReplaceMapPointMatch(const int &idx, MapPoint* pMP, bool fromRos)
{
    
    //if(!fromRos) {
    //  notifyObserverKFAction(mnId, 6, pMP->mnId, idx);
    //}

    mvpMapPoints[idx]=pMP;
}

set<MapPoint*> KeyFrame::GetMapPoints()
{
    unique_lock<mutex> lock(mMutexFeatures);
    set<MapPoint*> s;
    for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
    {
        if(!mvpMapPoints[i])
            continue;
        MapPoint* pMP = mvpMapPoints[i];
        if(!pMP->isBad())
            s.insert(pMP);
    }
    return s;
}

int KeyFrame::TrackedMapPoints(const int &minObs)
{
    unique_lock<mutex> lock(mMutexFeatures);

    int nPoints=0;
    const bool bCheckObs = minObs>0;
    for(int i=0; i<N; i++)
    {
        MapPoint* pMP = mvpMapPoints[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                if(bCheckObs)
                {
                    if(mvpMapPoints[i]->Observations()>=minObs)
                        nPoints++;
                }
                else
                    nPoints++;
            }
        }
    }

    return nPoints;
}

vector<MapPoint*> KeyFrame::GetMapPointMatches()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints;
}

MapPoint* KeyFrame::GetMapPoint(const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints[idx];
}

void KeyFrame::UpdateConnections(bool upParent, bool fromRos)
{
    
    if(!fromRos) {
      notifyObserverKFAction(mnId, 17, true);
    }

    map<KeyFrame*,int> KFcounter;

    vector<MapPoint*> vpMP;

    {
        unique_lock<mutex> lockMPs(mMutexFeatures);
        vpMP = mvpMapPoints;
    }
    //For all map points in keyframe check in which other keyframes are they seen
    //Increase counter for those keyframes
    for(vector<MapPoint*>::iterator vit=vpMP.begin(), vend=vpMP.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;

        if(!pMP)
            continue;

        if(pMP->isBad())
            continue;

        map<KeyFrame*,tuple<int,int>> observations = pMP->GetObservations();

        for(map<KeyFrame*,tuple<int,int>>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            if(mit->first->mnId==mnId || mit->first->isBad() || mit->first->GetMap() != mpMap)
            {
              continue;
            }
            KFcounter[mit->first]++;

        }
    }


    // This should not happen
    if(KFcounter.empty())
        return;

    //If the counter is greater than threshold add connection
    //In case no keyframe counter is over threshold add the one with maximum counter
    int nmax=0;
    KeyFrame* pKFmax=NULL;
    int th = 15;
    
    vector<pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(KFcounter.size());
    if(!upParent)
        cout << "UPDATE_CONN: current KF " << mnId << endl;
    for(map<KeyFrame*,int>::iterator mit=KFcounter.begin(), mend=KFcounter.end(); mit!=mend; mit++)
    {
        if(!upParent)
            cout << "  UPDATE_CONN: KF " << mit->first->mnId << " ; num matches: " << mit->second << endl;
        if(mit->second>nmax)
        {
            nmax=mit->second;
            pKFmax=mit->first;
        }
        if(mit->second>=th)
        {
            vPairs.push_back(make_pair(mit->second,mit->first));
            (mit->first)->AddConnection(this,mit->second);
        }
    }

    if(vPairs.empty())
    {
        vPairs.push_back(make_pair(nmax,pKFmax));
        pKFmax->AddConnection(this,nmax);
    }

    sort(vPairs.begin(),vPairs.end());
    list<KeyFrame*> lKFs;
    list<int> lWs;
    for(size_t i=0; i<vPairs.size();i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    {
        unique_lock<mutex> lockCon(mMutexConnections);

        mConnectedKeyFrameWeights = KFcounter;
        mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
        mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());


        if(mbFirstConnection && mnId!=mpMap->GetInitKFid())
        {
            mpParent = mvpOrderedConnectedKeyFrames.front();
            mpParent->AddChild(this);
            mbFirstConnection = false;
        }

    }
}

void KeyFrame::AddChild(KeyFrame *pKF, bool fromRos)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    
    if(!fromRos) {
      notifyObserverKFAction(mnId, 7, pKF->mnId);
    }
    
    mspChildrens.insert(pKF);
    //notifyObserverAddChild(mnId, pKF->mnId);
}

void KeyFrame::EraseChild(KeyFrame *pKF, bool fromRos)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    
    if(!fromRos) {
      notifyObserverKFAction(mnId, 8, pKF->mnId);
    }
    
    mspChildrens.erase(pKF);
}

void KeyFrame::ChangeParent(KeyFrame *pKF, bool fromRos)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    if(pKF == this)
    {
        cout << "ERROR: Change parent KF, the parent and child are the same KF" << endl;
        throw std::invalid_argument("The parent and child can not be the same");
    }
    
    if(!fromRos) {
      notifyObserverKFAction(mnId, 9, pKF->mnId);
    }
    

    mpParent = pKF;
    pKF->AddChild(this);
}

set<KeyFrame*> KeyFrame::GetChilds()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens;
}

KeyFrame* KeyFrame::GetParent()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mpParent;
}

bool KeyFrame::hasChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens.count(pKF);
}

void KeyFrame::SetFirstConnection(bool bFirst, bool fromRos)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    
    if(!fromRos) {
      notifyObserverKFAction(mnId, 18, true);
    }
    
    mbFirstConnection=bFirst;
}

void KeyFrame::AddLoopEdge(KeyFrame *pKF, bool fromRos)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    
    if(!fromRos) {
      notifyObserverKFAction(mnId, 10, pKF->mnId);
    }
    
    mbNotErase = true;
    mspLoopEdges.insert(pKF);
}

set<KeyFrame*> KeyFrame::GetLoopEdges()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspLoopEdges;
}

void KeyFrame::AddMergeEdge(KeyFrame* pKF, bool fromRos)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    
    if(!fromRos) {
      notifyObserverKFAction(mnId, 11, pKF->mnId);
    }
    
    mbNotErase = true;
    mspMergeEdges.insert(pKF);
}

set<KeyFrame*> KeyFrame::GetMergeEdges()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspMergeEdges;
}

void KeyFrame::SetNotErase(bool fromRos)
{
    unique_lock<mutex> lock(mMutexConnections);
    
    if(!fromRos) {
      notifyObserverKFAction(mnId, 19, true);
    }
    
    mbNotErase = true;
}

void KeyFrame::SetErase(bool fromRos)
{
    
    if(!fromRos) {
      notifyObserverKFAction(mnId, 20, true);
    }
    
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mspLoopEdges.empty())
        {
            mbNotErase = false;
        }
    }

    if(mbToBeErased)
    {
        SetBadFlag();
    }
}

void KeyFrame::SetBadFlag(bool fromRos)
{
    {
        unique_lock<mutex> lock(mMutexConnections);
    
        if(!fromRos) {
          notifyObserverKFAction(mnId, 21, true);
        }
    
        if(mnId==mpMap->GetInitKFid())
        {
            return;
        }
        else if(mbNotErase)
        {
            mbToBeErased = true;
            return;
        }
    }

    for(map<KeyFrame*,int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
    {
        mit->first->EraseConnection(this);
    }
    for(size_t i=0; i<mvpMapPoints.size(); i++)
    {
        if(mvpMapPoints[i])
        {
            mvpMapPoints[i]->EraseObservation(this);
        }
    }
    {
        unique_lock<mutex> lock(mMutexConnections);
        unique_lock<mutex> lock1(mMutexFeatures);

        mConnectedKeyFrameWeights.clear();
        mvpOrderedConnectedKeyFrames.clear();


        // Update Spanning Tree
        set<KeyFrame*> sParentCandidates;
        if(mpParent)
            sParentCandidates.insert(mpParent);

        // Assign at each iteration one children with a parent (the pair with highest covisibility weight)
        // Include that children as new parent candidate for the rest
        while(!mspChildrens.empty())
        {
            bool bContinue = false;

            int max = -1;
            KeyFrame* pC;
            KeyFrame* pP;

            for(set<KeyFrame*>::iterator sit=mspChildrens.begin(), send=mspChildrens.end(); sit!=send; sit++)
            {
                KeyFrame* pKF = *sit;
                if(pKF->isBad())
                    continue;

                // Check if a parent candidate is connected to the keyframe
                vector<KeyFrame*> vpConnected = pKF->GetVectorCovisibleKeyFrames();
                for(size_t i=0, iend=vpConnected.size(); i<iend; i++)
                {
                    for(set<KeyFrame*>::iterator spcit=sParentCandidates.begin(), spcend=sParentCandidates.end(); spcit!=spcend; spcit++)
                    {
                        if(vpConnected[i]->mnId == (*spcit)->mnId)
                        {
                            int w = pKF->GetWeight(vpConnected[i]);
                            if(w>max)
                            {
                                pC = pKF;
                                pP = vpConnected[i];
                                max = w;
                                bContinue = true;
                            }
                        }
                    }
                }
            }

            if(bContinue)
            {
                pC->ChangeParent(pP);
                sParentCandidates.insert(pC);
                mspChildrens.erase(pC);
            }
            else
                break;
        }

        // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
        if(!mspChildrens.empty())
        {
            for(set<KeyFrame*>::iterator sit=mspChildrens.begin(); sit!=mspChildrens.end(); sit++)
            {
                (*sit)->ChangeParent(mpParent);
            }
        }

        if(mpParent){
            mpParent->EraseChild(this);
            mTcp = mTcw * mpParent->GetPoseInverse();
        }
        mbBad = true;
    }

    mpMap->EraseKeyFrame(this);
    mpKeyFrameDB->erase(this);
}

bool KeyFrame::isBad()
{
    unique_lock<mutex> lock(mMutexConnections);
    return mbBad;
}

void KeyFrame::EraseConnection(KeyFrame* pKF, bool fromRos)
{
    bool bUpdate = false;
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mConnectedKeyFrameWeights.count(pKF))
        {
            mConnectedKeyFrameWeights.erase(pKF);
            bUpdate=true;
        }
    }

    if(bUpdate)
        UpdateBestCovisibles();
}


vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r, const bool bRight) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    float factorX = r;
    float factorY = r;

    const int nMinCellX = max(0,(int)floor((x-mnMinX-factorX)*mfGridElementWidthInv));
    if(nMinCellX>=mnGridCols)
        return vIndices;

    const int nMaxCellX = min((int)mnGridCols-1,(int)ceil((x-mnMinX+factorX)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-factorY)*mfGridElementHeightInv));
    if(nMinCellY>=mnGridRows)
        return vIndices;

    const int nMaxCellY = min((int)mnGridRows-1,(int)ceil((y-mnMinY+factorY)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = (!bRight) ? mGrid[ix][iy] : mGridRight[ix][iy];
            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = (NLeft == -1) ? mvKeysUn[vCell[j]]
                                                         : (!bRight) ? mvKeys[vCell[j]]
                                                                     : mvKeysRight[vCell[j]];
                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool KeyFrame::IsInImage(const float &x, const float &y) const
{
    return (x>=mnMinX && x<mnMaxX && y>=mnMinY && y<mnMaxY);
}

bool KeyFrame::UnprojectStereo(int i, Eigen::Vector3f &x3D)
{
    const float z = mvDepth[i];
    if(z>0)
    {
        const float u = mvKeys[i].pt.x;
        const float v = mvKeys[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        Eigen::Vector3f x3Dc(x, y, z);

        unique_lock<mutex> lock(mMutexPose);
        x3D = mRwc * x3Dc + mTwc.translation();
        return true;
    }
    else
        return false;
}

float KeyFrame::ComputeSceneMedianDepth(const int q)
{
    if(N==0)
        return -1.0;

    vector<MapPoint*> vpMapPoints;
    Eigen::Matrix3f Rcw;
    Eigen::Vector3f tcw;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPose);
        vpMapPoints = mvpMapPoints;
        tcw = mTcw.translation();
        Rcw = mRcw;
    }

    vector<float> vDepths;
    vDepths.reserve(N);
    Eigen::Matrix<float,1,3> Rcw2 = Rcw.row(2);
    float zcw = tcw(2);
    for(int i=0; i<N; i++) {
        if(mvpMapPoints[i])
        {
            MapPoint* pMP = mvpMapPoints[i];
            Eigen::Vector3f x3Dw = pMP->GetWorldPos();
            float z = Rcw2.dot(x3Dw) + zcw;
            vDepths.push_back(z);
        }
    }

    sort(vDepths.begin(),vDepths.end());

    return vDepths[(vDepths.size()-1)/q];
}

void KeyFrame::SetNewBias(const IMU::Bias &b, bool fromRos)
{
    unique_lock<mutex> lock(mMutexPose);
    
    if(!fromRos) {
      notifyObserverKFAction(mnId, 13, b);
    }
    
    mImuBias = b;
    if(mpImuPreintegrated)
        mpImuPreintegrated->SetNewBias(b);
}

Eigen::Vector3f KeyFrame::GetGyroBias()
{
    unique_lock<mutex> lock(mMutexPose);
    return Eigen::Vector3f(mImuBias.bwx, mImuBias.bwy, mImuBias.bwz);
}

Eigen::Vector3f KeyFrame::GetAccBias()
{
    unique_lock<mutex> lock(mMutexPose);
    return Eigen::Vector3f(mImuBias.bax, mImuBias.bay, mImuBias.baz);
}

IMU::Bias KeyFrame::GetImuBias()
{
    unique_lock<mutex> lock(mMutexPose);
    return mImuBias;
}

Map* KeyFrame::GetMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mpMap;
}

void KeyFrame::UpdateMap(Map* pMap, bool fromRos)
{
    unique_lock<mutex> lock(mMutexMap);
    
    //if(!fromRos) {
    //  notifyObserverKFAction(mnId, 14, pMap->GetId());
    //}
    
    mpMap = pMap;
}

void KeyFrame::PreSave(set<KeyFrame*>& spKF,set<MapPoint*>& spMP, set<GeometricCamera*>& spCam)
{
    // Save the id of each MapPoint in this KF, there can be null pointer in the vector
    mvBackupMapPointsId.clear();
    mvBackupMapPointsId.reserve(N);
    for(int i = 0; i < N; ++i)
    {

        if(mvpMapPoints[i] && spMP.find(mvpMapPoints[i]) != spMP.end()) // Checks if the element is not null
            mvBackupMapPointsId.push_back(mvpMapPoints[i]->mstrHexId);
        else // If the element is null his value is -1 because all the id are positives
            mvBackupMapPointsId.push_back("");
    }
    // Save the id of each connected KF with it weight
    mBackupConnectedKeyFrameIdWeights.clear();
    for(std::map<KeyFrame*,int>::const_iterator it = mConnectedKeyFrameWeights.begin(), end = mConnectedKeyFrameWeights.end(); it != end; ++it)
    {
        if(spKF.find(it->first) != spKF.end())
            mBackupConnectedKeyFrameIdWeights[it->first->mnId] = it->second;
    }

    // Save the parent id
    mBackupParentId = -1;
    if(mpParent && spKF.find(mpParent) != spKF.end())
        mBackupParentId = mpParent->mnId;

    // Save the id of the childrens KF
    mvBackupChildrensId.clear();
    mvBackupChildrensId.reserve(mspChildrens.size());
    for(KeyFrame* pKFi : mspChildrens)
    {
        if(spKF.find(pKFi) != spKF.end())
            mvBackupChildrensId.push_back(pKFi->mnId);
    }

    // Save the id of the loop edge KF
    mvBackupLoopEdgesId.clear();
    mvBackupLoopEdgesId.reserve(mspLoopEdges.size());
    for(KeyFrame* pKFi : mspLoopEdges)
    {
        if(spKF.find(pKFi) != spKF.end())
            mvBackupLoopEdgesId.push_back(pKFi->mnId);
    }

    // Save the id of the merge edge KF
    mvBackupMergeEdgesId.clear();
    mvBackupMergeEdgesId.reserve(mspMergeEdges.size());
    for(KeyFrame* pKFi : mspMergeEdges)
    {
        if(spKF.find(pKFi) != spKF.end())
            mvBackupMergeEdgesId.push_back(pKFi->mnId);
    }

    //Camera data
    mnBackupIdCamera = -1;
    if(mpCamera && spCam.find(mpCamera) != spCam.end())
        mnBackupIdCamera = mpCamera->GetId();

    mnBackupIdCamera2 = -1;
    if(mpCamera2 && spCam.find(mpCamera2) != spCam.end())
        mnBackupIdCamera2 = mpCamera2->GetId();

    //Inertial data
    mBackupPrevKFId = -1;
    if(mPrevKF && spKF.find(mPrevKF) != spKF.end())
        mBackupPrevKFId = mPrevKF->mnId;

    mBackupNextKFId = -1;
    if(mNextKF && spKF.find(mNextKF) != spKF.end())
        mBackupNextKFId = mNextKF->mnId;

    if(mpImuPreintegrated)
        mBackupImuPreintegrated.CopyFrom(mpImuPreintegrated);
}

void KeyFrame::PostLoad(map<long unsigned int, KeyFrame*>& mpKFid, map<std::string, MapPoint*>& mpMPid, map<unsigned int, GeometricCamera*>& mpCamId, bool* bUnprocessed){
    // Rebuild the empty variables

    // Pose
    SetPose(mTcw);
    mTrl = mTlr.inverse();

    //std::cout << "after setpose" << std::endl;
    // Reference reconstruction
    // Each MapPoint sight from this KeyFrame
    mvpMapPoints.clear();
    mvpMapPoints.resize(N);
    
    // TODO: Here we need to check the local mapping id and tracking id
    for(int i=0; i<N; ++i)  
    {
        if(mvBackupMapPointsId[i]!="" && mvBackupMapPointsId[i].length() == 6)
        {
          // Check if map point ptr can be found, if not, return
          if(mpMPid.find(mvBackupMapPointsId[i]) == mpMPid.end()) {
            std::cout << "MP is unprocessed " << mvBackupMapPointsId[i] << std::endl;
            *bUnprocessed = true; 
            return;
          }

          mvpMapPoints[i] = mpMPid[mvBackupMapPointsId[i]];
        } else {
          mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
        }
    }

    //std::cout << "after mvp map point stuff" << std::endl;
    // Conected KeyFrames with him weight
    mConnectedKeyFrameWeights.clear();
    for(map<long unsigned int, int>::const_iterator it = mBackupConnectedKeyFrameIdWeights.begin(), end = mBackupConnectedKeyFrameIdWeights.end(); it != end; ++it)
    {
        // Check if keyframe ptr can be found, if not, return
        if(mpKFid.find(it->first) == mpKFid.end()) {
          *bUnprocessed = true; 
          std::cout << "mConnectedKeyFrameWeights" << std::endl;
          return;
        }
        KeyFrame* pKFi = mpKFid[it->first];
        mConnectedKeyFrameWeights[pKFi] = it->second;
    }

    //std::cout << "after connected kfs" << std::endl;
    // Restore parent KeyFrame
    if(mBackupParentId != -1)
    {
      if(mpKFid.find(mBackupParentId) == mpKFid.end()) {
        *bUnprocessed = true; 
        std::cout << "ERROR: parentKF=" << mBackupParentId << " not found. This ID=" << mnId << std::endl;
        return;
      }
      mpParent = mpKFid[mBackupParentId];

    } else {
      mpParent = static_cast<KeyFrame*>(NULL);
    }

    //std::cout << "after parent" << std::endl;
    // KeyFrame childrens
    mspChildrens.clear();
    for(vector<long unsigned int>::const_iterator it = mvBackupChildrensId.begin(), end = mvBackupChildrensId.end(); it!=end; ++it)
    {
        if(mpKFid.find(*it) == mpKFid.end()) {
          *bUnprocessed = true; 
          std::cout << "mspChildrens" << std::endl;
          return;
        }
        mspChildrens.insert(mpKFid[*it]);
    }

    //std::cout << "after mspchildrens" << std::endl;
    // Loop edge KeyFrame
    mspLoopEdges.clear();
    for(vector<long unsigned int>::const_iterator it = mvBackupLoopEdgesId.begin(), end = mvBackupLoopEdgesId.end(); it != end; ++it)
    {
        if(mpKFid.find(*it) == mpKFid.end()) {
          *bUnprocessed = true; 
          std::cout << "mspLoopEdges" << std::endl;
          return;
        }
        mspLoopEdges.insert(mpKFid[*it]);
    }

    //std::cout << "after msploopedges" << std::endl;
    // Merge edge KeyFrame
    mspMergeEdges.clear();
    for(vector<long unsigned int>::const_iterator it = mvBackupMergeEdgesId.begin(), end = mvBackupMergeEdgesId.end(); it != end; ++it)
    {
        if(mpKFid.find(*it) == mpKFid.end()) {
          *bUnprocessed = true; 
          std::cout << "mspMergeEdges" << std::endl;
          return;
        }
        mspMergeEdges.insert(mpKFid[*it]);
    }

    //std::cout << "after mspMergeEdges" << std::endl;
    //Camera data
    if(mnBackupIdCamera >= 0)
    {
        mpCamera = mpCamId[mnBackupIdCamera];
    }
    else
    {
        cout << "ERROR: There is not a main camera in KF " << mnId << endl;
    }
    if(mnBackupIdCamera2 >= 0)
    {
        mpCamera2 = mpCamId[mnBackupIdCamera2];
    }

    //std::cout << "after backups" << std::endl;
    //Inertial data
    if(mBackupPrevKFId != -1)
    {
        if(mpKFid.find(mBackupPrevKFId) == mpKFid.end()) {
          *bUnprocessed = true; 
          std::cout << "ERROR: prevKF=" << mBackupPrevKFId << " not found. This ID=" << mnId << std::endl;
          return;
        }
        mPrevKF = mpKFid[mBackupPrevKFId];
    } else {
      mPrevKF = static_cast<KeyFrame*>(NULL);
    }
    //std::cout << "after mPrevKF" << std::endl;
    if(mBackupNextKFId != -1)
    {
        if(mpKFid.find(mBackupNextKFId) == mpKFid.end()) {
          *bUnprocessed = true; 
          std::cout << "ERROR: nextKF=" << mBackupNextKFId << " not found. This ID=" << mnId << std::endl;
          return;
        }
        mNextKF = mpKFid[mBackupNextKFId];
    } else {
      mNextKF = static_cast<KeyFrame*>(NULL);
    }
    //std::cout << "after mNextKF" << std::endl;
    mpImuPreintegrated = &mBackupImuPreintegrated;


    //std::cout << "after imu" << std::endl;
    // Remove all backup container
    mvBackupMapPointsId.clear();
    mBackupConnectedKeyFrameIdWeights.clear();
    mvBackupChildrensId.clear();
    mvBackupLoopEdgesId.clear();

    UpdateBestCovisibles();
    //std::cout << "after update cov" << std::endl;
}

bool KeyFrame::ProjectPointDistort(MapPoint* pMP, cv::Point2f &kp, float &u, float &v)
{

    // 3D in absolute coordinates
    Eigen::Vector3f P = pMP->GetWorldPos();

    // 3D in camera coordinates
    Eigen::Vector3f Pc = mRcw * P + mTcw.translation();
    float &PcX = Pc(0);
    float &PcY = Pc(1);
    float &PcZ = Pc(2);

    // Check positive depth
    if(PcZ<0.0f)
    {
        cout << "Negative depth: " << PcZ << endl;
        return false;
    }

    // Project in image and check it is not outside
    float invz = 1.0f/PcZ;
    u=fx*PcX*invz+cx;
    v=fy*PcY*invz+cy;

    // cout << "c";

    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;

    float x = (u - cx) * invfx;
    float y = (v - cy) * invfy;
    float r2 = x * x + y * y;
    float k1 = mDistCoef.at<float>(0);
    float k2 = mDistCoef.at<float>(1);
    float p1 = mDistCoef.at<float>(2);
    float p2 = mDistCoef.at<float>(3);
    float k3 = 0;
    if(mDistCoef.total() == 5)
    {
        k3 = mDistCoef.at<float>(4);
    }

    // Radial distorsion
    float x_distort = x * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
    float y_distort = y * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);

    // Tangential distorsion
    x_distort = x_distort + (2 * p1 * x * y + p2 * (r2 + 2 * x * x));
    y_distort = y_distort + (p1 * (r2 + 2 * y * y) + 2 * p2 * x * y);

    float u_distort = x_distort * fx + cx;
    float v_distort = y_distort * fy + cy;

    u = u_distort;
    v = v_distort;

    kp = cv::Point2f(u, v);

    return true;
}

bool KeyFrame::ProjectPointUnDistort(MapPoint* pMP, cv::Point2f &kp, float &u, float &v)
{

    // 3D in absolute coordinates
    Eigen::Vector3f P = pMP->GetWorldPos();

    // 3D in camera coordinates
    Eigen::Vector3f Pc = mRcw * P + mTcw.translation();
    float &PcX = Pc(0);
    float &PcY= Pc(1);
    float &PcZ = Pc(2);

    // Check positive depth
    if(PcZ<0.0f)
    {
        cout << "Negative depth: " << PcZ << endl;
        return false;
    }

    // Project in image and check it is not outside
    const float invz = 1.0f/PcZ;
    u = fx * PcX * invz + cx;
    v = fy * PcY * invz + cy;

    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;

    kp = cv::Point2f(u, v);

    return true;
}

Sophus::SE3f KeyFrame::GetRelativePoseTrl()
{
    unique_lock<mutex> lock(mMutexPose);
    return mTrl;
}

Sophus::SE3f KeyFrame::GetRelativePoseTlr()
{
    unique_lock<mutex> lock(mMutexPose);
    return mTlr;
}

Sophus::SE3<float> KeyFrame::GetRightPose() {
    unique_lock<mutex> lock(mMutexPose);

    return mTrl * mTcw;
}

Sophus::SE3<float> KeyFrame::GetRightPoseInverse() {
    unique_lock<mutex> lock(mMutexPose);

    return mTwc * mTlr;
}

Eigen::Vector3f KeyFrame::GetRightCameraCenter() {
    unique_lock<mutex> lock(mMutexPose);

    return (mTwc * mTlr).translation();
}

Eigen::Matrix<float,3,3> KeyFrame::GetRightRotation() {
    unique_lock<mutex> lock(mMutexPose);

    return (mTrl.so3() * mTcw.so3()).matrix();
}

Eigen::Vector3f KeyFrame::GetRightTranslation() {
    unique_lock<mutex> lock(mMutexPose);
    return (mTrl * mTcw).translation();
}

void KeyFrame::SetORBVocabulary(ORBVocabulary* pORBVoc)
{
    mpORBvocabulary = pORBVoc;
}

void KeyFrame::SetKeyFrameDatabase(KeyFrameDatabase* pKFDB)
{
    mpKeyFrameDB = pKFDB;
}

std::map<long unsigned int, int> KeyFrame::GetBackupConnectedKeyFrameIdWeights(){
  unique_lock<mutex> lock(mMutexConnections);
  std::map<long unsigned int, int> mBackupConnKFweigths;
  for(const auto& entry : mConnectedKeyFrameWeights) {
    long unsigned int id = entry.first->mnId;
    int weight = entry.second;
    mBackupConnKFweigths.insert(std::make_pair(id, weight));
  }

  return mBackupConnKFweigths;

}

} //namespace ORB_SLAM
