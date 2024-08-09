#pragma once

#ifndef DISTRIBUTOR_H
#define DISTRIBUTOR_H
#include <set>

#include "Thirdparty/Sophus/sophus/geometry.hpp"
#include "Thirdparty/Sophus/sophus/sim3.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <ImuTypes.h>
//#include "KeyFrame.h"
//#include "Map.h"

namespace ORB_SLAM3
{
class KeyFrame;
class MapPoint;
class Map;

class Distributor {
  public:
        virtual void onKeyframeAdded(KeyFrame* pKF) = 0;
        virtual void onKeyframeAdded(KeyFrame* pKF, std::set<std::string> msNewMapPointIds) = 0;
        //virtual void onMapAdded(Map* pM) = 0;
        virtual void onNewMap(ORB_SLAM3::Map* pM) = 0;
        virtual void onNewKeyFrame(ORB_SLAM3::KeyFrame* pKF) = 0;
        virtual void onNewMapPoint(ORB_SLAM3::MapPoint* pMP) = 0;
        virtual void onLocalMapUpdated(Map* pM) = 0;
        virtual void onGlobalMapUpdated(bool mbMerged, bool mbLoopClosure, std::vector<unsigned long int> mvMergeIds) = 0;
        //virtual void onChangeLMActive(bool bActive) = 0;
        virtual void onLMResetRequested() = 0;
        virtual void onLMStopRequest(const bool bStopLM) = 0;
        virtual void onActiveMapReset(unsigned long int mnMapId) = 0;
        
        virtual int KeyFramesInQueue() = 0;
        virtual int MapsInQueue() = 0;


        virtual ~Distributor() {}
};

}// namespace ORB_SLAM

#endif // DISTRIBUTOR_H

