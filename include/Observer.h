
#ifndef OBSERVER_H
#define OBSERVER_H

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

class Observer {
  public:
        virtual void onKeyframeChanged(int keyframeId) = 0;
        virtual void onKeyframeAdded(KeyFrame* kf) = 0;
        virtual void onMapPointAdded(MapPoint* pMp) = 0;
        //virtual void onMapAdded(Map* pM) = 0;
        virtual void onMapAddedById(unsigned long int id) = 0;
        virtual void onLocalMapUpdated(Map* pM) = 0;
        virtual void onChangeLMActive(bool bActive) = 0;

        virtual ~Observer() {}
};

}// namespace ORB_SLAM

#endif // SYSTEM_H

