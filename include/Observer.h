
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

class Observer {
  public:
        virtual void onKeyframeChanged(int keyframeId) = 0;
        virtual void onKeyframeAdded(KeyFrame* kf) = 0;
        //virtual void onMapAdded(Map* pM) = 0;
        virtual void onMapAddedById(unsigned long int id) = 0;
        

        virtual void onKFAction(unsigned long int hostKfId, int actionId, unsigned long int id) = 0;
        virtual void onKFAction(unsigned long int hostKfId, int actionId, bool boolAction) = 0;
        virtual void onKFAction(unsigned long int hostKfId, int actionId, unsigned long int id, long int vectorIdx) = 0;
        virtual void onKFAction(unsigned long int hostKfId, int actionId, Eigen::Vector3f t) = 0;
        virtual void onKFAction(unsigned long int hostKfId, int actionId, Sophus::SE3<float> p) = 0;
        virtual void onKFAction(unsigned long int hostKfId, int actionId, IMU::Bias b) = 0;

        virtual void onAtlasAction(int actionId, unsigned long int id) = 0;
        virtual void onAtlasAction(int actionId, bool boolAction) = 0;

        virtual void onKFDBAction(int actionId, bool boolAction) = 0;
        virtual void onKFDBAction(int actionId, unsigned long int id) = 0;
        virtual void onKFDBAction(int actionId, unsigned long int id, float minScore, std::vector<unsigned long int> vpLoopCandId, std::vector<unsigned long int> vpMergeCandId) = 0;
        virtual void onKFDBAction(int actionId, unsigned long int id, std::vector<unsigned long int> vpLoopCandId, std::vector<unsigned long int> vpMergeCandId, int nMinWords) = 0;

        virtual ~Observer() {}
};

}// namespace ORB_SLAM

#endif // SYSTEM_H

