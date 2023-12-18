
#ifndef OBSERVER_H
#define OBSERVER_H

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
        virtual void onKFAction(int actionId, Eigen::Vector3f t) = 0;
        virtual void onKFAction(unsigned long int hostKfId, int actionId, Sophus::SE3<float> p) = 0;

        virtual void onAtlasAction(int actionId, unsigned long int id) = 0;
        virtual void onAtlasAction(int actionId, bool boolAction) = 0;


        virtual ~Observer() {}
};

}// namespace ORB_SLAM

#endif // SYSTEM_H

