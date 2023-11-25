
#ifndef OBSERVER_H
#define OBSERVER_H

#include "KeyFrame.h"

namespace ORB_SLAM3
{

class Observer {
  public:
        virtual void onKeyframeChanged(int keyframeId) = 0;
        virtual void onKeyframeAdded(KeyFrame* kf) = 0;
        virtual ~Observer() {}
};

}// namespace ORB_SLAM

#endif // SYSTEM_H

