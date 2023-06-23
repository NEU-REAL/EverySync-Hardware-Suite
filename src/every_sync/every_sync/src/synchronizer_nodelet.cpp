#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "every_sync/synchronizer.h"

namespace every_sync {

class EverySyncNodelet : public nodelet::Nodelet {
  virtual void onInit() {
    try {
      synchronizer_ = std::make_shared<Synchronizer>(
          getNodeHandle(), getPrivateNodeHandle());
    } catch (std::runtime_error e) {
      ROS_ERROR("%s", e.what());
    }
  }

  std::shared_ptr<Synchronizer> synchronizer_;
};
} // namespace every_sync

PLUGINLIB_EXPORT_CLASS(every_sync::EverySyncNodelet, nodelet::Nodelet);
