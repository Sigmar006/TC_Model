#include "1q/sbirs_sensor/session/SbirsCycleOutputAdapter.h"

namespace sbirs_sensor {
namespace session {

bool SbirsOutputFrameContainsOnlyNativeFields(const SbirsOutputFrame& frame) {
  for (const output::SbirsDetectionRecord& record : frame.detections) {
    if (!record.detected) {
      return false;
    }
  }
  return true;
}

}  // namespace session
}  // namespace sbirs_sensor
