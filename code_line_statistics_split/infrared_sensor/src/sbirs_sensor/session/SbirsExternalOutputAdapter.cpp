#include "1q/sbirs_sensor/session/SbirsExternalOutputAdapter.h"

namespace sbirs_sensor {
namespace session {

std::size_t CountSbirsDetections(const SbirsOutputFrame& frame) { return frame.detections.size(); }

}  // namespace session
}  // namespace sbirs_sensor
