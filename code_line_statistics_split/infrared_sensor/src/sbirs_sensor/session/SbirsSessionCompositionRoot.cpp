#include "sbirs_sensor/session/SbirsSessionCompositionRoot.h"

#include "sbirs_sensor/runtime/SbirsPipelineConfigMapper.h"

namespace sbirs_sensor {
namespace session {

std::unique_ptr<runtime::SbirsController> CreateSbirsController(
    const config::SbirsSessionConfig& config) {
  return std::unique_ptr<runtime::SbirsController>(
      new runtime::SbirsController(runtime::MapSessionToInternal(config)));
}

}  // namespace session
}  // namespace sbirs_sensor
