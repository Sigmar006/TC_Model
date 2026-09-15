#include "sbirs_sensor/runtime/SbirsPipelineConfigMapper.h"

namespace sbirs_sensor {
namespace runtime {

config::SbirsInternalExecutionConfig MapSessionToInternal(
    const config::SbirsSessionConfig& config) {
  config::SbirsInternalExecutionConfig internal;
  internal.session = config;
  return internal;
}

}  // namespace runtime
}  // namespace sbirs_sensor
