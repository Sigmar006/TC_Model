// Copyright 2026. All Rights Reserved.
//
// @file rir_receive_window_audit_test.cpp
// @brief 2026-09-28 审计 Q-1：验收第30项只选择外部干扰源。

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "1q/electromagnetics/RfScene.h"
#include "remote_identification_radar/runtime/RirAcceptanceRecords.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using oneq::electromagnetics::RfEmissionIdentity;
using oneq::electromagnetics::RfIncidentLinkResult;
using runtime::SelectRirExternalInterferenceLinks;

RfIncidentLinkResult MakeLink(std::uint32_t sensor_entity_id, std::uint64_t emission_id,
                              double received_power_w) {
  RfIncidentLinkResult link;
  link.identity = RfEmissionIdentity{sensor_entity_id, emission_id};
  link.received_power_before_overlap_w = received_power_w;
  link.received_power_w = received_power_w;
  return link;
}

TEST(RirInterferenceSourceSelectionTest, OwnCoSiteCouplingIsExcluded) {
  const RfEmissionIdentity own{5U, 740U};
  const std::vector<RfIncidentLinkResult> links = {
      MakeLink(5U, 740U, 5.249e-8),
      MakeLink(9U, 12U, 1.0e-10),
      MakeLink(7U, 3U, 0.0),
  };
  const std::vector<RfIncidentLinkResult> external =
      SelectRirExternalInterferenceLinks(own, links);
  ASSERT_EQ(external.size(), 2U);
  EXPECT_EQ(external[0].identity.sensor_entity_id, 9U);
  EXPECT_EQ(external[0].identity.emission_id, 12U);
  EXPECT_EQ(external[1].identity.sensor_entity_id, 7U);
  for (const RfIncidentLinkResult& link : external) {
    EXPECT_FALSE(link.identity.sensor_entity_id == own.sensor_entity_id &&
                 link.identity.emission_id == own.emission_id);
  }
}

TEST(RirInterferenceSourceSelectionTest, SameEntityDifferentEmissionIsExternal) {
  const RfEmissionIdentity own{5U, 740U};
  const std::vector<RfIncidentLinkResult> external = SelectRirExternalInterferenceLinks(
      own, {MakeLink(5U, 741U, 1.0e-9), MakeLink(5U, 740U, 5.249e-8)});
  ASSERT_EQ(external.size(), 1U);
  EXPECT_EQ(external[0].identity.emission_id, 741U);
}

TEST(RirInterferenceSourceSelectionTest, OwnOnlyYieldsEmpty) {
  const RfEmissionIdentity own{5U, 740U};
  EXPECT_TRUE(
      SelectRirExternalInterferenceLinks(own, {MakeLink(5U, 740U, 5.249e-8)}).empty());
  EXPECT_TRUE(SelectRirExternalInterferenceLinks(own, {}).empty());
}

TEST(RirInterferenceSourceSelectionTest, InvalidOwnIdentityKeepsAllLinks) {
  const std::vector<RfIncidentLinkResult> links = {MakeLink(5U, 740U, 1.0e-9)};
  EXPECT_EQ(SelectRirExternalInterferenceLinks(RfEmissionIdentity{0U, 740U}, links).size(), 1U);
  EXPECT_EQ(SelectRirExternalInterferenceLinks(RfEmissionIdentity{5U, 0U}, links).size(), 1U);
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
