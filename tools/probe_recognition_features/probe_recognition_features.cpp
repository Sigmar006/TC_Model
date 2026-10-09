/**
 * @file probe_recognition_features.cpp
 * @brief 开发期探针：直接调库内识别提取器，打印我铺的特征真值实际算出什么。
 *
 * 目的：定位"特征铺齐后识别置信度反而降到 0.120"的原因——是提取端没算出预期值，
 * 还是模板均值口径与我猜的不同。只读探针，不跑会话、不改库。
 */

#include <cstdio>
#include <vector>

#include "1q/remote_identification_radar/session/RirSceneTypes.h"
#include "remote_identification_radar/recognition/PolarizationFeatureExtractor.h"
#include "remote_identification_radar/recognition/RangeProfileFeatureExtractor.h"
#include "remote_identification_radar/recognition/RcsFeatureExtractor.h"

namespace {

namespace rec = remote_identification_radar::recognition;
namespace ses = remote_identification_radar::session;

ses::RirPolSMatrixSample MakePol(float hh, float vv) {
  ses::RirPolSMatrixSample s;
  s.aspect_az_deg = 0.0f;
  s.aspect_el_deg = 14.0f;
  s.hh_amp_db = hh;
  s.hh_phase_deg = 0.0f;
  s.hv_amp_db = -60.0f;
  s.hv_phase_deg = 0.0f;
  s.vh_amp_db = -60.0f;
  s.vh_phase_deg = 0.0f;
  s.vv_amp_db = vv;
  s.vv_phase_deg = 0.0f;
  return s;
}

}  // namespace

int main() {
  std::printf("=== recognition feature probe ===\n\n");
  const float kSnrDb = 20.0f;
  const float kRangeM = 200000.0f;

  // ---- 1) 极化（只消费 HH/VV）----
  std::printf("[POL] template BALLISTIC_EXAMPLE_A: E_diff=2.0(std1.5) "
              "rel_diff=-6.0(std2.0) E_sum=5.0(std4.0)\n");
  const float pairs[][2] = {{2.0f, 0.0f}, {0.0f, 0.0f}, {-2.0f, 0.0f}, {6.0f, 0.0f}};
  for (const auto& pr : pairs) {
    std::vector<ses::RirPolSMatrixSample> w{MakePol(pr[0], pr[1])};
    const rec::RirPolarizationObservation o =
        rec::RirPolarizationFeatureExtractor::Extract(w, 0.0f, 14.0f, kSnrDb, kRangeM);
    std::printf("      hh=%+.1f vv=%+.1f -> valid=%d E_diff=%+.3f rel_diff=%+.3f "
                "E_sum=%+.3f q=%.3f\n",
                static_cast<double>(pr[0]), static_cast<double>(pr[1]), o.valid ? 1 : 0,
                static_cast<double>(o.energy_difference_db),
                static_cast<double>(o.relative_difference_db),
                static_cast<double>(o.energy_sum_db), static_cast<double>(o.quality));
  }

  // ---- 2) RCS ----
  std::printf("\n[RCS] template: mean=-3.0(std2.0) az_var=4.0 el_var=3.0 min_cov=15\n");
  std::vector<ses::RirAspectRcsSample> samples;
  for (float az = -10.0f; az <= 10.0f; az += 5.0f) {
    for (float el = 5.0f; el <= 30.0f; el += 10.0f) {
      ses::RirAspectRcsSample a;
      a.aspect_az_deg = az;
      a.aspect_el_deg = el;
      a.rcs_dbsm = -3.0f;
      samples.push_back(a);
    }
  }
  const rec::RirRcsObservation r = rec::RirRcsFeatureExtractor::Extract(samples, 0.0f, 14.0f,
                                                                       kSnrDb, 15.0f);
  std::printf("      20 samples @ -3 dBsm -> valid=%d mean=%.3f std=%.3f az_var=%.3f "
              "el_var=%.3f p2v=%.3f cov=%.3f q=%.3f\n",
              r.valid ? 1 : 0, static_cast<double>(r.mean_dbsm), static_cast<double>(r.std_db),
              static_cast<double>(r.azimuth_variation_db),
              static_cast<double>(r.elevation_variation_db),
              static_cast<double>(r.peak_to_valley_db),
              static_cast<double>(r.aspect_coverage_deg), static_cast<double>(r.quality));

  // ---- 3) 距离像 ----
  std::printf("\n[RNG] template: length=8.0(std2.0) peaks=3.0(std1.0) conc=0.75(std0.1) "
              "min_bw=3e6\n");
  std::vector<ses::RirRangeRcsScatterer> sc;
  for (float off : {-4.0f, 0.0f, 4.0f}) {
    ses::RirRangeRcsScatterer e;
    e.range_offset_m = off;
    e.rcs_dbsm = -6.0f;
    e.phase_deg = 0.0f;
    sc.push_back(e);
  }
  const rec::RirRangeProfileObservation g =
      rec::RirRangeProfileFeatureExtractor::Extract(sc, 4.5e6f, kSnrDb, 50.0f);
  std::printf("      3 scatterers spanning 8 m -> valid=%d length=%.3f peaks=%u conc=%.3f "
              "res=%.3f q=%.3f\n",
              g.valid ? 1 : 0, static_cast<double>(g.length_m), g.peak_count,
              static_cast<double>(g.peak_energy_concentration),
              static_cast<double>(g.resolution_m), static_cast<double>(g.quality));
  return 0;
}
