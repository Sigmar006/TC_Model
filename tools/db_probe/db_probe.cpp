// @file db_probe.cpp
// 一次性排查工具：用 1q 的真实识别库加载器读指定 sqlite 库，打印模板值。
// 用途：定位"改了库但集成行为不变"是加载失败、路径不符，还是值确实不同。

#include <cstdio>
#include <string>

#include <remote_identification_radar/recognition/RecognitionFeatureDatabase.h>

int main(int argc, char** argv) {
  if (argc < 2) {
    std::printf("usage: db_probe <db_path> [model_id]\n");
    return 2;
  }
  const std::string path = argv[1];
  const std::string model_id = argc >= 3 ? argv[2] : "BALLISTIC_EXAMPLE_B";

  remote_identification_radar::recognition::RirFeatureDatabase db;
  std::string error;
  const bool ok =
      remote_identification_radar::recognition::RirFeatureDatabase::Load(path, &db, &error);
  std::printf("load=%d path=%s\n", ok ? 1 : 0, path.c_str());
  if (!ok) {
    std::printf("error='%s'\n", error.c_str());
    return 1;
  }
  std::printf("version='%s' loaded=%d models=%u\n", db.version().c_str(), db.IsLoaded() ? 1 : 0,
              static_cast<unsigned>(db.models().size()));
  for (std::size_t m = 0; m < db.models().size(); ++m) {
    const remote_identification_radar::recognition::RirModel& model = db.models()[m];
    if (model.model_id != model_id) {
      continue;
    }
    std::printf("model_id='%s' category='%s' profiles=%u\n", model.model_id.c_str(),
                model.category_id.c_str(), static_cast<unsigned>(model.profiles.size()));
    for (std::size_t p = 0; p < model.profiles.size(); ++p) {
      const remote_identification_radar::recognition::RirModelProfile& profile =
          model.profiles[p];
      std::printf(
          "  profile[%u] id='%s' min_snr_db=%.3f max_range_res_m=%.1f\n"
          "    speed      mean=%.3f std=%.3f\n"
          "    altitude   mean=%.3f std=%.3f\n"
          "    accel      mean=%.3f std=%.3f\n"
          "    turn_log10 mean=%.3f std=%.3f\n"
          "    rcs        mean=%.3f std=%.3f\n",
          static_cast<unsigned>(p), profile.profile_id.c_str(), profile.min_snr_db,
          profile.max_range_resolution_m, profile.motion.speed_mps.mean,
          profile.motion.speed_mps.std, profile.motion.altitude_m.mean,
          profile.motion.altitude_m.std, profile.motion.acceleration_mps2.mean,
          profile.motion.acceleration_mps2.std, profile.motion.turn_radius_log10.mean,
          profile.motion.turn_radius_log10.std, profile.rcs.mean_dbsm, profile.rcs.std_db);
    }
    return 0;
  }
  std::printf("model_id='%s' not found\n", model_id.c_str());
  return 3;
}
