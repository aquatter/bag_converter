#pragma once
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

std::optional<std::pair<int, int>>
parce_record_part(const std::string_view str);

struct GPMFChunkBase;

struct GPMFParserSettings {
  double resize_;
  std::vector<std::string> paths_to_mp4_;
  std::string output_path_;
  int jpeg_quality_;
  bool extract_images_;
  bool compress_;
  bool no_images_;
  bool no_imu_;
  bool no_gps_;
  int64_t start_time_;
  int64_t end_time_;
  bool save_geojson_;
  bool save_bag_;
  std::vector<std::pair<int64_t, int64_t>> gps_exclusion_intervals_;
  std::string path_to_exclusion_intervals_;
  std::function<void(const GPMFChunkBase *)> callback_;
};

class GPMFParser {
public:
  GPMFParser(const GPMFParserSettings &set);
  void parse();

  ~GPMFParser();

private:
  struct impl;
  std::unique_ptr<impl> pimpl_;
};