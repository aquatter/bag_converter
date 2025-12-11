#pragma once
#include <functional>
#include <memory>
#include <string>
#include <vector>

struct GPMFChunkBase;

struct GPMFParserSettings {
  double resize_;
  std::vector<std::string> paths_to_mp4_;
  std::string output_path_;
  int jpeg_quality_;
  bool extract_images_;
  bool compress_;
  bool no_images_;
  int64_t start_time_;
  int64_t end_time_;
  bool save_geojson_;
  bool save_bag_;
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