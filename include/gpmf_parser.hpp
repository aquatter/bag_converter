#pragma once
#include <memory>
#include <vector>

struct GPMFParserSettings {
  double resize_;
  std::vector<std::string> paths_to_mp4_;
  std::string output_path_;
  int jpeg_quality_;
  bool extract_images_;
  bool compress_;
  bool no_images_;
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