#pragma once
#include <memory>
#include <string_view>

struct GPMFParserSettings {
  double resize_;
  std::string path_to_mp4_;
  std::string output_path_;
  int jpeg_quality_;
  bool extract_images_;
  bool compress_;
};

class GPMFParser {
public:
  GPMFParser(const GPMFParserSettings &set);
  void parse();
  void write_bag();

  ~GPMFParser();

private:
  struct impl;
  std::unique_ptr<impl> pimpl_;
};