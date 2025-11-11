#include "CLI/CLI.hpp"
#include <CLI/CLI.hpp>
#include <cstdlib>
#include <exception>
#include <fmt/color.h>
#include <fmt/format.h>
#include <gpmf_parser.hpp>

int main(int argc, char const *const *argv) {

  try {
    GPMFParserSettings set{};

    CLI::App app{"Convert GOPRO video to ROS2 bag"};

    app.add_option("-i, --input", set.paths_to_mp4_,
                   "Specify input video paths")
        ->required()
        ->check(CLI::ExistingFile);

    app.add_option("-o, --output", set.output_path_,
                   "Specify output ros2 bag path")
        ->required();

    app.add_option("-r, --resize", set.resize_, "Resize factor")
        ->check(CLI::Range{0.0, 1.0})
        ->default_val(1.0);

    app.add_flag("-e, --extract", set.extract_images_, "Extract images")
        ->default_val(false);

    app.add_flag("--compressed", set.compress_, "Create compressed topic")
        ->default_val(false);

    app.add_option("--quality", set.jpeg_quality_, "JPEG quality")
        ->check(CLI::Range{0, 100})
        ->default_val(70);

    app.add_flag("--no-images", set.no_images_, "Don't extract images")
        ->default_val(false);

    CLI11_PARSE(app, argc, argv);

    if (std::filesystem::exists(set.output_path_)) {
      fmt::print("\e[38;2;154;205;50mPath\e[0m \e[38;2;255;127;80m'{}'\e[0m "
                 "\e[38;2;154;205;50malready "
                 "exists. Remove? [y/n] \e[0m",
                 set.output_path_);

      char ans{};
      std::cin >> ans;

      if (ans == 'y' or ans == 'Y') {
        std::filesystem::remove_all(set.output_path_);
      } else {
        return EXIT_FAILURE;
      }
    }

    GPMFParser gpmf_parser{set};
    gpmf_parser.parse();

  } catch (const std::exception &ex) {
    fmt::print(fmt::fg(fmt::color::red), "{}\n", ex.what());
  }

  fmt::print(fmt::fg(fmt::color::yellow_green), "Done\n");
  return EXIT_SUCCESS;
}