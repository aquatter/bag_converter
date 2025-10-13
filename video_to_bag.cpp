
#include "CLI/CLI.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include <CLI/CLI.hpp>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fmt/color.h>
#include <fmt/format.h>
#include <iostream>
#include <memory>
#include <opencv2/core/matx.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <progress_bar.hpp>
#include <rclcpp/time.hpp>
#include <regex>
#include <rosbag2_cpp/writer.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/header.hpp>
#include <stdexcept>
#include <tinyxml2.h>

struct VideoProcessSettings {
  std::string input_video_;
  std::string output_bag_;
  std::string gpx_path_;
  bool extract_images_;
  bool compressed_;
  float resize_;
  int jpeq_quality_;
};

struct GPSCoord {
  double lat_;
  double lon_;
  int64_t timestamp_;
};

struct GPSData {
  std::vector<GPSCoord> gps_;
  uint64_t video_timestamp_;
};

[[nodiscard]] int64_t parse_timestamp(const std::string_view str,
                                      const std::string_view pattern) {
  std::istringstream is{str.data()};

  std::tm tm{};
  is >> std::get_time(&tm, pattern.data());

  if (is.fail()) {
    throw std::runtime_error{
        fmt::format("unable to parse timestamp {} with pattern {}", str.data(),
                    pattern.data())};
  }

  std::chrono::system_clock::time_point tp{
      std::chrono::system_clock::from_time_t(mktime(&tm))};

  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             tp.time_since_epoch())
      .count();
}

[[nodiscard]] GPSData parse_gps(const std::string_view path) {
  tinyxml2::XMLDocument doc;

  if (tinyxml2::XML_SUCCESS != doc.LoadFile(path.data())) {
    throw std::runtime_error{
        fmt::format("unable to open file: {}", path.data())};
  }

  const std::string video_filename{doc.FirstChildElement("gpx")
                                       ->FirstChildElement("trk")
                                       ->FirstChildElement("name")
                                       ->GetText()};

  GPSData res;

  const std::regex r{"video(.*).mp4"};

  if (std::smatch match;
      std::regex_match(video_filename, match, r) and match.size() == 2) {
    res.video_timestamp_ = parse_timestamp(match[1].str(), "%Y%m%d-%H%M%S");
  }

  auto track_point_element{doc.FirstChildElement("gpx")
                               ->FirstChildElement("trk")
                               ->FirstChildElement("trkseg")
                               ->FirstChildElement("trkpt")};

  while (track_point_element != nullptr) {

    GPSCoord coord;

    coord.lat_ = track_point_element->DoubleAttribute("lat");
    coord.lon_ = track_point_element->DoubleAttribute("lon");

    const std::string time_str{
        track_point_element->FirstChildElement("time")->GetText()};

    coord.timestamp_ =
        parse_timestamp(time_str, "%Y-%m-%dT%H:%M:%SZ") - res.video_timestamp_;

    res.gps_.push_back(coord);
    track_point_element = track_point_element->NextSiblingElement();
  }

  return res;
}

class VideoProcessor {

public:
  VideoProcessor(const VideoProcessSettings &set) : set_{set} {
    cap_ = std::make_unique<cv::VideoCapture>(set_.input_video_);

    if (not cap_->isOpened()) {
      throw std::invalid_argument{fmt::format(
          "Error: Could not open video file: {}", set_.input_video_)};
    }

    if (not set_.output_bag_.empty()) {
      rosbag2_storage::StorageOptions storage_options;
      storage_options.uri = set_.output_bag_;
      storage_options.max_bagfile_duration = 30;

      writer_ = std::make_unique<rosbag2_cpp::Writer>();
      writer_->open(storage_options);
    }

    if (not set_.gpx_path_.empty()) {
      gps_data_ = parse_gps(set_.gpx_path_);
    }
  }

  void process() {
    size_t gps_ind{0};
    cv::Mat_<cv::Vec3b> img{};

    const auto start_timestamp{
        std::chrono::high_resolution_clock::now().time_since_epoch().count()};

    int64_t frame_ind{0};

    const auto num_frames{
        static_cast<uint64_t>(cap_->get(cv::CAP_PROP_FRAME_COUNT))};

    ProgressBar bar{
        std::vector{ProgressBar::ProgressInfo{.message_count_ = num_frames,
                                              .processed_count_ = 0,
                                              .topic_name_ = "frame",
                                              .ind_ = 0}}};

    bar.draw();

    while (true) {
      *cap_ >> img;

      if (img.empty()) {
        break;
      }

      if (set_.resize_ != 1.0f) {
        cv::resize(img, img,
                   {static_cast<int>(img.cols * set_.resize_),
                    static_cast<int>(img.rows * set_.resize_)});
      }

      const int64_t curr_timestamp{start_timestamp + frame_ind * 33'333'333l};

      if (set_.extract_images_) {
        cv::imwrite(fmt::format("{}/{}.png", set_.output_bag_, curr_timestamp),
                    img);
      }

      if (writer_) {

        if (not gps_data_.gps_.empty() and gps_ind < gps_data_.gps_.size()) {
          int64_t gps_timestamp{start_timestamp +
                                gps_data_.gps_[gps_ind].timestamp_};

          while (curr_timestamp >= gps_timestamp) {

            const int32_t gps_timestamp_sec{
                static_cast<int32_t>(gps_timestamp / 1'000'000'000)};

            const uint32_t gps_timestamp_nanosec{static_cast<uint32_t>(
                gps_timestamp - gps_timestamp_sec * 1'000'000'000)};

            sensor_msgs::msg::NavSatFix gps_msg{};
            gps_msg.latitude = gps_data_.gps_[gps_ind].lat_;
            gps_msg.longitude = gps_data_.gps_[gps_ind].lon_;
            gps_msg.altitude = 0.0;
            gps_msg.header.frame_id = "gps";
            gps_msg.header.stamp.sec = gps_timestamp_sec;
            gps_msg.header.stamp.nanosec = gps_timestamp_nanosec;

            writer_->write(gps_msg, "/fix", rclcpp::Time{gps_timestamp});

            ++gps_ind;

            if (gps_ind < gps_data_.gps_.size()) {
              gps_timestamp =
                  start_timestamp + gps_data_.gps_[gps_ind].timestamp_;
            } else {
              break;
            }
          }
        }

        const int32_t curr_timestamp_sec{
            static_cast<int32_t>(curr_timestamp / 1'000'000'000)};

        const uint32_t curr_timestamp_nanosec{static_cast<uint32_t>(
            curr_timestamp - curr_timestamp_sec * 1'000'000'000)};

        if (set_.compressed_) {
          sensor_msgs::msg::CompressedImage compressed_img;

          compressed_img.header.frame_id = "cam0";
          compressed_img.header.stamp.sec = curr_timestamp_sec;
          compressed_img.header.stamp.nanosec = curr_timestamp_nanosec;

          compressed_img.format = "jpeg";
          cv::imencode(".jpeg", img, compressed_img.data,
                       {cv::IMWRITE_JPEG_QUALITY, set_.jpeq_quality_});

          writer_->write(compressed_img, "/camera/image_raw/compressed",
                         rclcpp::Time{curr_timestamp});
        } else {
          sensor_msgs::msg::Image raw_img;

          raw_img.header.frame_id = "cam0";
          raw_img.header.stamp.sec = curr_timestamp_sec;
          raw_img.header.stamp.nanosec = curr_timestamp_nanosec;

          raw_img.height = img.rows;
          raw_img.width = img.cols;
          raw_img.encoding = "bgr8";
          raw_img.is_bigendian = false;
          raw_img.step = img.step;
          raw_img.data.assign(img.data,
                              img.data + img.cols * img.rows * img.channels());

          writer_->write(raw_img, "/camera/image_raw",
                         rclcpp::Time{curr_timestamp});
        }
      }

      ++frame_ind;
      bar.advance("frame");
    }

    bar.done();

    if (writer_ and not gps_data_.gps_.empty()) {

      while (gps_ind < gps_data_.gps_.size()) {

        const int64_t gps_timestamp{start_timestamp +
                                    gps_data_.gps_[gps_ind].timestamp_};

        const int32_t gps_timestamp_sec{
            static_cast<int32_t>(gps_timestamp / 1'000'000'000)};

        const uint32_t gps_timestamp_nanosec{static_cast<uint32_t>(
            gps_timestamp - gps_timestamp_sec * 1'000'000'000)};

        sensor_msgs::msg::NavSatFix gps_msg{};
        gps_msg.latitude = gps_data_.gps_[gps_ind].lat_;
        gps_msg.longitude = gps_data_.gps_[gps_ind].lon_;
        gps_msg.altitude = 0.0;
        gps_msg.header.frame_id = "gps";
        gps_msg.header.stamp.sec = gps_timestamp_sec;
        gps_msg.header.stamp.nanosec = gps_timestamp_nanosec;

        writer_->write(gps_msg, "/fix", rclcpp::Time{gps_timestamp});

        ++gps_ind;
      }
    }
  }

private:
  VideoProcessSettings set_;
  std::unique_ptr<cv::VideoCapture> cap_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  GPSData gps_data_;
};

int main(int argc, char const *const *argv) {

  try {
    CLI::App app{"Convert video to ROS bag"};

    VideoProcessSettings set{};

    app.add_option("-i, --input", set.input_video_, "Specify input video path")
        ->required()
        ->check(CLI::ExistingFile);

    app.add_option("-g, --gpx", set.gpx_path_, "Specify path to gpx file")
        ->check(CLI::ExistingFile);

    app.add_option("-o, --output", set.output_bag_,
                   "Specify output ros bag path");

    app.add_option("-r, --resize", set.resize_)->default_val(1.0f);

    app.add_flag("-e, --extract", set.extract_images_, "Extract images")
        ->default_val(false);

    app.add_flag("--compressed", set.compressed_, "Extract images")
        ->default_val(false);

    app.add_option("--quality", set.jpeq_quality_, "JPEG quality")
        ->check(CLI::Range{0, 100})
        ->default_val(70);

    CLI11_PARSE(app, argc, argv);

    if (std::filesystem::exists(set.output_bag_)) {
      fmt::print(fmt::fg(fmt::color::yellow_green),
                 "Path '{}' already exists. Remove? [y/n] ", set.output_bag_);

      char ans{};
      std::cin >> ans;

      if (ans == 'y' or ans == 'Y') {
        std::filesystem::remove_all(set.output_bag_);
      } else {
        return EXIT_FAILURE;
      }
    }

    VideoProcessor video_process{set};
    video_process.process();

  } catch (const std::exception &ex) {
    fmt::print(fmt::fg(fmt::color::red), "{}\n", ex.what());
  }

  fmt::print(fmt::fg(fmt::color::aquamarine), "Done\n");
  return EXIT_SUCCESS;
}