// clang-format off
#include <algorithm>
#include <boost/algorithm/string/case_conv.hpp>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <GPMF_common.h>
#include <GPMF_mp4reader.h>
#include <GPMF_parser.h>
#include <GPMF_utils.h>
#include <ctime>
#include <filesystem>
#include <functional>
#include <gpmf_parser.hpp>
#include <limits>
#include <memory>
#include <nlohmann/json_fwd.hpp>
#include <optional>
#include <stdexcept>
// clang-format on

#include <GeographicLib/LocalCartesian.hpp>
#include <boost/algorithm/string.hpp>
#include <chrono>
#include <fmt/chrono.h>
#include <fmt/color.h>
#include <fmt/format.h>
#include <fstream>
#include <gpmf_frame.hpp>
#include <nlohmann/json.hpp>
#include <progress_bar.hpp>
#include <queue>
#include <range/v3/action/push_back.hpp>
#include <range/v3/algorithm/sort.hpp>
#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/view/zip.hpp>
#include <regex>
#include <rosbag2_cpp/writer.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>
#include <string_view>
#include <tinyxml2.h>
#include <unordered_map>
#include <vector>

using ranges::views::enumerate;
using ranges::views::ints;
using ranges::views::transform;
using ranges::views::zip;

std::optional<std::pair<int, int>>
parce_record_part(const std::string_view str) {

  std::regex re{R"(gx(\d{2})(\d{4}).mp4)"};

  int part_num{0};
  int record_num{0};

  std::cmatch match{};
  if (std::regex_search(str.data(), match, re)) {
    const auto part_str{match[1].str()};
    const auto record_str{match[2].str()};

    re = R"(0*(\d+))";

    if (std::regex_search(part_str.data(), match, re)) {
      part_num = std::stoi(match[1].str());
    } else {
      return {};
    }

    if (std::regex_search(record_str.data(), match, re)) {
      record_num = std::stoi(match[1].str());
    } else {
      return {};
    }

    return std::pair{record_num, part_num};
  }

  return {};
}

struct Payload {
  Payload(size_t mp4handle, uint32_t payload_index) : mp4handle_{mp4handle} {
    payloadsize_ = GetPayloadSize(mp4handle, payload_index);
    payloadres_ = GetPayloadResource(mp4handle, payloadres_, payloadsize_);
    payload_ = GetPayload(mp4handle, payloadres_, payload_index);

    if (payload_ == nullptr) {
      throw std::runtime_error{
          fmt::format("unable to load payload, index: {}", payload_index)};
    }

    if (GPMF_OK != GPMF_Init(&metadata_stream_, payload_, payloadsize_)) {
      throw std::runtime_error{
          fmt::format("unable to initialize stream, index: {}", payload_index)};
    }
  }

  void parse(GPMFChunkBase &chunk, const std::string_view four_cc) {

    if (GPMF_OK == GPMF_FindNext(&metadata_stream_, STR2FOURCC(four_cc),
                                 static_cast<GPMF_LEVELS>(GPMF_RECURSE_LEVELS |
                                                          GPMF_TOLERANT))) {

      uint64_t start_timestamp{0};
      {
        GPMF_stream find_stream;
        GPMF_CopyState(&metadata_stream_, &find_stream);

        if (GPMF_OK == GPMF_FindPrev(&find_stream, GPMF_KEY_TIME_STAMP,
                                     GPMF_CURRENT_LEVEL)) {
          start_timestamp =
              BYTESWAP64(*static_cast<uint64_t *>(GPMF_RawData(&find_stream)));
        }
      }

      const uint32_t samples{GPMF_Repeat(&metadata_stream_)};
      const uint32_t elements{GPMF_ElementsInStruct(&metadata_stream_)};

      if (samples != 0) {
        std::vector<double> buf;
        buf.resize(samples * elements);

        if (GPMF_OK == GPMF_ScaledData(&metadata_stream_, buf.data(),
                                       buf.size() * sizeof(double), 0, samples,
                                       GPMF_TYPE_DOUBLE)) {

          chunk.add(four_cc, start_timestamp, buf);
        }
      }
    }

    GPMF_ResetState(&metadata_stream_);
  }

  ~Payload() {
    if (payloadres_ != 0) {
      FreePayloadResource(mp4handle_, payloadres_);
    }

    if (metadata_stream_.cbhandle != 0) {
      GPMF_Free(&metadata_stream_);
    }
  }
  size_t mp4handle_;
  uint32_t payloadsize_{0};
  size_t payloadres_{0};
  uint32_t *payload_{nullptr};
  GPMF_stream metadata_stream_;
};

struct MP4Source {
  MP4Source(const std::string_view path_to_mp4) {
    mp4handle_ = OpenMP4Source(const_cast<char *>(path_to_mp4.data()),
                               MOV_GPMF_TRAK_TYPE, MOV_GPMF_TRAK_SUBTYPE, 0);

    if (mp4handle_ == 0) {
      throw std::runtime_error{
          fmt::format("error: {} is an invalid MP4/MOV or it has no GPMF data",
                      path_to_mp4.data())};
    }

    fmt::print("\e[38;2;154;205;50mSuccessfully opened file:\e[0m "
               "\e[38;2;255;127;80m{}\e[0m\n",
               path_to_mp4.data());

    uint32_t fr_num{0}, fr_dem{0};

    num_frames_ = GetVideoFrameRateAndCount(mp4handle_, &fr_num, &fr_dem);

    fmt::print("\e[38;2;154;205;50mVideo framerate:\e[0m "
               "\e[38;2;255;127;80m{:.3f}\e[0m "
               "\e[38;2;154;205;50mwith\e[0m \e[38;2;255;127;80m{}\e[0m "
               "\e[38;2;154;205;50mframes\e[0m\n",
               static_cast<float>(fr_num) / static_cast<float>(fr_dem),
               num_frames_);

    mp4callbacks cbobject;
    cbobject.mp4handle = mp4handle_;
    cbobject.cbGetNumberPayloads = GetNumberPayloads;
    cbobject.cbGetPayload = GetPayload;
    cbobject.cbGetPayloadSize = GetPayloadSize;
    cbobject.cbGetPayloadResource = GetPayloadResource;
    cbobject.cbGetPayloadTime = GetPayloadTime;
    cbobject.cbFreePayloadResource = FreePayloadResource;
    cbobject.cbGetEditListOffsetRationalTime = GetEditListOffsetRationalTime;

    data_frame_rate_["SHUT"] =
        GetGPMFSampleRate(cbobject, STR2FOURCC("SHUT"), STR2FOURCC("SHUT"),
                          GPMF_SAMPLE_RATE_PRECISE, nullptr, nullptr);

    data_frame_rate_["ACCL"] =
        GetGPMFSampleRate(cbobject, STR2FOURCC("ACCL"), STR2FOURCC("SHUT"),
                          GPMF_SAMPLE_RATE_PRECISE, nullptr, nullptr);

    data_frame_rate_["GYRO"] =
        GetGPMFSampleRate(cbobject, STR2FOURCC("GYRO"), STR2FOURCC("SHUT"),
                          GPMF_SAMPLE_RATE_PRECISE, nullptr, nullptr);

    data_frame_rate_["GPS9"] =
        GetGPMFSampleRate(cbobject, STR2FOURCC("GPS9"), STR2FOURCC("SHUT"),
                          GPMF_SAMPLE_RATE_PRECISE, nullptr, nullptr);

    if (static_cast<int>(data_frame_rate_["ACCL"]) !=
        static_cast<int>(data_frame_rate_["GYRO"])) {
      throw std::runtime_error{
          "accelerometer and gyroscope rates are not the same"};
    }
  }

  double frame_rate(std::string fourcc) const {
    if (data_frame_rate_.contains(fourcc)) {
      return data_frame_rate_.at(fourcc);
    }

    return 0.0;
  }

  int duration() const {
    return static_cast<int>(GetDuration(mp4handle_) + 0.5f);
  }

  void parse(std::span<std::unique_ptr<GPMFChunkBase>> chunks) {
    const auto num_payloads{GetNumberPayloads(mp4handle_)};

    for (auto &&payload_index : ints(0u, num_payloads)) {

      Payload payload{mp4handle_, payload_index};

      for (auto &&chunk : chunks) {

        for (const auto four_cc : chunk->four_cc()) {
          payload.parse(*chunk, four_cc);
        }
      }
    }

    for (auto &&chunk : chunks) {
      chunk->create_measurements();
    }

    fmt::print(fmt::fg(fmt::color::yellow_green),
               "GPMF data successfully parsed\n");
  }

  uint32_t num_frames() const noexcept { return num_frames_; }

  ~MP4Source() { CloseSource(mp4handle_); }

  size_t mp4handle_;
  uint32_t num_frames_;
  std::unordered_map<std::string, double> data_frame_rate_;
};

struct GPMFParser::impl {

  struct Record {
    struct Part {
      int id_;
      std::filesystem::path path_;
    };

    int id_;
    std::vector<Part> parts_;
  };

  impl(const GPMFParserSettings &set) : set_{set} {

    for (auto &&[start, end] : set_.gps_exclusion_intervals_) {
      start *= 1'000'000'000l;
      end *= 1'000'000'000l;
    }

    int rec_id{0};

    std::unordered_map<int, Record> rec{};

    for (auto &&input_path : set_.paths_to_mp4_) {
      if (std::filesystem::file_type::directory ==
          std::filesystem::status(input_path).type()) {

        for (auto &&entry : std::filesystem::directory_iterator{input_path}) {

          while (rec.contains(rec_id)) {
            ++rec_id;
          }

          if (entry.is_regular_file() and
              boost::algorithm::to_lower_copy(
                  entry.path().extension().string()) == ".mp4") {

            const auto record_part{
                parce_record_part(boost::algorithm::to_lower_copy(
                    entry.path().filename().string()))};

            if (record_part.has_value()) {
              rec[record_part->first].id_ = record_part->first;
              rec[record_part->first].parts_.push_back(Record::Part{
                  .id_ = record_part->second, .path_ = entry.path()});
            } else {
              rec[rec_id].id_ = rec_id;
              rec[rec_id].parts_.push_back(
                  Record::Part{.id_ = 0, .path_ = entry.path()});
            }
          }
        }
      } else {
        while (rec.contains(rec_id)) {
          ++rec_id;
        }

        if (boost::algorithm::to_lower_copy(
                std::filesystem::path{input_path}.extension().string()) ==
            ".mp4") {

          const auto record_part{
              parce_record_part(boost::algorithm::to_lower_copy(
                  std::filesystem::path{input_path}.filename().string()))};

          if (record_part.has_value()) {
            rec[record_part->first].id_ = record_part->first;
            rec[record_part->first].parts_.push_back(
                Record::Part{.id_ = record_part->second, .path_ = input_path});
          } else {
            rec[rec_id].id_ = rec_id;
            rec[rec_id].parts_.push_back(
                Record::Part{.id_ = 0, .path_ = input_path});
          }
        }
      }
    }

    for (auto &&[id, r] : rec) {
      ranges::sort(r.parts_,
                   [](const auto &a, const auto &b) { return a.id_ < b.id_; });

      records_.push_back(r);
    }

    if (not set_.no_images_) {
      chunks_.emplace_back(std::make_unique<SHUTChunk>(SHUTChunkSettings{
          .resize_ = set_.resize_,
          .output_path_ = set_.output_path_,
          .jpeg_quality_ = set_.jpeg_quality_,
          .extract_images_ = set_.extract_images_,
          .compress_ = set_.compress_,

      }));
    }

    if (not set_.no_gps_) {
      chunks_.emplace_back(
          std::make_unique<GPSChunk>(set_.gps_exclusion_intervals_));
    }
    if (not set_.no_imu_) {
      chunks_.emplace_back(std::make_unique<IMUChunk>());
    }

    start_time_ = 1'000'000'000l * set_.start_time_;
    end_time_ = 1'000'000'000l * set_.end_time_;

    start_time_ = start_time_ < 0 ? 0 : start_time_;
    end_time_ = end_time_ < 0 ? std::numeric_limits<int64_t>::max() : end_time_;
  }

  void parse() {

    int total_duration{0};

    for (auto &&record : records_) {
      lla_.clear();
      create_bag(fmt::format("{}record_{}", set_.output_path_, record.id_));

      int track_duration{0};

      for (auto &&part : record.parts_) {

        MP4Source mp4{part.path_.string()};

        for (auto &&chunk : chunks_) {
          chunk->reset();
          chunk->open_mp4(part.path_.string());
          chunk->set_frame_rate(mp4.frame_rate(chunk->four_cc()[0].data()));
        }

        track_duration += mp4.duration();
        mp4.parse(chunks_);

        if (set_.save_geojson_) {

          for (auto &&chunk : chunks_) {
            chunk->visit([this](const GPMFChunkBase *chunk) {
              if (chunk->whoami() != ChunkType::GPS) {
                return;
              }

              const auto &m{
                  static_cast<const GPSChunk *>(chunk)->measurements_};

              lla_ = std::move(lla_) |
                     ranges::actions::push_back(
                         m | transform([](auto &&val) { return val.lla_; }));

              gps_fix_ =
                  std::move(gps_fix_) |
                  ranges::actions::push_back(
                      m | transform([](auto &&val) { return val.fix_; }));

              gps_timestamps_ =
                  std::move(gps_timestamps_) |
                  ranges::actions::push_back(
                      m | transform([](auto &&val) { return val.timestamp_; }));
            });
          }
        }

        if (set_.callback_) {
          for (auto &&chunk : chunks_) {
            chunk->visit(set_.callback_);
          }
        }

        write_bag();
      }

      total_duration += track_duration;

      fmt::print("\e[38;2;154;205;50mTrack duration: \e[0m"
                 "\e[38;2;255;127;80m{}\e[0m\e[38;2;154;205;50m hours \e[0m"
                 "\e[38;2;255;127;80m{}\e[0m\e[38;2;154;205;50m minuts \e[0m"
                 "\e[38;2;255;127;80m{}\e[0m\e[38;2;154;205;50m "
                 "seconds\e[0m\n",
                 track_duration / 3600, (track_duration % 3600) / 60,
                 (track_duration % 3600) % 60);

      write_geojson(
          fmt::format("record_{}", record.id_),
          fmt::format("{}record_{}.geojson", set_.output_path_, record.id_));

      write_gpx(fmt::format("{}record_{}.gpx", set_.output_path_, record.id_));

      track_length();

      close_bag();
    }

    if (set_.save_geojson_) {
      fmt::print(
          "\e[38;2;154;205;50mTotal GPS track length: \e[0m"
          "\e[38;2;255;127;80m{:.3f}\e[0m\e[38;2;154;205;50m kilometers\e[0m\n",
          1.0e-3 * track_length_);
    }

    fmt::print("\e[38;2;154;205;50mTotal duration: \e[0m"
               "\e[38;2;255;127;80m{}\e[0m\e[38;2;154;205;50m hours \e[0m"
               "\e[38;2;255;127;80m{}\e[0m\e[38;2;154;205;50m minuts \e[0m"
               "\e[38;2;255;127;80m{}\e[0m\e[38;2;154;205;50m "
               "seconds\e[0m\n",
               total_duration / 3600, (total_duration % 3600) / 60,
               (total_duration % 3600) % 60);
  }

  void close_bag() {
    if (not set_.save_bag_) {
      return;
    }

    writer_.close();
  }

  void write_bag() {

    if (not set_.save_bag_) {
      return;
    }

    using queue_type = std::pair<int64_t, GPMFChunkBase *>;

    std::priority_queue<queue_type, std::vector<queue_type>,
                        decltype([](const queue_type &a, const queue_type &b) {
                          return a.first > b.first;
                        })>
        q;

    int64_t min_timestamp{std::numeric_limits<int64_t>::max()};
    int64_t max_timestamp{std::numeric_limits<int64_t>::min()};

    for (auto &&chunk : chunks_) {
      if (chunk->timestamp().has_value()) {
        q.emplace(chunk->timestamp().value(), chunk.get());
      }

      const auto [min_stamp, max_stamp] = chunk->timestamp_range();

      min_timestamp = std::min(min_timestamp, min_stamp);
      max_timestamp = std::max(max_timestamp, max_stamp);
    }

    ProgressBar bar{std::vector{ProgressBar::ProgressInfo{
        .message_count_ = static_cast<size_t>(
            1.0e-9 * static_cast<double>(max_timestamp - min_timestamp + 1) +
            0.5),
        .processed_count_ = 0,
        .topic_name_ = "sec",
        .ind_ = 0}}};

    bar.draw();

    int64_t curr_timestamp{min_timestamp};

    while (not q.empty()) {
      auto ptr{q.top().second};
      q.pop();

      if (ptr->timestamp().value() < start_time_) {
        ptr->increment();
      } else if (ptr->timestamp().value() > end_time_) {
        continue;
      } else {

        ptr->write(writer_);
      }

      if (ptr->timestamp().has_value()) {
        q.emplace(ptr->timestamp().value(), ptr);

        const int64_t timestamp_diff{ptr->timestamp().value() - curr_timestamp};

        if (timestamp_diff >= 1'000'000'000) {
          bar.progress(
              "sec", static_cast<size_t>(
                         1.0e-9 * static_cast<double>(ptr->timestamp().value() -
                                                      min_timestamp + 1) +
                         0.5));
          curr_timestamp = ptr->timestamp().value();
        }
      }
    }

    bar.done();
  }

  void write_gpx(const std::string_view path) {
    if (not set_.save_geojson_) {
      return;
    }

    tinyxml2::XMLDocument doc{};

    auto decl{doc.NewDeclaration(
        "xml version='1.0' encoding='UTF-8' standalone='no'")};

    doc.InsertFirstChild(decl);
    auto gpx{doc.NewElement("gpx")};
    doc.InsertEndChild(gpx);

    auto trk{doc.NewElement("trk")};
    gpx->InsertEndChild(trk);

    auto trkseg{doc.NewElement("trkseg")};
    trk->InsertEndChild(trkseg);

    gpx->SetAttribute("xmlns", "http://www.topografix.com/GPX/1/1");
    gpx->SetAttribute("creator", "lomakin");
    gpx->SetAttribute("version", "1.1");
    gpx->SetAttribute("xmlns:xsi", "http://www.w3.org/2001/XMLSchema-instance");
    gpx->SetAttribute("xsi:schemaLocation",
                      "http://www.topografix.com/GPX/1/1 "
                      "http://www.topografix.com/GPX/1/1/gpx.xsd");

    auto name{doc.NewElement("name")};
    name->SetText(std::filesystem::path{
        set_.output_path_.substr(0, set_.output_path_.size() - 1)}
                      .filename()
                      .string()
                      .c_str());

    trk->InsertEndChild(name);

    for (auto &&[gps, fix, timestamp] : zip(lla_, gps_fix_, gps_timestamps_)) {
      if (fix != 3.0) {
        continue;
      }

      if (exclude(timestamp)) {
        continue;
      }

      auto trkpt{doc.NewElement("trkpt")};
      trkpt->SetAttribute("lat", gps.x());
      trkpt->SetAttribute("lon", gps.y());

      auto time{doc.NewElement("time")};

      time->SetText(
          fmt::format("{:%Y-%m-%dT%H:%M:%S}Z", std::chrono::system_clock::now())
              .c_str());

      trkpt->InsertEndChild(time);
      trkseg->InsertEndChild(trkpt);
    }

    doc.SaveFile(path.data());
  }

  void track_length() {
    if (not set_.save_geojson_) {
      return;
    }

    double current_length{0.0};

    if (proj_.has_value() == false && lla_.size() > 0) {

      for (auto &&[lla, fix, timestamp] :
           zip(lla_, gps_fix_, gps_timestamps_)) {

        if (fix != 3.0) {
          continue;
        }

        if (exclude(timestamp)) {
          continue;
        }

        proj_ = GeographicLib::LocalCartesian(lla.x(), lla.y(), lla.z());
        track_length_ = 0.0;
        break;
      }

      if (not proj_.has_value()) {
        fmt::print(fmt::fg(fmt::color::red),
                   "unable to initialize a local CS\n");
        return;
      }
    }

    Eigen::Vector2d prev_coord{Eigen::Vector2d::Zero()};
    int64_t prev_timestamp{0};
    size_t ind{0};

    {
      double z{0.0};

      for (auto &&[lla, fix, timestamp] :
           zip(lla_, gps_fix_, gps_timestamps_)) {

        if (fix != 3.0) {
          ++ind;
          continue;
        }

        if (exclude(timestamp)) {
          ++ind;
          continue;
        }

        proj_->Forward(lla.x(), lla.y(), lla.z(), prev_coord.x(),
                       prev_coord.y(), z);
        prev_timestamp = timestamp;
        break;

        ++ind;
      }

      ++ind;
    }

    while (ind < lla_.size()) {
      if (gps_fix_[ind] != 3.0) {
        ++ind;
        continue;
      }

      if (exclude(gps_timestamps_[ind])) {
        ++ind;
        continue;
      }

      double z{0.0};
      Eigen::Vector2d enu{};
      proj_->Forward(lla_[ind].x(), lla_[ind].y(), lla_[ind].z(), enu.x(),
                     enu.y(), z);

      if (gps_timestamps_[ind] - prev_timestamp > 200'000'000) {
        prev_coord = enu;
      }

      if ((enu - prev_coord).norm() > 5.0) {
        current_length += (enu - prev_coord).norm();
        prev_coord = enu;
      }

      prev_timestamp = gps_timestamps_[ind];
      ++ind;
    }

    track_length_ += current_length;

    fmt::print(
        "\e[38;2;154;205;50mGPS track length: \e[0m"
        "\e[38;2;255;127;80m{:.3f}\e[0m\e[38;2;154;205;50m kilometers\e[0m\n",
        1.0e-3 * current_length);
  }

  void create_bag(const std::string_view name) {
    if (not set_.save_bag_) {
      return;
    }

    if (std::filesystem::exists(name.data())) {
      std::filesystem::remove_all(name.data());
    }

    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = name.data();
    writer_.open(storage_options);
  }

  void write_geojson(const std::string_view name, const std::string_view path) {
    if (not set_.save_geojson_) {
      return;
    }

    nlohmann::json j{};
    j["type"] = "FeatureCollection";
    j["name"] = name.data();

    nlohmann::json gps_track{};
    gps_track["type"] = "Feature";
    gps_track["geometry"]["type"] = "LineString";

    for (auto &&[i, val] : zip(lla_, gps_timestamps_, gps_fix_) | enumerate) {

      auto &&[gps, timestamp, fix] = val;

      if (fix != 3.0) {
        continue;
      }

      if (exclude(timestamp)) {
        continue;
      }

      nlohmann::json feature{};

      feature["type"] = "Feature";
      feature["id"] = i;
      feature["geometry"]["type"] = "Point";
      feature["geometry"]["coordinates"] = {gps.y(), gps.x()};
      feature["properties"]["timestamp"] = timestamp;
      gps_track["geometry"]["coordinates"].push_back({gps.y(), gps.x()});
      j["features"].push_back(feature);
    }

    j["features"].push_back(gps_track);

    std::ofstream f{path.data()};
    f << j.dump(4);
  }

  bool exclude(int64_t timestamp) {
    for (auto &&[start, end] : set_.gps_exclusion_intervals_) {
      if (timestamp >= start and timestamp <= end) {
        return true;
      }
    }

    return false;
  }

  GPMFParserSettings set_;
  rosbag2_cpp::Writer writer_;
  std::vector<std::unique_ptr<GPMFChunkBase>> chunks_;
  int64_t start_time_;
  int64_t end_time_;
  std::vector<Eigen::Vector3d> lla_;
  std::vector<double> gps_fix_;
  std::vector<int64_t> gps_timestamps_;
  std::vector<Record> records_;
  std::optional<GeographicLib::LocalCartesian> proj_;
  double track_length_{0.0};
  float track_duration_{0.0f};
};

GPMFParser::GPMFParser(const GPMFParserSettings &set)
    : pimpl_{std::make_unique<impl>(set)} {}

void GPMFParser::parse() { pimpl_->parse(); }

GPMFParser::~GPMFParser() = default;