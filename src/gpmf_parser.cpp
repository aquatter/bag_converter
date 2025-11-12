// clang-format off
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <GPMF_common.h>
#include <GPMF_mp4reader.h>
#include <GPMF_parser.h>
#include <GPMF_utils.h>
#include <gpmf_parser.hpp>
#include <limits>
#include <memory>
#include <stdexcept>
// clang-format on

#include <fmt/color.h>
#include <fmt/format.h>
#include <gpmf_frame.hpp>
#include <queue>
#include <range/v3/view/iota.hpp>
#include <string_view>
#include <unordered_map>
#include <vector>

#include <progress_bar.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/header.hpp>

using ranges::views::ints;

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

    if (data_frame_rate_["ACCL"] != data_frame_rate_["GYRO"]) {
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
  impl(const GPMFParserSettings &set) : set_{set} {

    if (not set_.no_images_) {
      chunks_.emplace_back(std::make_unique<SHUTChunk>(SHUTChunkSettings{
          .resize_ = set_.resize_,
          .output_path_ = set_.output_path_,
          .jpeg_quality_ = set_.jpeg_quality_,
          .extract_images_ = set_.extract_images_,
          .compress_ = set_.compress_,

      }));
    }

    chunks_.emplace_back(std::make_unique<GPSChunk>());
    chunks_.emplace_back(std::make_unique<IMUChunk>());
  }

  void parse() {

    for (auto &&path_to_mp4 : set_.paths_to_mp4_) {
      MP4Source mp4{path_to_mp4};

      for (auto &&chunk : chunks_) {
        chunk->reset();
        chunk->open_mp4(path_to_mp4);
        chunk->set_frame_rate(mp4.frame_rate(chunk->four_cc()[0].data()));
      }

      mp4.parse(chunks_);
    }
  }

  void write_bag(const std::string_view path) const {

    rosbag2_cpp::Writer writer{};
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = path.data();

    writer.open(storage_options);

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

      ptr->write(writer);

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

  GPMFParserSettings set_;
  std::vector<std::unique_ptr<GPMFChunkBase>> chunks_;
};

GPMFParser::GPMFParser(const GPMFParserSettings &set)
    : pimpl_{std::make_unique<impl>(set)} {}

void GPMFParser::parse() { pimpl_->parse(); }

void GPMFParser::write_bag(const std::string_view path) const {
  pimpl_->write_bag(path);
}

GPMFParser::~GPMFParser() = default;