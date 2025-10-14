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
#include <memory>
#include <stdexcept>
// clang-format on

#include <fmt/color.h>
#include <fmt/format.h>
#include <gpmf_frame.hpp>
#include <queue>
#include <range/v3/view/iota.hpp>
#include <string_view>
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

struct GPMFParser::impl {
  impl(const GPMFParserSettings &set) : set_{set} {

    mp4handle_ = OpenMP4Source(const_cast<char *>(set_.path_to_mp4_.data()),
                               MOV_GPMF_TRAK_TYPE, MOV_GPMF_TRAK_SUBTYPE, 0);

    if (mp4handle_ == 0) {
      throw std::runtime_error{
          fmt::format("error: {} is an invalid MP4/MOV or it has no GPMF data",
                      set_.path_to_mp4_.data())};
    }

    fmt::print("\e[38;2;154;205;50mSuccessfully opened file:\e[0m "
               "\e[38;2;255;127;80m{}\e[0m\n",
               set_.path_to_mp4_.data());

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

    chunks_.emplace_back(std::make_unique<SHUTChunk>(SHUTChunkSettings{
        .frame_rate_ =
            GetGPMFSampleRate(cbobject, STR2FOURCC("SHUT"), STR2FOURCC("SHUT"),
                              GPMF_SAMPLE_RATE_PRECISE, nullptr, nullptr),
        .resize_ = 1.0,
        .path_to_mp4_ = set_.path_to_mp4_,
        .output_path_ = set_.output_path_,
        .jpeg_quality_ = set_.jpeg_quality_,
        .extract_images_ = set_.extract_images_,
        .compress_ = set_.compress_,

    }));

    chunks_.emplace_back(std::make_unique<GPS5Chunk>(
        GetGPMFSampleRate(cbobject, STR2FOURCC("GPS5"), STR2FOURCC("SHUT"),
                          GPMF_SAMPLE_RATE_PRECISE, nullptr, nullptr)));

    chunks_.emplace_back(std::make_unique<IMUChunk>(
        GetGPMFSampleRate(cbobject, STR2FOURCC("ACCL"), STR2FOURCC("SHUT"),
                          GPMF_SAMPLE_RATE_PRECISE, nullptr, nullptr),
        GetGPMFSampleRate(cbobject, STR2FOURCC("GYRO"), STR2FOURCC("SHUT"),
                          GPMF_SAMPLE_RATE_PRECISE, nullptr, nullptr)));
  }

  void parse() {
    const auto num_payloads{GetNumberPayloads(mp4handle_)};

    for (auto &&payload_index : ints(0u, num_payloads)) {

      Payload payload{mp4handle_, payload_index};

      for (auto &&chunk : chunks_) {

        for (const auto four_cc : chunk->four_cc()) {
          payload.parse(*chunk, four_cc);
        }
      }
    }

    for (auto &&chunk : chunks_) {
      chunk->create_measurements();
    }

    fmt::print(fmt::fg(fmt::color::yellow_green),
               "GPMF data successfully parsed\n");
  }

  void write_bag() {

    fmt::print(fmt::fg(fmt::color::yellow_green), "Creating bag...\n");

    using queue_type = std::pair<int64_t, GPMFChunkBase *>;

    std::priority_queue<queue_type, std::vector<queue_type>,
                        decltype([](const queue_type &a, const queue_type &b) {
                          return a.first > b.first;
                        })>
        q;

    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = set_.output_path_;

    rosbag2_cpp::Writer writer;
    writer.open(storage_options);

    for (auto &&chunk : chunks_) {
      if (chunk->timestamp().has_value()) {
        q.emplace(chunk->timestamp().value(), chunk.get());
      }
    }

    ProgressBar bar{
        std::vector{ProgressBar::ProgressInfo{.message_count_ = num_frames_,
                                              .processed_count_ = 0,
                                              .topic_name_ = "frame",
                                              .ind_ = 0}}};

    bar.draw();

    while (not q.empty()) {
      auto ptr{q.top().second};
      q.pop();

      ptr->write(writer);
      if (ptr->timestamp().has_value()) {
        q.emplace(ptr->timestamp().value(), ptr);
      }

      if (ptr->whoami() == ChunkType::Camera) {
        bar.advance("frame");
      }
    }

    bar.done();
  }

  ~impl() { CloseSource(mp4handle_); }

  GPMFParserSettings set_;
  size_t mp4handle_;
  uint32_t num_frames_;
  std::vector<std::unique_ptr<GPMFChunkBase>> chunks_;
};

GPMFParser::GPMFParser(const GPMFParserSettings &set)
    : pimpl_{std::make_unique<impl>(set)} {}

void GPMFParser::parse() { pimpl_->parse(); }
void GPMFParser::write_bag() { pimpl_->write_bag(); }

GPMFParser::~GPMFParser() = default;