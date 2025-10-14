
// clang-format off
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <GPMF_mp4reader.h>
// clang-format on
#include <fmt/format.h>
#include <gpmf_frame.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <range/v3/numeric/accumulate.hpp>
#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/sliding.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/view/zip.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <stdexcept>

using ranges::views::enumerate;
using ranges::views::ints;
using ranges::views::sliding;
using ranges::views::transform;
using ranges::views::zip;

SHUTChunk::SHUTChunk(const SHUTChunkSettings &set)
    : set_{set}, cap_{set_.path_to_mp4_} {

  if (not cap_.isOpened()) {
    throw std::runtime_error{
        fmt::format("unable to open video: {}", set_.path_to_mp4_.data())};
  }
}

SHUTChunk::~SHUTChunk() { cap_.release(); }

void SHUTChunk::add(const std::string_view, uint64_t timestamp,
                    std::span<const double> vec) {
  data_.emplace_back(timestamp * 1'000, vec.size());
}

void SHUTChunk::create_measurements() {

  const auto total_num{ranges::accumulate(
      data_ | transform([](const Data &d) { return d.num_frames_; }), 0ul)};

  const auto num_frames{
      static_cast<uint64_t>(cap_.get(cv::CAP_PROP_FRAME_COUNT))};

  // if (total_num != num_frames) {
  //   throw std::runtime_error{
  //       "number of frames in GPMF and video are not equal"};
  // }

  measurements_.reserve(total_num);

  for (auto &&d : data_ | sliding(2)) {
    const int64_t delta{d[1].timestamp_ - d[0].timestamp_};
    const double delta_frame{static_cast<double>(delta) /
                             static_cast<double>(d[0].num_frames_)};

    for (auto &&k : ints(0ul, d[0].num_frames_)) {
      measurements_.push_back(d[0].timestamp_ +
                              static_cast<int64_t>(k * delta_frame + 0.5));
    }
  }

  const double delta_frame{1'000'000'000.0 / set_.frame_rate_};

  for (auto &&k : ints(0ul, data_.back().num_frames_)) {
    measurements_.push_back(data_.back().timestamp_ +
                            static_cast<int64_t>(k * delta_frame + 0.5));
  }
}

void SHUTChunk::write(rosbag2_cpp::Writer &writer) {
  if (index_ >= measurements_.size()) {
    return;
  }

  cv::Mat_<cv::Vec3b> img{};
  cap_ >> img;

  if (img.empty()) {
    ++index_;
    return;
  }

  if (set_.resize_ != 1.0) {
    cv::resize(img, img, {}, set_.resize_, set_.resize_);
  }

  if (set_.extract_images_) {
    cv::imwrite(
        fmt::format("{}/{}.png", set_.output_path_, measurements_[index_]),
        img);
  } else {

    const int32_t timestamp_sec{
        static_cast<int32_t>(measurements_[index_] / 1'000'000'000)};

    const uint32_t timestamp_nanosec{static_cast<uint32_t>(
        measurements_[index_] - timestamp_sec * 1'000'000'000)};

    if (set_.compress_) {
      sensor_msgs::msg::CompressedImage compressed_img;
      compressed_img.header.frame_id = "cam0";
      compressed_img.header.stamp.sec = timestamp_sec;
      compressed_img.header.stamp.nanosec = timestamp_nanosec;
      compressed_img.format = "jpeg";
      cv::imencode(".jpeg", img, compressed_img.data,
                   {cv::IMWRITE_JPEG_QUALITY, set_.jpeg_quality_});

      writer.write(compressed_img, "/camera/image_raw/compressed",
                   rclcpp::Time{measurements_[index_]});
    } else {
      sensor_msgs::msg::Image raw_img;

      raw_img.header.frame_id = "cam0";
      raw_img.header.stamp.sec = timestamp_sec;
      raw_img.header.stamp.nanosec = timestamp_nanosec;

      raw_img.height = img.rows;
      raw_img.width = img.cols;
      raw_img.encoding = "bgr8";
      raw_img.is_bigendian = false;
      raw_img.step = img.step;
      raw_img.data.assign(img.data,
                          img.data + img.cols * img.rows * img.channels());

      writer.write(raw_img, "/camera/image_raw",
                   rclcpp::Time{measurements_[index_]});
    }
  }

  ++index_;
}

void IMUChunk::add(const std::string_view fourcc_str, uint64_t timestamp,
                   std::span<const double> vec) {

  Data d{};
  d.timestamp_ = timestamp * 1'000;
  const auto num_elements{vec.size() / num_components_};

  d.val_.resize(num_elements);

  for (auto &&i : ints(0ul, num_elements)) {
    d.val_[i].x() = vec[i * num_components_ + 1];
    d.val_[i].y() = vec[i * num_components_ + 2];
    d.val_[i].z() = vec[i * num_components_];
  }

  if (fourcc_str == "ACCL") {
    accl_data_.push_back(std::move(d));
  } else if (fourcc_str == "GYRO") {
    gyro_data_.push_back(std::move(d));
  }
}

void IMUChunk::create_measurements() {
  const auto total_num_accl{ranges::accumulate(
      accl_data_ | transform([](const Data &d) { return d.val_.size(); }),
      0ul)};

  const auto total_num_gyro{ranges::accumulate(
      gyro_data_ | transform([](const Data &d) { return d.val_.size(); }),
      0ul)};

  if (total_num_accl != total_num_gyro) {
    throw std::runtime_error{
        "accelerometer and gyroscope are not synchronized"};
  }

  measurements_.reserve(total_num_accl);

  for (auto &&i : ints(0ul, accl_data_.size() - 1)) {
    const int64_t delta{accl_data_[i + 1].timestamp_ -
                        accl_data_[i].timestamp_};

    if (accl_data_[i + 1].timestamp_ != gyro_data_[i + 1].timestamp_ or
        accl_data_[i].timestamp_ != gyro_data_[i].timestamp_) {
      throw std::runtime_error{
          "accelerometer and gyroscope are not synchronized"};
    }

    if (accl_data_[i].val_.size() != gyro_data_[i].val_.size()) {
      throw std::runtime_error{
          "different data size in the accelerometer and gyroscope"};
    }

    const double delta_frame{static_cast<double>(delta) /
                             static_cast<double>(accl_data_[i].val_.size())};

    for (auto &&[k, imu_data] :
         enumerate(zip(accl_data_[i].val_, gyro_data_[i].val_))) {

      auto &&[accl, gyro] = imu_data;

      measurements_.push_back(
          Measurement{.timestamp_ = accl_data_[i].timestamp_ +
                                    static_cast<int64_t>(k * delta_frame + 0.5),
                      .accl_ = accl,
                      .gyro_ = gyro});
    }
  }

  const double delta_frame{1'000'000'000.0 / accl_rate_};

  for (auto &&[k, imu_data] :
       enumerate(zip(accl_data_.back().val_, gyro_data_.back().val_))) {

    auto &&[accl, gyro] = imu_data;

    measurements_.push_back(
        Measurement{.timestamp_ = accl_data_.back().timestamp_ +
                                  static_cast<int64_t>(k * delta_frame + 0.5),
                    .accl_ = accl,
                    .gyro_ = gyro});
  }
}

void IMUChunk::write(rosbag2_cpp::Writer &writer) {

  if (index_ >= measurements_.size()) {
    return;
  }

  const int32_t timestamp_sec{
      static_cast<int32_t>(measurements_[index_].timestamp_ / 1'000'000'000)};

  const uint32_t timestamp_nanosec{static_cast<uint32_t>(
      measurements_[index_].timestamp_ - timestamp_sec * 1'000'000'000)};

  sensor_msgs::msg::Imu imu_msg{};

  imu_msg.linear_acceleration.x = measurements_[index_].accl_.x();
  imu_msg.linear_acceleration.y = measurements_[index_].accl_.y();
  imu_msg.linear_acceleration.z = measurements_[index_].accl_.z();
  imu_msg.angular_velocity.x = measurements_[index_].gyro_.x();
  imu_msg.angular_velocity.y = measurements_[index_].gyro_.y();
  imu_msg.angular_velocity.z = measurements_[index_].gyro_.z();
  imu_msg.orientation_covariance[0] = -1.0;
  imu_msg.header.frame_id = "base_link";
  imu_msg.header.stamp.sec = timestamp_sec;
  imu_msg.header.stamp.nanosec = timestamp_nanosec;

  writer.write(imu_msg, "/imu", rclcpp::Time{measurements_[index_].timestamp_});
  ++index_;
}

void GPS5Chunk::add(const std::string_view, uint64_t timestamp,
                    std::span<const double> vec) {

  Data d{};
  d.timestamp_ = timestamp * 1'000;
  const auto num_elements{vec.size() / num_components_};

  d.lla_.resize(num_elements);
  d.vel_.resize(num_elements);

  for (auto &&i : ints(0ul, num_elements)) {
    d.lla_[i].x() = vec[i * num_components_];
    d.lla_[i].y() = vec[i * num_components_ + 1];
    d.lla_[i].z() = vec[i * num_components_ + 2];
    d.vel_[i].x() = vec[i * num_components_ + 3];
    d.vel_[i].y() = vec[i * num_components_ + 4];
  }

  data_.push_back(std::move(d));
}

void GPS5Chunk::create_measurements() {

  const auto total_num{ranges::accumulate(
      data_ | transform([](const Data &d) { return d.lla_.size(); }), 0ul)};

  measurements_.reserve(total_num);

  for (auto &&d : data_ | sliding(2)) {
    const int64_t delta{d[1].timestamp_ - d[0].timestamp_};
    const double delta_frame{static_cast<double>(delta) /
                             static_cast<double>(d[0].lla_.size())};

    for (auto &&[k, gps_data] : enumerate(zip(d[0].lla_, d[0].vel_))) {
      auto &&[lla, vel] = gps_data;

      measurements_.push_back(
          Measurement{.timestamp_ = d[0].timestamp_ +
                                    static_cast<int64_t>(k * delta_frame + 0.5),
                      .lla_ = lla,
                      .vel_ = vel});
    }
  }

  const double delta_frame{1'000'000'000.0 / frame_rate_};

  for (auto &&[k, gps_data] :
       enumerate(zip(data_.back().lla_, data_.back().vel_))) {
    auto &&[lla, vel] = gps_data;

    measurements_.push_back(
        Measurement{.timestamp_ = data_.back().timestamp_ +
                                  static_cast<int64_t>(k * delta_frame + 0.5),
                    .lla_ = lla,
                    .vel_ = vel});
  }
}

void GPS5Chunk::write(rosbag2_cpp::Writer &writer) {

  if (index_ >= measurements_.size()) {
    return;
  }

  const int32_t timestamp_sec{
      static_cast<int32_t>(measurements_[index_].timestamp_ / 1'000'000'000)};

  const uint32_t timestamp_nanosec{static_cast<uint32_t>(
      measurements_[index_].timestamp_ - timestamp_sec * 1'000'000'000)};

  sensor_msgs::msg::NavSatFix gps_msg{};

  gps_msg.latitude = measurements_[index_].lla_.x();
  gps_msg.longitude = measurements_[index_].lla_.y();
  gps_msg.altitude = measurements_[index_].lla_.z();
  gps_msg.header.frame_id = "gps";
  gps_msg.header.stamp.sec = timestamp_sec;
  gps_msg.header.stamp.nanosec = timestamp_nanosec;

  writer.write(gps_msg, "/fix", rclcpp::Time{measurements_[index_].timestamp_});
  ++index_;
}