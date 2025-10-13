#include "sensor_msgs/msg/compressed_image.hpp"
#include <CLI/CLI.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fmt/color.h>
#include <fmt/format.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <opencv2/core/cvstd.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <optional>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/time.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/bag_metadata.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/topic_metadata.hpp>
#include <rosbag2_transport/reader_writer_factory.hpp>
#include <rosbag2_transport/record_options.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

template <typename MsgType> struct RosMessage {
  int64_t timestamp_;
  std::shared_ptr<MsgType> msg_;
};

struct MessageStorageBase {
  virtual bool empty() const = 0;
  virtual ~MessageStorageBase() = default;
};

template <typename MsgType> struct MessageStorage : MessageStorageBase {

  void put(RosMessage<MsgType> &&msg) { heap_.emplace(std::move(msg)); }

  std::optional<RosMessage<MsgType>> get(size_t queue_size) {
    if (heap_.empty()) {
      return std::nullopt;
    }

    if (heap_.size() < queue_size) {
      return std::nullopt;
    }

    const auto msg{heap_.top()};
    heap_.pop();

    return msg;
  }

  bool empty() const override { return heap_.empty(); }

  std::priority_queue<RosMessage<MsgType>, std::vector<RosMessage<MsgType>>,
                      decltype([](const RosMessage<MsgType> &a,
                                  const RosMessage<MsgType> &b) {
                        return a.timestamp_ > b.timestamp_;
                      })>
      heap_;
};

class ProgressBar {
public:
  ProgressBar(std::span<const rosbag2_storage::TopicInformation> topics) {

    for (int i{0}; auto &&topic : topics) {
      info_.push_back(ProgressInfo{.message_count_ = topic.message_count,
                                   .processed_count_ = 0,
                                   .topic_name_ = topic.topic_metadata.name,
                                   .ind_ = i});

      max_name_size_ =
          std::max(max_name_size_, topic.topic_metadata.name.size());
      max_count_size_ = std::max(max_count_size_,
                                 fmt::format("{}", topic.message_count).size());

      topic_name_to_ind_[topic.topic_metadata.name] = i;
      ++i;
    }
  }

  void advance(const std::string &topic, size_t how_much = 1) {

    if (not topic_name_to_ind_.contains(topic)) {
      return;
    }

    auto &info{info_[topic_name_to_ind_[topic]]};

    info.processed_count_ += how_much;
    const int progress{
        std::min(static_cast<int>(static_cast<float>(length_) *
                                  static_cast<float>(info.processed_count_) /
                                  static_cast<float>(info.message_count_)),
                 length_)};

    const int percents{std::min(
        static_cast<int>(100.0f * static_cast<float>(info.processed_count_) /
                             static_cast<float>(info.message_count_) +
                         0.5f),
        100)};

    if (info.ind_ < curr_pos_) {
      fmt::print("\e[{}F", curr_pos_ - info.ind_);
    }

    if (info.ind_ > curr_pos_) {
      fmt::print("\e[{}E", info.ind_ - curr_pos_);
    }

    if (progress == length_) {
      fmt::print("\e[38;5;84m{:<{}}\e[0m [\e[38;5;69m{:-^{}}\e[0m] "
                 "{:>{}}/{:>{}} \e[38;5;208m100%\e[0m\n",
                 topic, max_name_size_, "-", length_, info.processed_count_,
                 max_count_size_, info.message_count_, max_count_size_);
    } else if (progress == 0) {
      fmt::print("\e[38;5;84m{:<{}}\e[0m [{:^{}}] "
                 "{:>{}}/{:>{}} \e[38;5;208m{:>3}%\e[0m\n",
                 topic, max_name_size_, " ", length_, info.processed_count_,
                 max_count_size_, info.message_count_, max_count_size_,
                 percents);
    } else {
      fmt::print("\e[38;5;84m{:<{}}\e[0m [\e[38;5;69m{:-^{}}\e[0m{:^{}}] "
                 "{:>{}}/{:>{}} \e[38;5;208m{:>3}%\e[0m\n",
                 topic, max_name_size_, "-", progress, " ", length_ - progress,
                 info.processed_count_, max_count_size_, info.message_count_,
                 max_count_size_, percents);
    }

    curr_pos_ = info.ind_ + 1;
  }

  void done() {
    fmt::print("\e[{}E", static_cast<int>(info_.size()) - curr_pos_);
  }

  void draw() {
    for (auto &&info : info_) {
      fmt::print("\e[38;5;84m{:<{}}\e[0m [{:^{}}] {:>{}}/{:>{}}   "
                 "\e[38;5;208m0%\e[0m\n",
                 info.topic_name_, max_name_size_, " ", length_,
                 info.processed_count_, max_count_size_, info.message_count_,
                 max_count_size_);
    }

    curr_pos_ = static_cast<int>(info_.size());
  }

private:
  struct ProgressInfo {
    size_t message_count_;
    size_t processed_count_;
    std::string topic_name_;
    int ind_;
  };

  std::vector<ProgressInfo> info_;
  std::unordered_map<std::string, int> topic_name_to_ind_;
  size_t max_name_size_{0};
  size_t max_count_size_{0};
  static constexpr int length_{50};
  int curr_pos_;
};

struct BagConverterSettings {
  std::string input_bag_;
  std::string output_bag_;
  bool extract_images_;
  bool extract_video_;
  float resize_;
  int64_t start_time_;
  int64_t end_time_;
};

class BagConverter {
public:
  BagConverter(const BagConverterSettings &set)
      : extract_images_{set.extract_images_},
        extract_video_{set.extract_video_}, output_path_{set.output_bag_},
        resize_{set.resize_} {
    reader_.open(set.input_bag_);
    fmt::print(fmt::fg(fmt::color::aquamarine), "Successfully opened bag: {}\n",
               set.input_bag_);

    if (not(extract_images_ or extract_video_)) {
      rosbag2_storage::StorageOptions storage_options;
      storage_options.uri = set.output_bag_;
      storage_options.max_bagfile_duration = 30;

      writer_.open(storage_options);
    }

    if (extract_images_) {
      if (not std::filesystem::exists(set.output_bag_)) {
        std::filesystem::create_directory(set.output_bag_);
      }

      fmt::print(fmt::fg(fmt::color::aquamarine), "Extracting images to: {}\n",
                 set.output_bag_);
    }

    if (extract_video_) {
      if (output_path_.back() == '/') {
        output_path_.pop_back();
      }
      video_path_ = fmt::format("{}.avi", output_path_);

      fmt::print(fmt::fg(fmt::color::aquamarine), "Extracting video to: {}\n",
                 video_path_);
    }

    const int64_t bag_duration{reader_.get_metadata().duration.count()};

    const int64_t start_timestamp{
        reader_.get_metadata().starting_time.time_since_epoch().count()};

    start_time_ = 1'000'000'000l * set.start_time_;
    end_time_ = 1'000'000'000l * set.end_time_;

    start_time_ = start_time_ < 0 ? 0 : start_time_;
    end_time_ = end_time_ < 0 ? bag_duration : end_time_;

    if (start_time_ > bag_duration) {
      throw std::runtime_error{"start time beyond bag duration"};
    }

    if (end_time_ > bag_duration) {
      fmt::print(fmt::fg(fmt::color::orange),
                 "warning, end time exeeds bag duration and got truncated\n");

      end_time_ = bag_duration;
    }

    if (start_time_ >= end_time_) {
      throw std::runtime_error{
          "start time greater or equal to end time, nothing to convert"};
    }

    start_time_ += start_timestamp;
    end_time_ += start_timestamp;
  }

  void init_video_writer(int width, int height) {

    video_writer_ = std::make_unique<cv::VideoWriter>(cv::VideoWriter{
        video_path_, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10.0,
        cv::Size{width, height}, true});
  }

  template <typename MsgType>
  std::optional<RosMessage<MsgType>> try_pop(const std::string &topic_name,
                                             size_t queue_size) {
    if (not msg_map_.contains(topic_name)) {
      return std::nullopt;
    }

    auto msg_storage{std::dynamic_pointer_cast<MessageStorage<MsgType>>(
        msg_map_[topic_name])};

    return msg_storage->get(queue_size);
  }

  template <typename MsgType>
  void
  receive_message(std::shared_ptr<rosbag2_storage::SerializedBagMessage> msg) {

    const rclcpp::SerializedMessage serialized_msg{*msg->serialized_data};

    auto ros_msg{std::make_shared<MsgType>()};
    rclcpp::Serialization<MsgType> serialization{};
    serialization.deserialize_message(&serialized_msg, ros_msg.get());

    int64_t timestamp{static_cast<int64_t>(ros_msg->header.stamp.nanosec) +
                      static_cast<int64_t>(ros_msg->header.stamp.sec) *
                          1'000'000'000};

    if (not msg_map_.contains(msg->topic_name)) {
      msg_map_[msg->topic_name] = std::make_shared<MessageStorage<MsgType>>();
    }

    auto msg_storage{std::dynamic_pointer_cast<MessageStorage<MsgType>>(
        msg_map_[msg->topic_name])};

    msg_storage->put(
        RosMessage<MsgType>{.timestamp_ = timestamp, .msg_ = ros_msg});
  }

  void
  receive_message(std::shared_ptr<rosbag2_storage::SerializedBagMessage> msg) {

    if (msg->topic_name == "/camera/image_raw/compressed") {
      receive_message<sensor_msgs::msg::CompressedImage>(std::move(msg));
    } else if (msg->topic_name == "/camera/image_raw") {
      receive_message<sensor_msgs::msg::Image>(std::move(msg));
    } else if (msg->topic_name == "/fix") {
      receive_message<sensor_msgs::msg::NavSatFix>(std::move(msg));
    } else if (msg->topic_name == "/imu") {
      receive_message<sensor_msgs::msg::Imu>(std::move(msg));
    }
  }

  void process_message(const std::string &topic_name, size_t queue_size) {

    if (topic_name == "/camera/image_raw/compressed" or
        topic_name == "/camera/image_raw") {
      auto [img, header] = process_image(topic_name, queue_size);

      if (img.empty()) {
        return;
      }

      const rclcpp::Time time{header.stamp.sec, header.stamp.nanosec};

      if (extract_video_) {
        if (not video_writer_) {
          init_video_writer(img.cols, img.rows);
        }

        video_writer_->write(img);
      } else if (extract_images_) {
        cv::imwrite(fmt::format("{}/{}.png", output_path_, time.nanoseconds()),
                    img);
      } else {
        sensor_msgs::msg::Image raw_img;

        raw_img.header = header;
        raw_img.height = img.rows;
        raw_img.width = img.cols;
        raw_img.encoding = "bgr8";
        raw_img.is_bigendian = false;
        raw_img.step = img.step;
        raw_img.data.assign(img.data,
                            img.data + img.cols * img.rows * img.channels());

        writer_.write(raw_img, "/camera/image_raw", time);
      }
    } else if (topic_name == "/fix") {

      if (auto ros_msg{
              try_pop<sensor_msgs::msg::NavSatFix>(topic_name, queue_size)};
          ros_msg.has_value()) {

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header = ros_msg->msg_->header;

        const rclcpp::Time time{ros_msg->msg_->header.stamp.sec,
                                ros_msg->msg_->header.stamp.nanosec};

        if (not local_converter_) {
          local_converter_ = std::make_shared<GeographicLib::LocalCartesian>(
              ros_msg->msg_->latitude, ros_msg->msg_->longitude,
              ros_msg->msg_->altitude);

        } else {
          local_converter_->Forward(
              ros_msg->msg_->latitude, ros_msg->msg_->longitude,
              ros_msg->msg_->altitude, pose_msg.pose.position.x,
              pose_msg.pose.position.y, pose_msg.pose.position.z);
        }

        writer_.write(*(ros_msg->msg_), topic_name, time);
        writer_.write(pose_msg, "/gp_data", time);
      }

    } else if (topic_name == "/imu") {

      if (auto ros_msg{try_pop<sensor_msgs::msg::Imu>(topic_name, queue_size)};
          ros_msg.has_value()) {

        writer_.write(*(ros_msg->msg_), topic_name,
                      rclcpp::Time{ros_msg->msg_->header.stamp.sec,
                                   ros_msg->msg_->header.stamp.nanosec});
      }
    }
  }

  std::tuple<cv::Mat_<cv::Vec3b>, std_msgs::msg::Header>
  process_image(const std::string &topic_name, size_t queue_size) {
    if (topic_name == "/camera/image_raw/compressed") {

      if (auto ros_msg{try_pop<sensor_msgs::msg::CompressedImage>(topic_name,
                                                                  queue_size)};
          ros_msg.has_value()) {

        cv::Mat_<cv::Vec3b> img =
            cv::imdecode(ros_msg->msg_->data, cv::IMREAD_UNCHANGED);

        if (resize_ != 1.0f) {
          cv::resize(img, img,
                     {static_cast<int>(img.cols * resize_),
                      static_cast<int>(img.rows * resize_)});
        }

        return {img, ros_msg->msg_->header};
      }
    } else if (topic_name == "/camera/image_raw") {

      if (auto ros_msg{
              try_pop<sensor_msgs::msg::Image>(topic_name, queue_size)};
          ros_msg.has_value()) {

        cv::Mat_<cv::Vec3b> img =
            cv::Mat_<uint8_t>{ros_msg->msg_->data, true}.reshape(
                3, ros_msg->msg_->height);

        if (resize_ != 1.0f) {
          cv::resize(img, img,
                     {static_cast<int>(img.cols * resize_),
                      static_cast<int>(img.rows * resize_)});
        }

        return {img, ros_msg->msg_->header};
      }
    }

    return {cv::Mat_<cv::Vec3b>{}, std_msgs::msg::Header{}};
  }

  void process() {

    ProgressBar bar{reader_.get_metadata().topics_with_message_count};
    bar.draw();

    while (reader_.has_next()) {
      auto msg{reader_.read_next()};

      if (msg->recv_timestamp < start_time_) {
        bar.advance(msg->topic_name);
        continue;
      }

      if (msg->recv_timestamp > end_time_) {
        break;
      }

      if (msg->topic_name == "/camera/image_raw/compressed") {
        const rclcpp::SerializedMessage serialized_msg{*msg->serialized_data};

        sensor_msgs::msg::CompressedImage ros_msg{};
        rclcpp::Serialization<sensor_msgs::msg::CompressedImage>
            serialization{};

        serialization.deserialize_message(&serialized_msg, &ros_msg);

        cv::Mat_<cv::Vec3b> img =
            cv::imdecode(ros_msg.data, cv::IMREAD_UNCHANGED);

        sensor_msgs::msg::Image raw_img;

        raw_img.header = ros_msg.header;
        raw_img.height = img.rows;
        raw_img.width = img.cols;
        raw_img.encoding = "bgr8";
        raw_img.is_bigendian = false;
        raw_img.step = img.step;
        raw_img.data.assign(img.data,
                            img.data + img.cols * img.rows * img.channels());

        writer_.write(raw_img, "/camera/image_raw",
                      rclcpp::Time{msg->recv_timestamp});

      } else if (msg->topic_name == "/imu" or
                 msg->topic_name == "/bmi160/imu") {
        writer_.write(msg, msg->topic_name, "sensor_msgs/msg/Imu");
      } else if (msg->topic_name == "/fix") {
        writer_.write(msg, "/fix", "sensor_msgs/msg/NavSatFix");
      }

      // receive_message(msg);
      // process_message(msg->topic_name, 10);

      bar.advance(msg->topic_name);
    }

    bar.done();

    for (auto &&[topic_name, storage] : msg_map_) {
      while (not storage->empty()) {
        process_message(topic_name, 1);
      }
    }
  }

  ~BagConverter() {
    if (video_writer_) {
      video_writer_->release();
    }
  }

private:
  rosbag2_cpp::Reader reader_;
  rosbag2_cpp::Writer writer_;
  rclcpp::Serialization<sensor_msgs::msg::CompressedImage>
      serialization_compressed_;
  rclcpp::Serialization<sensor_msgs::msg::Image> serialization_raw_;
  rclcpp::Serialization<sensor_msgs::msg::NavSatFix> serialization_gps_;
  bool extract_images_;
  bool extract_video_;
  std::string output_path_;
  std::unique_ptr<cv::VideoWriter> video_writer_;
  std::string video_path_;
  std::unordered_map<std::string, std::shared_ptr<MessageStorageBase>> msg_map_;
  float resize_{1.0};
  std::shared_ptr<GeographicLib::LocalCartesian> local_converter_;
  int64_t start_time_;
  int64_t end_time_;
};

int main(int argc, char const *const *argv) {

  try {
    CLI::App app{"Bag Converter"};

    BagConverterSettings set{};

    app.add_option("-i, --input", set.input_bag_, "Specify input bag path")
        ->required()
        ->check(CLI::ExistingDirectory);

    app.add_option("-o, --output", set.output_bag_, "Specify output path")
        ->required();

    app.add_option("-r, --resize", set.resize_)->default_val(1.0f);

    app.add_flag("-e, --extract", set.extract_images_, "Extract images")
        ->default_val(false);

    app.add_flag("-v, --video", set.extract_video_, "Extract video")
        ->default_val(false);

    app.add_option("--start", set.start_time_, "Start time to process, s")
        ->default_val(-1);

    app.add_option("--end", set.end_time_, "End time to process, s")
        ->default_val(-1);

    CLI11_PARSE(app, argc, argv);

    BagConverter cvt{set};

    cvt.process();
  } catch (const std::exception &ex) {
    fmt::print(fmt::fg(fmt::color::red), "{}\n", ex.what());
  }

  fmt::print(fmt::fg(fmt::color::aquamarine), "Done\n");
  return EXIT_SUCCESS;
}
