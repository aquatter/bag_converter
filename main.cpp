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
#include <iostream>
#include <memory>
#include <opencv2/core/cvstd.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/time.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/bag_metadata.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/topic_metadata.hpp>
#include <rosbag2_transport/reader_writer_factory.hpp>
#include <rosbag2_transport/record_options.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <string_view>

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

class BagConverter {
public:
  BagConverter(const std::string_view in, const std::string_view out,
               bool extract_images, bool extract_video)
      : extract_images_{extract_images}, extract_video_{extract_video},
        output_path_{out} {
    reader_.open(in.data());
    fmt::print(fmt::fg(fmt::color::aquamarine), "Successfully opened bag: {}\n",
               in.data());

    if (not(extract_images_ or extract_video_)) {
      rosbag2_storage::StorageOptions storage_options;
      storage_options.uri = out.data();
      storage_options.max_bagfile_duration = 30;

      writer_.open(storage_options);
    }

    if (extract_images_) {
      if (not std::filesystem::exists(out)) {
        std::filesystem::create_directory(out);
      }

      fmt::print(fmt::fg(fmt::color::aquamarine), "Extracting images to: {}\n",
                 out.data());
    }

    if (extract_video_) {
      if (output_path_.back() == '/') {
        output_path_.pop_back();
      }
      video_path_ = fmt::format("{}.avi", output_path_);

      fmt::print(fmt::fg(fmt::color::aquamarine), "Extracting video to: {}\n",
                 video_path_);
    }
  }

  void init_video_writer(int width, int height) {

    video_writer_ = std::make_unique<cv::VideoWriter>(cv::VideoWriter{
        video_path_, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10.0,
        cv::Size{width, height}, true});
  }

  void process(float resize = 1.0f) {

    ProgressBar bar{reader_.get_metadata().topics_with_message_count};
    bar.draw();

    bool initial_gps_reading{true};
    GeographicLib::LocalCartesian local_converter{};

    while (reader_.has_next()) {
      auto msg{reader_.read_next()};

      if (extract_images_ or extract_video_) {
        if (msg->topic_name == "/camera/image_raw/compressed") {

          const rclcpp::SerializedMessage serialized_msg{*msg->serialized_data};
          sensor_msgs::msg::CompressedImage compressed_img_msg;

          serialization_compressed_.deserialize_message(&serialized_msg,
                                                        &compressed_img_msg);

          cv::Mat_<cv::Vec3b> img =
              cv::imdecode(compressed_img_msg.data, cv::IMREAD_UNCHANGED);

          if (resize != 1.0f) {
            cv::resize(img, img,
                       {static_cast<int>(img.cols * resize),
                        static_cast<int>(img.rows * resize)});
          }

          const uint64_t timestamp{
              static_cast<uint64_t>(compressed_img_msg.header.stamp.nanosec) +
              static_cast<uint64_t>(compressed_img_msg.header.stamp.sec) *
                  1'000'000'000};

          if (extract_video_) {
            if (not video_writer_) {
              init_video_writer(img.cols, img.rows);
            }

            video_writer_->write(img);
          } else {
            cv::imwrite(fmt::format("{}/{}.png", output_path_, timestamp), img);
          }

        } else if (msg->topic_name == "/camera/image_raw") {

          const rclcpp::SerializedMessage serialized_msg{*msg->serialized_data};

          sensor_msgs::msg::Image img_msg;
          serialization_raw_.deserialize_message(&serialized_msg, &img_msg);

          cv::Mat_<cv::Vec3b> img =
              cv::Mat_<uint8_t>{img_msg.data, true}.reshape(3, img_msg.height);

          if (resize != 1.0f) {
            cv::resize(img, img,
                       {static_cast<int>(img.cols * resize),
                        static_cast<int>(img.rows * resize)});
          }

          const uint64_t timestamp{
              static_cast<uint64_t>(img_msg.header.stamp.nanosec) +
              static_cast<uint64_t>(img_msg.header.stamp.sec) * 1'000'000'000};

          if (extract_video_) {
            if (not video_writer_) {
              init_video_writer(img.cols, img.rows);
            }

            video_writer_->write(img);
          } else {
            cv::imwrite(fmt::format("{}/{}.png", output_path_, timestamp), img);
          }
        }

      } else {

        if (msg->topic_name == "/fix") {
          writer_.write(msg, msg->topic_name, "sensor_msgs/msg/NavSatFix");

          const rclcpp::SerializedMessage serialized_msg{*msg->serialized_data};
          sensor_msgs::msg::NavSatFix gps_msg;
          serialization_gps_.deserialize_message(&serialized_msg, &gps_msg);

          geometry_msgs::msg::PoseStamped pose_msg;
          pose_msg.header = gps_msg.header;

          if (initial_gps_reading) {
            local_converter = GeographicLib::LocalCartesian{
                gps_msg.latitude, gps_msg.longitude, gps_msg.altitude};
            initial_gps_reading = false;
          } else {
            local_converter.Forward(gps_msg.latitude, gps_msg.longitude,
                                    gps_msg.altitude, pose_msg.pose.position.x,
                                    pose_msg.pose.position.y,
                                    pose_msg.pose.position.z);
          }

          writer_.write(pose_msg, "/gp_data",
                        rclcpp::Time{msg->send_timestamp});

        } else if (msg->topic_name == "/imu/mpu6050") {
          writer_.write(msg, msg->topic_name, "sensor_msgs/msg/Imu");

        } else if (msg->topic_name == "/camera/image_raw/compressed") {

          const rclcpp::SerializedMessage serialized_msg{*msg->serialized_data};
          sensor_msgs::msg::CompressedImage compressed_img_msg;

          serialization_compressed_.deserialize_message(&serialized_msg,
                                                        &compressed_img_msg);

          cv::Mat_<cv::Vec3b> img =
              cv::imdecode(compressed_img_msg.data, cv::IMREAD_UNCHANGED);

          if (resize != 1.0f) {
            cv::resize(img, img,
                       {static_cast<int>(img.cols * resize),
                        static_cast<int>(img.rows * resize)});
          }

          // cv::imwrite("test.png", img);

          sensor_msgs::msg::Image raw_img;

          raw_img.header = compressed_img_msg.header;
          raw_img.height = img.rows;
          raw_img.width = img.cols;
          raw_img.encoding = "bgr8";
          raw_img.is_bigendian = false;
          raw_img.step = img.step;
          raw_img.data.assign(img.data,
                              img.data + img.cols * img.rows * img.channels());

          writer_.write(raw_img, "/camera/image_raw",
                        rclcpp::Time{msg->send_timestamp});
        } else if (msg->topic_name == "/camera/image_raw") {

          if (resize == 1.0f) {
            writer_.write(msg, msg->topic_name, "sensor_msgs/msg/Image");
          } else {

            const rclcpp::SerializedMessage serialized_msg{
                *msg->serialized_data};
            sensor_msgs::msg::Image img_msg;

            serialization_raw_.deserialize_message(&serialized_msg, &img_msg);

            cv::Mat_<cv::Vec3b> img =
                cv::Mat_<uint8_t>{img_msg.data, true}.reshape(3,
                                                              img_msg.height);

            cv::resize(img, img,
                       {static_cast<int>(img.cols * resize),
                        static_cast<int>(img.rows * resize)});

            sensor_msgs::msg::Image raw_img;

            raw_img.header = img_msg.header;
            raw_img.height = img.rows;
            raw_img.width = img.cols;
            raw_img.encoding = "bgr8";
            raw_img.is_bigendian = false;
            raw_img.step = img.step;
            raw_img.data.assign(img.data, img.data + img.cols * img.rows *
                                                         img.channels());

            writer_.write(raw_img, msg->topic_name,
                          rclcpp::Time{msg->send_timestamp});
          }
        }
      }

      bar.advance(msg->topic_name);
    }

    bar.done();
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
};

int main(int argc, char const *const *argv) {

  try {
    CLI::App app{"Bag Converter"};

    std::string input_bag, output_bag;
    app.add_option("-i, --input", input_bag, "Specify input bag path")
        ->required()
        ->check(CLI::ExistingDirectory);

    app.add_option("-o, --output", output_bag, "Specify output path")
        ->required();

    float resize{1.0f};
    app.add_option("-r, --resize", resize)->default_val(1.0f);

    bool extract_images{false};
    app.add_flag("-e, --extract", extract_images, "Extract images")
        ->default_val(false);

    bool extract_video{false};
    app.add_flag("-v, --video", extract_video, "Extract video")
        ->default_val(false);

    CLI11_PARSE(app, argc, argv);

    BagConverter cvt{input_bag, output_bag, extract_images, extract_video};
    cvt.process(resize);
  } catch (const std::exception &ex) {
    fmt::print(fmt::fg(fmt::color::red), "{}\n", ex.what());
  }

  fmt::print(fmt::fg(fmt::color::aquamarine), "Done\n");
  return EXIT_SUCCESS;
}
