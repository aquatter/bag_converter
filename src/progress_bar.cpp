#include <fmt/color.h>
#include <fmt/format.h>
#include <progress_bar.hpp>

ProgressBar::ProgressBar(std::span<const ProgressInfo> topics)
    : info_{topics.begin(), topics.end()} {

  for (int i{0}; auto &&topic : topics) {
    info_[i].ind_ = i;
    info_[i].processed_count_ = 0;

    max_name_size_ = std::max(max_name_size_, topic.topic_name_.size());
    max_count_size_ = std::max(max_count_size_,
                               fmt::format("{}", topic.message_count_).size());

    topic_name_to_ind_[topic.topic_name_] = i;
    ++i;
  }
}

void ProgressBar::progress(const std::string &topic, size_t progress) {
  if (not topic_name_to_ind_.contains(topic)) {
    return;
  }

  auto &info{info_[topic_name_to_ind_[topic]]};
  info.processed_count_ = progress;

  advance(topic, 0);
}

void ProgressBar::advance(const std::string &topic, size_t how_much) {

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
    fmt::print("\e[38;2;154;205;50m{:<{}}\e[0m [\e[38;5;69m{:-^{}}\e[0m] "
               "{:>{}}/{:>{}} \e[38;2;255;127;80m100%\e[0m\n",
               topic, max_name_size_, "-", length_, info.processed_count_,
               max_count_size_, info.message_count_, max_count_size_);
  } else if (progress == 0) {
    fmt::print("\e[38;2;154;205;50m{:<{}}\e[0m [{:^{}}] "
               "{:>{}}/{:>{}} \e[38;2;255;127;80m{:>3}%\e[0m\n",
               topic, max_name_size_, " ", length_, info.processed_count_,
               max_count_size_, info.message_count_, max_count_size_, percents);
  } else {
    fmt::print("\e[38;2;154;205;50m{:<{}}\e[0m [\e[38;5;69m{:-^{}}\e[0m{:^{}}] "
               "{:>{}}/{:>{}} \e[38;2;255;127;80m{:>3}%\e[0m\n",
               topic, max_name_size_, "-", progress, " ", length_ - progress,
               info.processed_count_, max_count_size_, info.message_count_,
               max_count_size_, percents);
  }

  curr_pos_ = info.ind_ + 1;
}

void ProgressBar::done() {
  fmt::print("\e[{}E", static_cast<int>(info_.size()) - curr_pos_);
}

void ProgressBar::draw() {
  for (auto &&info : info_) {
    fmt::print("\e[38;5;84m{:<{}}\e[0m [{:^{}}] {:>{}}/{:>{}}   "
               "\e[38;5;208m0%\e[0m\n",
               info.topic_name_, max_name_size_, " ", length_,
               info.processed_count_, max_count_size_, info.message_count_,
               max_count_size_);
  }

  curr_pos_ = static_cast<int>(info_.size());
}
