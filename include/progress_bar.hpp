#include <mutex>
#include <span>
#include <string>
#include <unordered_map>
#include <vector>

class ProgressBar {
public:
  struct ProgressInfo {
    size_t message_count_;
    size_t processed_count_;
    std::string topic_name_;
    int ind_;
  };

  ProgressBar(std::span<const ProgressInfo> topics);
  void advance(const std::string &topic, size_t how_much = 1);
  void progress(const std::string &topic, size_t progress);
  void done();
  void draw();

private:
  std::vector<ProgressInfo> info_;
  std::unordered_map<std::string, int> topic_name_to_ind_;
  size_t max_name_size_{0};
  size_t max_count_size_{0};
  static constexpr int length_{50};
  int curr_pos_;
  std::mutex protector_;
};