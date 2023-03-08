#include <filesystem>
#include <functional>
#include <memory>
#include <queue>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class ImagePublisher : public rclcpp::Node {
 public:
  ImagePublisher() : Node("image_publisher") {
    this->declare_parameter("image_dir", ".");
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(30ms, std::bind(&ImagePublisher::timer_callback, this));
    image_path_ = this->get_parameter("image_dir").get_parameter_value().get<std::string>();
    directory_it_ = std::filesystem::directory_iterator(image_path_);
    image_it_ = directory_it_;
  }

 private:
  void timer_callback() {
    auto wd = this->get_parameter("image_dir").get_parameter_value().get<std::string>();
    if (image_path_ != wd) {
      image_path_ = wd;
      directory_it_ = std::filesystem::directory_iterator(image_path_);
      image_it_ = directory_it_;
    }

    if (image_it_ != std::filesystem::end(directory_it_)) {
      auto message = std_msgs::msg::String();
      message.data = image_it_->path().string();
      RCLCPP_INFO(this->get_logger(), "Publishing file: %s", message.data.c_str());
      ++image_it_;
      publisher_->publish(message);
    }
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::filesystem::directory_iterator directory_it_;
  std::filesystem::directory_iterator image_it_;
  std::filesystem::path image_path_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImagePublisher>());
  rclcpp::shutdown();
  return 0;
}