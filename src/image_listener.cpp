#include <memory>

#include <deplex/deplex.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MinimalSubscriber : public rclcpp::Node {
 public:
  MinimalSubscriber() : Node("image_listener") {
    this->declare_parameter("config_path", ".");
    this->declare_parameter("intrinsics_path", ".");
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
  }

 private:
  void topic_callback(const std_msgs::msg::String& msg) const {
    auto image_path = msg.data;
    if (image_path.substr(image_path.find_last_of('.') + 1) != "png") {
      RCLCPP_INFO(this->get_logger(), "Received %s, NOT A PNG file!", image_path.c_str());
      return;
    }
    auto config_path = this->get_parameter("config_path").get_parameter_value().get<std::string>();
    auto intrinsics_path = this->get_parameter("intrinsics_path").get_parameter_value().get<std::string>();

    auto image = deplex::utils::Image(image_path);
    auto intrinsics = deplex::utils::readIntrinsics(intrinsics_path);
    auto algorithm = deplex::PlaneExtractor(image.getHeight(), image.getWidth(), deplex::config::Config(config_path));
    auto extracted_planes = algorithm.process(image.toPointCloud(intrinsics));

    std::sort(extracted_planes.begin(), extracted_planes.end());
    int planes_count =
        std::distance(extracted_planes.begin(), std::unique(extracted_planes.begin(), extracted_planes.end()));

    RCLCPP_INFO(this->get_logger(), "Processing %s\nPlanes found: %d", msg.data.c_str(), planes_count);
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}