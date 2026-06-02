#pragma once

#include <mutex>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include "cubesat_msgs/msg/image_request.hpp"
#include "cubesat_msgs/msg/image_metadata.hpp"

class ImageHandlerNode final : public rclcpp::Node {
public:
  explicit ImageHandlerNode();
  ~ImageHandlerNode() override;

private:
  // Pub Subs
  rclcpp::Subscription<cubesat_msgs::msg::ImageRequest>::SharedPtr imageRequestSub;
  rclcpp::Publisher<cubesat_msgs::msg::ImageRequest>::SharedPtr imageMetadataPub;
  // rclcpp::Subscription<cubesat_msgs::msg::Image>::SharedPtr rawImageSub;

  
  // Callbacks
  void handleImageRequest(const cubesat_msgs::msg::ImageRequest::SharedPtr msg);
  // void handleRawImage(const sensor_msgs::msg::Image::SharedPtr msg);


  std::mutex imageMutex;

  // File saving
  std::string saveDirectory;

  // SSDV parameters
  std::string callsign;

  // Helpers
  void compressAndSave(uint8_t imageId, uint8_t quality, bool fec, uint16_t maxPacketSize);
  void saveRawImageToDisk();
};
