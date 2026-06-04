#pragma once

#include "cubesat_msgs/msg/image_metadata.hpp"
#include "cubesat_msgs/msg/image_request.hpp"
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>

class ImageHandlerNode final : public rclcpp::Node {
  public:
    explicit ImageHandlerNode();
    ~ImageHandlerNode() override;

  private:
    // Pub Subs
    rclcpp::Subscription<cubesat_msgs::msg::ImageRequest>::SharedPtr imageRequestSub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rawImageSub;
    // where we publish info about the image we just took
    rclcpp::Publisher<cubesat_msgs::msg::ImageMetadata>::SharedPtr imageMetadataPub;

    // Callbacks
    void handleImageRequest(const cubesat_msgs::msg::ImageRequest::SharedPtr msg);
    void handleRawImage(const sensor_msgs::msg::Image::SharedPtr msg);

    // Helpers
    std::optional<uint32_t> nextImageIdForDir(const std::string &dir);
    std::string pathForFullImage(uint32_t image_id);
    std::string pathForCompressedImage(uint32_t image_id);
    std::string pathForMetadata(uint32_t image_id);
    std::string pathForPacket(uint32_t image_id, uint16_t block_id);
    bool createFolderForImage(uint32_t image_id);
    void savePacket(uint8_t *data, uint32_t length, uint32_t image_id, uint16_t block_index);
    void saveImageMetadata(const cubesat_msgs::msg::ImageMetadata &pkt, uint32_t image_id);
    uint16_t encodeSSDV(std::vector<uint8_t> &data, bool fec, uint8_t quality, int maxPacketSize, uint32_t imageId);


    // data
    std::mutex imageMutex;
    sensor_msgs::msg::Image::SharedPtr lastRawImage;

    std::optional<cubesat_msgs::msg::ImageRequest> outstanding_request;

    // File saving
    std::string flight_dir;
    std::string saveDirectory;

    // SSDV parameters
    std::string callsign;

    // Helpers
    uint16_t compressAndSave(uint8_t imageId, uint8_t quality, bool fec, uint16_t maxPacketSize, uint16_t cropLeft,
                             uint16_t cropRight, uint16_t cropTop, uint16_t cropBottom, uint16_t downscaleWidth);
    void saveRawImageToDisk(uint32_t id);
};
