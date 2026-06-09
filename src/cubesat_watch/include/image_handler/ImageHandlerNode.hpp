#pragma once

#include "cubesat_msgs/msg/image_metadata.hpp"
#include "cubesat_msgs/msg/image_request.hpp"
#include <mutex>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <span>
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
    // where we publish info about the image we just took
    rclcpp::Publisher<cubesat_msgs::msg::ImageMetadata>::SharedPtr imageMetadataPub;

    // Callbacks
    void handleImageRequest(const cubesat_msgs::msg::ImageRequest::SharedPtr msg);

    // Helpers
    std::optional<uint32_t> nextImageIdForDir(const std::string &dir);
    std::string pathForFullImage(uint32_t image_id);
    std::string pathForCompressedImage(uint32_t image_id);
    std::string pathForMetadata(uint32_t image_id);
    std::string pathForPacket(uint32_t image_id, uint16_t block_id);

    bool createFolderForImage(uint32_t image_id);
    void savePacket(uint8_t *data, uint32_t length, uint32_t image_id, uint16_t block_index);
    void saveImageMetadata(const cubesat_msgs::msg::ImageMetadata &pkt, uint32_t image_id);
    bool loadImageMetadata(std::string path, cubesat_msgs::msg::ImageMetadata &metadata);

    uint16_t encodeSSDV(std::vector<uint8_t> &jpeg_data, bool fec, uint8_t quality, int maxPacketSize,
                        uint32_t imageId);

    // File saving
    std::string flight_dir;
    std::string saveDirectory;

    // SSDV parameters
    std::string callsign;

    // Helpers
    uint16_t compressAndSave(cv::Mat &fullImage, uint8_t imageId, uint8_t quality, bool fec,
                                               uint16_t maxPacketSize);
};
