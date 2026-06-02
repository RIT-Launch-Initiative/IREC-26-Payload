#include "image_handler/ImageHandlerNode.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <filesystem>
#include <sstream>
#include <thread>
#include <algorithm>

extern "C" {
#include "ssdv/ssdv.h"
}

ImageHandlerNode::ImageHandlerNode() : Node("image_compressor") {
  imageRequestSub = this->create_subscription<cubesat_msgs::msg::ImageRequest>(
      "image_request", 10,
      std::bind(&ImageHandlerNode::handleImageRequest, this, std::placeholders::_1));

  // rawImageSub = this->create_subscription<sensor_msgs::msg::Image>(
      // "raw_image", 10,
      // std::bind(&ImageHandlerNode::handleRawImage, this, std::placeholders::_1));
// 

  // TODO: Think of better default location on Pi
  saveDirectory = this->declare_parameter<std::string>("save_directory", "/tmp/images");
  callsign = this->declare_parameter<std::string>("callsign", "KD2YIE");
}

ImageHandlerNode::~ImageHandlerNode() = default;

void ImageHandlerNode::handleImageRequest(const cubesat_msgs::msg::ImageRequest::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(imageMutex);

  // if (!lastRawImage) {
  //   RCLCPP_WARN(this->get_logger(), "No raw image available for request ID %u", msg->image_id);
  //   return;
  // }

  uint16_t pktSize = msg->max_packet_size;
  if (pktSize == 0) {
    pktSize = SSDV_PKT_SIZE;
  }

  RCLCPP_INFO(this->get_logger(),
              "Processing image request ID %u (quality=%u, pkt_size=%u) crop (%u,%u %u,%u )",
              msg->image_id, msg->quality, pktSize, msg->left, msg->top, msg->right, msg->bottom);

  saveRawImageToDisk();
  compressAndSave(msg->image_id, msg->quality, msg->fec, pktSize);
}


void ImageHandlerNode::saveRawImageToDisk() {
  if (!lastRawImage) {
    RCLCPP_WARN(this->get_logger(), "No raw image to save");
    return;
  }

  try {
    std::filesystem::create_directories(saveDirectory);

    // const auto cvImage = cv_bridge::toCvShare(lastRawImage, lastRawImage->encoding);

    // std::ostringstream pathBuilder;
    // pathBuilder << saveDirectory;
    // if (!saveDirectory.empty() && saveDirectory.back() != '/') {
    //   pathBuilder << '/';
    // }
    // pathBuilder << "image_" << this->now().nanoseconds() << ".png";

    // const auto filePath = pathBuilder.str();
    // if (cv::imwrite(filePath, cvImage->image)) {
    //   RCLCPP_INFO(this->get_logger(), "Saved image to %s", filePath.c_str());
    // } else {
    //   RCLCPP_ERROR(this->get_logger(), "Failed to write image to %s", filePath.c_str());
    // }
  } catch (const cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception while saving image: %s", e.what());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception while saving image: %s", e.what());
  }
}

void ImageHandlerNode::compressAndSave(uint8_t imageId, uint8_t quality, bool fec, uint16_t maxPacketSize) {
  if (!lastRawImage) {
    RCLCPP_WARN(this->get_logger(), "No raw image to compress and transmit");
    return;
  }

  try {
    const auto cvImage = cv_bridge::toCvShare(lastRawImage, lastRawImage->encoding);
    cv::Mat image = cvImage->image;

    // SSDV requires image dimensions to be multiples of 16
    int width = image.cols - (image.cols % 16);
    int height = image.rows - (image.rows % 16);
    if (width <= 0 || height <= 0) {
      RCLCPP_ERROR(this->get_logger(), "Image too small for SSDV encoding (%dx%d)",
                   image.cols, image.rows);
      return;
    }
    if (width != image.cols || height != image.rows) {
      RCLCPP_INFO(this->get_logger(), "Cropping image from %dx%d to %dx%d (multiple of 16)",
                  image.cols, image.rows, width, height);
      image = image(cv::Rect(0, 0, width, height));
    }

    // Encode as JPEG for SSDV input (high quality will have SSDV re-quantize)
    std::vector<uint8_t> jpegData;
    std::vector<int> compressionParams = {cv::IMWRITE_JPEG_QUALITY, 95};
    if (!cv::imencode(".jpg", image, jpegData, compressionParams)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to encode image as JPEG");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "JPEG encoded: %zu bytes (%dx%d)",
                jpegData.size(), width, height);

    // Init SSDV encoder
    ssdv_t ssdv;
    uint8_t type = fec ? SSDV_TYPE_NORMAL : SSDV_TYPE_NOFEC;
    int pktSize = static_cast<int>(maxPacketSize);

    // Clamp
    if (pktSize > SSDV_PKT_SIZE) {
      pktSize = SSDV_PKT_SIZE;
    }

    std::string callsignCopy = callsign;
    if (ssdv_enc_init(&ssdv, type, callsignCopy.data(), imageId,
                      static_cast<int8_t>(quality), pktSize) != SSDV_OK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize SSDV encoder");
      return;
    }

    // Encode all SSDV packets
    std::vector<std::vector<uint8_t>> packets;
    uint8_t pktBuf[SSDV_PKT_SIZE];
    ssdv_enc_set_buffer(&ssdv, pktBuf);

    size_t feedOffset = 0;
    const size_t feedChunkSize = 128;

    while (true) {
      // Feed JPEG data to the encoder
      size_t remaining = jpegData.size() - feedOffset;
      size_t toFeed = std::min(feedChunkSize, remaining);

      int status;
      if (toFeed > 0) {
        status = ssdv_enc_feed(&ssdv, jpegData.data() + feedOffset, toFeed);
        feedOffset += toFeed;
      } else {
        // No more data to feed - feed empty to flush
        status = ssdv_enc_feed(&ssdv, nullptr, 0);
      }

      if (status == SSDV_HAVE_PACKET) {
        ssdv_enc_get_packet(&ssdv);
        packets.emplace_back(pktBuf, pktBuf + pktSize);
        ssdv_enc_set_buffer(&ssdv, pktBuf);
      } else if (status == SSDV_EOI || status == SSDV_ERROR) {
        if (status == SSDV_ERROR) {
          RCLCPP_ERROR(this->get_logger(), "SSDV encoding error");
        }
        break;
      }
      // SSDV_FEED_ME or SSDV_OK: continue feeding
    }

    if (packets.empty()) {
      RCLCPP_WARN(this->get_logger(), "SSDV encoding produced no packets");
      return;
    }

    const uint32_t totalPackets = static_cast<uint32_t>(packets.size());
    RCLCPP_INFO(this->get_logger(), "SSDV encoded image %u: %u packets (%d bytes each, fec=%d, quality=%u)",
                imageId, totalPackets, pktSize, fec, quality);

    // Save each SSDV packet
    for (uint32_t i = 0; i < totalPackets; ++i) {

      RCLCPP_DEBUG(this->get_logger(), "TOTO saved SSDV packet %u/%u (%zu bytes)",
                   i + 1, totalPackets, packets[i].size());

      // Small delay between packets to avoid overwhelming the radio
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    RCLCPP_INFO(this->get_logger(), "Completed SSDV encoding of image %u", imageId);
    cubesat_msgs::msg::ImageMetadata meta;
    meta.

  } catch (const cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception during SSDV encoding: %s", e.what());
  }
}
