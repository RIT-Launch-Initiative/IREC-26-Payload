#include "image_handler/ImageHandlerNode.hpp"

#include "image_handler/ImageTaker.hpp"
#include "rclcpp/serialization.hpp"
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <ostream>
#include <sstream>
#include <thread>

using namespace std::chrono_literals;

extern "C" {
#include "ssdv/ssdv.h"
}

ImageHandlerNode::ImageHandlerNode() : Node("image_compressor") {
    imageRequestSub = this->create_subscription<cubesat_msgs::msg::ImageRequest>(
        "/watcher/image_request", 10, std::bind(&ImageHandlerNode::handleImageRequest, this, std::placeholders::_1));

    imageMetadataPub = create_publisher<cubesat_msgs::msg::ImageMetadata>("watcher/image_metadata", 10);

    flight_dir = declare_parameter<std::string>("flight_dir", "~/unconfigured_flight_dir");
    saveDirectory = flight_dir + "/images/";
    callsign = this->declare_parameter<std::string>("callsign", "KD2YIE");

    auto maybe_next_image = nextImageIdForDir(saveDirectory);
    if (maybe_next_image) {
        uint32_t next_image = *maybe_next_image;
        if (next_image > 0) {
            next_image--;
        }
        auto path = pathForMetadata(next_image);
        cubesat_msgs::msg::ImageMetadata meta;
        if (loadImageMetadata(path, meta)) {
            imageMetadataPub->publish(meta);
        }
    }
}

ImageHandlerNode::~ImageHandlerNode() = default;
void ImageHandlerNode::handleImageRequest(const cubesat_msgs::msg::ImageRequest::SharedPtr msg) {
    auto maybe_id = nextImageIdForDir(saveDirectory);
    if (!maybe_id) {
        RCLCPP_ERROR(get_logger(), "Could not find next id to save image at");
        return;
    }
    uint32_t image_id = *maybe_id;

    if (!createFolderForImage(image_id)) {
        RCLCPP_ERROR(get_logger(), "Could not create directory for image");
        return;
    }

    std::filesystem::create_directories(saveDirectory);

    RCLCPP_INFO(get_logger(), "Cropping to x(%d,%d) y(%d,%d)", msg->left, msg->top, msg->right, msg->bottom);

    auto filePath = pathForFullImage(image_id);
    cv::Mat downscaledImage =
        take_image_and_crop(filePath, msg->left, msg->right, msg->top, msg->bottom, msg->output_width);

    if (downscaledImage.rows == 0) {
        RCLCPP_ERROR(get_logger(), "Failed to take image");
        return;
    }

    const uint16_t pktSize = 128;

    RCLCPP_INFO(this->get_logger(), "Processing image request (quality=%u) crop (%u,%u %u,%u ) -> ID %d", msg->quality,
                msg->left, msg->top, msg->right, msg->bottom, image_id);

    RCLCPP_INFO(this->get_logger(), "Saved raw image");

    rclcpp::Time img_time = now();

    bool doFec = false; // handled by lora layer
    uint16_t num_blocks = compressAndSave(downscaledImage, image_id, msg->quality, doFec, pktSize);

    RCLCPP_INFO(this->get_logger(), "Completed SSDV encoding of image %u", image_id);

    cubesat_msgs::msg::ImageMetadata meta;
    meta.timestamp = (uint32_t)img_time.seconds();
    meta.image_id = image_id;
    meta.request = *msg;
    meta.num_blocks = num_blocks;
    saveImageMetadata(meta, image_id);
    imageMetadataPub->publish(meta);
}

std::optional<uint32_t> stringToImageId(const std::string &str) {
    try {
        uint32_t num = std::stoi(str);
        return num;
    } catch (const std::invalid_argument &e) {
        return std::nullopt;
    } catch (const std::out_of_range &e) {
        return std::nullopt;
    }
}

std::optional<uint32_t> ImageHandlerNode::nextImageIdForDir(const std::string &dir) {
    namespace fs = std::filesystem;
    uint32_t max_id = 0;
    bool any_seen = false;
    if (!(fs::exists(dir) && fs::is_directory(dir))) {
        return std::nullopt;
    }
    for (const auto &entry : fs::directory_iterator(dir)) {
        fs::path p = entry.path();
        if (!fs::is_directory(p)) {
            continue;
        }
        auto maybe_id = stringToImageId(p.filename().string());
        if (!maybe_id) {
            continue;
        }
        // parse name
        uint32_t id = *maybe_id;
        max_id = std::max(id, max_id);
        any_seen |= true;
    }
    if (!any_seen) {
        return 0;
    }
    return max_id + 1;
}

uint16_t ImageHandlerNode::encodeSSDV(std::vector<uint8_t> &data, bool fec, uint8_t quality, int maxPacketSize,
                                      uint32_t imageId) {
    // Init SSDV encoder
    ssdv_t ssdv;
    uint8_t type = fec ? SSDV_TYPE_NORMAL : SSDV_TYPE_NOFEC;
    int pktSize = static_cast<int>(maxPacketSize);

    // Clamp
    if (pktSize > SSDV_PKT_SIZE) {
        pktSize = SSDV_PKT_SIZE;
    }

    if (ssdv_enc_init(&ssdv, type, callsign.data(), imageId, static_cast<int8_t>(quality), pktSize) != SSDV_OK) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize SSDV encoder");
        return 0;
    }

    // Encode all SSDV packets
    uint8_t pktBuf[SSDV_PKT_SIZE];
    ssdv_enc_set_buffer(&ssdv, pktBuf);

    size_t feedOffset = 0;

    uint32_t totalPackets = 0;

    // reading buffer
    uint8_t b[128]{0};
    auto fread = [&](uint8_t *buf, [[maybe_unused]] int count, size_t length) {
        size_t real_amt = std::min(length, (data.size() - feedOffset));
        memcpy(buf, data.data() + feedOffset, real_amt);
        feedOffset += real_amt;
        return real_amt;
    };

    int c = 0; // ssdv status
    // was a while 1 in ssdv source, not taking any chances
    for (int iterCount = 0; iterCount < 65535; iterCount++) {
        while ((c = ssdv_enc_get_packet(&ssdv)) == SSDV_FEED_ME) {
            size_t r = fread(b, 1, 128);

            if (r <= 0) {
                // fprintf(stderr, "Premature end of file\n");
                break;
            }
            ssdv_enc_feed(&ssdv, b, r);
        }

        if (c == SSDV_EOI) {
            // fprintf(stderr, "ssdv_enc_get_packet said EOI\n");
            break;
        } else if (c != SSDV_OK) {
            // fprintf(stderr, "ssdv_enc_get_packet failed: %i\n", c);
            return 0;
        }

        // fwrite(pkt, 1, pkt_length, fout);
        savePacket(pktBuf, pktSize, imageId, totalPackets);
        totalPackets++;
    }

    RCLCPP_INFO(this->get_logger(), "SSDV encoded image %u: %u packets (%d bytes each, fec=%d, quality=%u)", imageId,
                totalPackets, pktSize, fec, quality);

    return totalPackets;
}

uint16_t ImageHandlerNode::compressAndSave(cv::Mat &image, uint8_t imageId, uint8_t quality, bool fec,
                                           uint16_t maxPacketSize) {

    try {
        // SSDV requires image dimensions to be multiples of 16
        int width = image.cols - (image.cols % 16);
        int height = image.rows - (image.rows % 16);
        if (width <= 0 || height <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Image too small for SSDV encoding (%dx%d)", image.cols, image.rows);
            return 0;
        }
        if (width != image.cols || height != image.rows) {
            RCLCPP_INFO(this->get_logger(), "Cropping image from %dx%d to %dx%d (multiple of 16)", image.cols,
                        image.rows, width, height);
            image = image(cv::Rect(0, 0, width, height));
        }

        // Encode as JPEG for SSDV input (high quality will have SSDV re-quantize)
        std::vector<uint8_t> jpegData;
        std::vector<int> compressionParams = {cv::IMWRITE_JPEG_QUALITY, 95};
        if (!cv::imencode(".jpg", image, jpegData, compressionParams)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to encode image as JPEG");
            return 0;
        }
        if (!cv::imwrite(pathForCompressedImage(imageId), image)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to encode image as JPEG");
            return 0;
        }

        RCLCPP_INFO(this->get_logger(), "JPEG encoded: %zu bytes (%dx%d)", jpegData.size(), width, height);

        return encodeSSDV(jpegData, fec, quality, maxPacketSize, imageId);

    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Exception during SSDV encoding: %s", e.what());
    }
    return 0;
}

std::string ImageHandlerNode::pathForFullImage(uint32_t image_id) {
    std::ostringstream pathBuilder;
    pathBuilder << saveDirectory;
    if (!saveDirectory.empty() && saveDirectory.back() != '/') {
        pathBuilder << '/';
    }
    pathBuilder << image_id << "/image_full.png";

    return pathBuilder.str();
}
std::string ImageHandlerNode::pathForCompressedImage(uint32_t image_id) {
    std::ostringstream pathBuilder;
    pathBuilder << saveDirectory;
    if (!saveDirectory.empty() && saveDirectory.back() != '/') {
        pathBuilder << '/';
    }
    pathBuilder << image_id << "/image_compressed.png";

    return pathBuilder.str();
}

std::string ImageHandlerNode::pathForMetadata(uint32_t image_id) {
    std::ostringstream pathBuilder;
    pathBuilder << saveDirectory;
    if (!saveDirectory.empty() && saveDirectory.back() != '/') {
        pathBuilder << '/';
    }
    pathBuilder << image_id << "/metadata.bin";

    return pathBuilder.str();
}

std::string ImageHandlerNode::pathForPacket(uint32_t image_id, uint16_t block_id) {
    std::ostringstream pathBuilder;
    pathBuilder << saveDirectory;
    if (!saveDirectory.empty() && saveDirectory.back() != '/') {
        pathBuilder << '/';
    }
    pathBuilder << image_id << "/packets/" << block_id << ".bin";

    return pathBuilder.str();
}
bool ImageHandlerNode::createFolderForImage(uint32_t image_id) {
    std::ostringstream pathBuilder;
    pathBuilder << saveDirectory;
    if (!saveDirectory.empty() && saveDirectory.back() != '/') {
        pathBuilder << '/';
    }
    pathBuilder << image_id << "/packets/";
    return std::filesystem::create_directories(pathBuilder.str());
}

void ImageHandlerNode::savePacket(uint8_t *data, uint32_t length, uint32_t image_id, uint16_t block_index) {
    std::string filename = pathForPacket(image_id, block_index);
    std::ofstream file(filename, std::ios::out | std::ios::binary);

    if (file.is_open()) {
        file.write(reinterpret_cast<const char *>(data), length);
        file.close();
    }
}

void ImageHandlerNode::saveImageMetadata(const cubesat_msgs::msg::ImageMetadata &pkt, uint32_t image_id) {

    rclcpp::Serialization<cubesat_msgs::msg::ImageMetadata> serializer;
    rclcpp::SerializedMessage serialized_msg;
    serializer.serialize_message(&pkt, &serialized_msg);

    std::string path = pathForMetadata(image_id);

    std::ofstream file(path, std::ios::out | std::ios::binary);
    file.write(reinterpret_cast<const char *>(serialized_msg.get_rcl_serialized_message().buffer),
               serialized_msg.get_rcl_serialized_message().buffer_length);
    file.close();
}

bool ImageHandlerNode::loadImageMetadata(std::string path, cubesat_msgs::msg::ImageMetadata &metadata) {
    try {
        // read file
        std::ifstream file(path, std::ios::in | std::ios::binary | std::ios::ate);
        file.exceptions(std::ofstream::failbit | std::ofstream::badbit);

        std::streamsize size = file.tellg();
        file.seekg(0, std::ios::beg);

        std::vector<uint8_t> buffer(size);
        file.read(reinterpret_cast<char *>(buffer.data()), size);
        file.close();

        rclcpp::SerializedMessage serialized_msg;
        auto &rcl_msg = serialized_msg.get_rcl_serialized_message();

        // deserialize
        rmw_serialized_message_resize(&rcl_msg, size);
        std::memcpy(rcl_msg.buffer, buffer.data(), size);
        rcl_msg.buffer_length = size; // Explicitly set length to avoid invalid state errors

        rclcpp::Serialization<cubesat_msgs::msg::ImageMetadata> serializer;
        serializer.deserialize_message(&serialized_msg, &metadata);
        return true;
    } catch (std::exception &e) {
        RCLCPP_WARN(get_logger(), "Failed to load image metadata profile from file %s: %s", path.c_str(), e.what());
        return false;
    }
}
