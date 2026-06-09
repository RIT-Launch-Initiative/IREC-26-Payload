#include "image_handler/ImageTaker.hpp"
#include "rclcpp/rclcpp.hpp"
#include <libcamera/formats.h>
#include <libcamera/libcamera.h>

#include <iomanip>
#include <iostream>
#include <memory>
#include <sys/mman.h>
#include <thread>

#define WIDTH 1280
#define HEIGHT 800

using namespace libcamera;
using namespace std::chrono_literals;

static std::shared_ptr<Camera> camera;

volatile bool ignore = false;
int count = 0;

std::string path_to_save = "/tmp/unconfigured_image.jpg";
uint16_t cropLeft_ = 0;
uint16_t cropRight_ = 0;
uint16_t cropTop_ = 0;
uint16_t cropBottom_ = 0;
uint16_t downscaleWidth_ = 0;
cv::Mat downscaledImage{};

cv::Mat cropAndDownscaleImage(const cv::Mat &full_image) {
    auto logger = rclcpp::get_logger("image_taker");
    cv::Rect roi(cropLeft_, cropTop_, cropRight_ - cropLeft_, cropBottom_ - cropTop_);
    cv::Mat croppedImage = full_image(roi);

    uint16_t croppedHeight = croppedImage.rows;
    uint16_t croppedWidth = croppedImage.cols;
    RCLCPP_INFO(logger, "Original w%d h%d, Cropped to w%d h%d", full_image.cols, full_image.rows, croppedWidth,
                croppedHeight);

    float aspect = (float)croppedHeight / (float)croppedWidth;
    float encodedHeightF = downscaleWidth_ * aspect;
    uint16_t encodedHeightNonAligned = (uint16_t)encodedHeightF;
    uint16_t encodedHeightAligned = (encodedHeightNonAligned / 16) * 16;
    RCLCPP_INFO(logger, "Downscaled to w%d h%d", downscaleWidth_, encodedHeightAligned);

    cv::Mat downscaled_image;
    cv::resize(croppedImage, downscaled_image, cv::Size(downscaleWidth_, encodedHeightAligned), 0, 0, cv::INTER_LINEAR);
    return downscaled_image;
}

static void requestComplete(Request *request) {
    auto logger = rclcpp::get_logger("image_taker");

    if (ignore) {
        return;
    }
    ignore = true;
    if (request->status() == Request::RequestCancelled) {
        return;
    }
    const std::map<const Stream *, FrameBuffer *> &buffers = request->buffers();

    int buffCount = 0;
    for (auto bufferPair : buffers) {
        FrameBuffer *buffer = bufferPair.second;
        // const FrameMetadata &metadata = buffer->metadata();
        buffCount++;

        if (buffer->planes().size() != 1) {
            std::cout << "Got " << buffer->planes().size() << "planes and dont know what to do" << std::endl;
            return;
        }

        auto plane0 = buffer->planes()[0];

        std::cout << "Plane length" << plane0.length << " fd " << plane0.fd.get() << std::endl;

        void *mmapped_memory = mmap(nullptr, plane0.length, PROT_READ, MAP_SHARED, plane0.fd.get(), plane0.offset);
        if (mmapped_memory == NULL) {
            std::cerr << "Failed to map memory" << std::endl;
            break;
        }

        cv::Mat full_mat = cv::Mat(HEIGHT, WIDTH, CV_8UC3, mmapped_memory);
        try {
            if (cv::imwrite(path_to_save, full_mat)) {
                RCLCPP_INFO(logger, "Saved image to %s", path_to_save.c_str());
            } else {
                RCLCPP_ERROR(logger, "Failed to write image to %s", path_to_save.c_str());
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(logger, "Exception while saving image: %s", e.what());
        }

        downscaledImage = cropAndDownscaleImage(full_mat);

        munmap(mmapped_memory, plane0.length);

        break;
    }

    // for (auto bufferPair : buffers) {
    //     FrameBuffer *buffer = bufferPair.second;
    //     const FrameMetadata &metadata = buffer->metadata();
    // }

    request->reuse(Request::ReuseBuffers);
    // camera->queueRequest(request);
}

cv::Mat take_image_and_crop(std::string path, uint16_t cropLeft, uint16_t cropRight, uint16_t cropTop,
                            uint16_t cropBottom, uint16_t downscaleWidth) {
    ignore = false;
    count = 0;
    path_to_save = path;

    cropLeft_ = cropLeft;
    cropRight_ = cropRight;
    cropTop_ = cropTop;
    cropBottom_ = cropBottom;
    downscaleWidth_ = downscaleWidth;

    std::unique_ptr<CameraManager> cm = std::make_unique<CameraManager>();
    cm->start();
    for (auto const &camera : cm->cameras()) {
        std::cout << camera->id() << std::endl;
    }

    auto cameras = cm->cameras();
    if (cameras.empty()) {
        std::cout << "No cameras were identified on the system." << std::endl;
        cm->stop();
        return cv::Mat{};
    }

    std::string cameraId = cameras[0]->id();

    camera = cm->get(cameraId);
    /*
     * Note that `camera` may not compare equal to `cameras[0]`.
     * In fact, it might simply be a `nullptr`, as the particular
     * device might have disappeared (and reappeared) in the meantime.
     */
    camera->acquire();

    std::unique_ptr<CameraConfiguration> config = camera->generateConfiguration({StreamRole::StillCapture});
    StreamConfiguration &streamConfig = config->at(0);
    std::cout << "Default StillCapture configuration is: " << streamConfig.toString() << std::endl;

    streamConfig.size.width = WIDTH;
    streamConfig.size.height = HEIGHT;
    streamConfig.pixelFormat = libcamera::formats::RGB888;
    streamConfig.bufferCount = 1;

    config->validate();
    std::cout << "Validated StillCapture configuration is: " << streamConfig.toString() << std::endl;

    camera->configure(config.get());

    FrameBufferAllocator *allocator = new FrameBufferAllocator(camera);

    for (StreamConfiguration &cfg : *config) {
        int ret = allocator->allocate(cfg.stream());
        if (ret < 0) {
            std::cerr << "Can't allocate buffers" << std::endl;
            return cv::Mat{};
        }

        size_t allocated = allocator->buffers(cfg.stream()).size();
        std::cout << "Allocated " << allocated << " buffers for stream" << std::endl;
    }

    Stream *stream = streamConfig.stream();
    const std::vector<std::unique_ptr<FrameBuffer>> &buffers = allocator->buffers(stream);

    // the scope for requests
    std::vector<std::unique_ptr<Request>> requests;

    for (unsigned int i = 0; i < buffers.size(); ++i) {
        std::unique_ptr<Request> request = camera->createRequest();
        if (!request) {
            std::cerr << "Can't create request" << std::endl;
            return cv::Mat{};
        }

        std::unique_ptr<CameraConfiguration> config = camera->generateConfiguration({StreamRole::Viewfinder});

        ControlList &controls = request->controls();
        controls.set(controls::AeEnable, true);
        controls.set(controls::AwbEnable, true);

        const std::unique_ptr<FrameBuffer> &buffer = buffers[i];
        int ret = request->addBuffer(stream, buffer.get());
        if (ret < 0) {
            std::cerr << "Can't set buffer for request" << std::endl;
            return cv::Mat{};
        }

        requests.push_back(std::move(request));
    }

    camera->requestCompleted.connect(requestComplete);

    camera->start();
    for (std::unique_ptr<Request> &request : requests)
        camera->queueRequest(request.get());

    while (!ignore) {
        std::this_thread::sleep_for(100ms);
    }
    std::cout << "Handler made image. Cleaning up" << std::endl;

    camera->stop();
    std::this_thread::sleep_for(100ms);
    allocator->free(stream);
    delete allocator;
    camera->release();
    camera.reset();
    cm->stop();

    return downscaledImage;
}
