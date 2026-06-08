#include "image_handler/ImageTaker.hpp"
#include <libcamera/libcamera.h>
#include <libcamera/formats.h>

#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>
#include <sys/mman.h>


#define WIDTH 1280
#define HEIGHT 800

using namespace libcamera;
using namespace std::chrono_literals;

static std::shared_ptr<Camera> camera;

Image global_image{
    .mmapped_memory = nullptr,
    .length = 0,
    .mat = cv::Mat(),
};

Image &get_global_image(){
    return global_image;
}

volatile bool ignore = false;
int count = 0;

static void requestComplete(Request *request) {
    // count++;
    // if (count < 100) {
    //     count++;
    // }
    if (ignore) {
        return;
    }
    ignore = true;
    if (request->status() == Request::RequestCancelled)
        return;
    const std::map<const Stream *, FrameBuffer *> &buffers = request->buffers();

    int buffCount = 0;
    for (auto bufferPair : buffers) {
        FrameBuffer *buffer = bufferPair.second;
        const FrameMetadata &metadata = buffer->metadata();
        buffCount++;

        if (buffer->planes().size() != 1) {
            std::cout << "Got " << buffer->planes().size() << "planes and odnt knwo what to do" << std::endl;
            return;
        }

        auto plane0 = buffer->planes()[0];

        std::cout << "Plane length" << plane0.length << " fd " << plane0.fd.get() << std::endl;

        global_image.mmapped_memory = mmap(nullptr, plane0.length, PROT_READ, MAP_SHARED, plane0.fd.get(), plane0.offset);
        global_image.length = plane0.length;
        global_image.mat = cv::Mat(HEIGHT, WIDTH, CV_8UC3, global_image.mmapped_memory);

    
        break;
    }
    std::cout << "bufferPairs: " << buffCount << std::endl;

    for (auto bufferPair : buffers) {
        FrameBuffer *buffer = bufferPair.second;
        const FrameMetadata &metadata = buffer->metadata();
    }

    request->reuse(Request::ReuseBuffers);
    // camera->queueRequest(request);
}

bool take_global_image() {
    ignore = false;
    count = 0;

    std::unique_ptr<CameraManager> cm = std::make_unique<CameraManager>();
    cm->start();
    for (auto const &camera : cm->cameras()) {
        std::cout << camera->id() << std::endl;
    }

    auto cameras = cm->cameras();
    if (cameras.empty()) {
        std::cout << "No cameras were identified on the system." << std::endl;
        cm->stop();
        return false;
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
            return false;
        }

        size_t allocated = allocator->buffers(cfg.stream()).size();
        std::cout << "Allocated " << allocated << " buffers for stream" << std::endl;
    }

    Stream *stream = streamConfig.stream();
    const std::vector<std::unique_ptr<FrameBuffer>> &buffers = allocator->buffers(stream);
    std::vector<std::unique_ptr<Request>> requests;

    for (unsigned int i = 0; i < buffers.size(); ++i) {
        std::unique_ptr<Request> request = camera->createRequest();
        if (!request) {
            std::cerr << "Can't create request" << std::endl;
            return false;
        }

        std::unique_ptr<CameraConfiguration> config = camera->generateConfiguration({StreamRole::Viewfinder});

        ControlList &controls = request->controls();
        controls.set(controls::AeEnable, true);
        controls.set(controls::AwbEnable, true);

        const std::unique_ptr<FrameBuffer> &buffer = buffers[i];
        int ret = request->addBuffer(stream, buffer.get());
        if (ret < 0) {
            std::cerr << "Can't set buffer for request" << std::endl;
            return ret;
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



    camera->stop();
    std::this_thread::sleep_for(100ms);
    allocator->free(stream);
    delete allocator;
    camera->release();
    camera.reset();
    cm->stop();

    return true;
}

void free_global_image() {
    if (global_image.mmapped_memory == nullptr) {
        return;
    }
    munmap(global_image.mmapped_memory, global_image.length);
    global_image.mmapped_memory = nullptr;
    global_image.length = 0;
    global_image.mat = cv::Mat();
}
