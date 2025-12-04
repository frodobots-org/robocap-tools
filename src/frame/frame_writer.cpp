#include "frame_writer.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cstring>
#include <cstdio>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

FrameWriter::FrameWriter()
    : width_(0)
    , height_(0)
    , src_pix_fmt_(AV_PIX_FMT_NONE)
    , sws_ctx_(nullptr)
    , frame_rgb_(nullptr)
    , rgb_buffer_(nullptr)
    , rgb_buffer_size_(0)
    , initialized_(false)
{
}

FrameWriter::~FrameWriter() {
    cleanup();
}

bool FrameWriter::initialize(const std::string& output_dir, int width, int height, AVPixelFormat pix_fmt) {
    output_dir_ = output_dir;
    width_ = width;
    height_ = height;
    src_pix_fmt_ = pix_fmt;

    // Allocate RGB frame
    frame_rgb_ = av_frame_alloc();
    if (frame_rgb_ == nullptr) {
        std::cerr << "Error: Could not allocate RGB frame" << std::endl;
        return false;
    }

    // Calculate buffer size for RGB frame
    rgb_buffer_size_ = av_image_get_buffer_size(AV_PIX_FMT_RGB24, width, height, 1);
    if (rgb_buffer_size_ < 0) {
        std::cerr << "Error: Could not calculate buffer size" << std::endl;
        cleanup();
        return false;
    }

    // Allocate buffer
    rgb_buffer_ = (uint8_t*)av_malloc(rgb_buffer_size_);
    if (rgb_buffer_ == nullptr) {
        std::cerr << "Error: Could not allocate RGB buffer" << std::endl;
        cleanup();
        return false;
    }

    // Fill frame with buffer
    if (av_image_fill_arrays(frame_rgb_->data, frame_rgb_->linesize, rgb_buffer_,
                             AV_PIX_FMT_RGB24, width, height, 1) < 0) {
        std::cerr << "Error: Could not fill image arrays" << std::endl;
        cleanup();
        return false;
    }

    frame_rgb_->width = width;
    frame_rgb_->height = height;
    frame_rgb_->format = AV_PIX_FMT_RGB24;

    initialized_ = true;
    return true;
}

bool FrameWriter::write_frame(AVFrame* frame, int64_t timestamp_ns) {
    if (!initialized_) {
        std::cerr << "Error: Frame writer not initialized" << std::endl;
        return false;
    }

    // Convert frame to RGB
    AVFrame* rgb_frame = convert_to_rgb(frame);
    if (rgb_frame == nullptr) {
        return false;
    }

    // Generate filename from timestamp (in nanoseconds) - use faster method
    char filename[512];
    snprintf(filename, sizeof(filename), "%s/%ld.png", output_dir_.c_str(), timestamp_ns);

    // Write PNG
    bool success = write_png(rgb_frame, filename);

    return success;
}

AVFrame* FrameWriter::convert_to_rgb(AVFrame* frame) {
    // Initialize or update sws context with optimized flags for speed
    // SWS_FAST_BILINEAR is faster than SWS_BILINEAR with minimal quality loss
    // Remove SWS_ACCURATE_RND for better performance (minimal quality impact)
    int flags = SWS_FAST_BILINEAR;
    
    sws_ctx_ = sws_getCachedContext(sws_ctx_,
                                    frame->width, frame->height, (AVPixelFormat)frame->format,
                                    width_, height_, AV_PIX_FMT_RGB24,
                                    flags, nullptr, nullptr, nullptr);

    if (sws_ctx_ == nullptr) {
        std::cerr << "Error: Could not create sws context" << std::endl;
        return nullptr;
    }

    // Convert frame
    sws_scale(sws_ctx_,
              frame->data, frame->linesize, 0, frame->height,
              frame_rgb_->data, frame_rgb_->linesize);

    return frame_rgb_;
}

bool FrameWriter::write_png(AVFrame* frame, const std::string& filename) {
    return write_png(frame, filename.c_str());
}

bool FrameWriter::write_png(AVFrame* frame, const char* filename) {
    // Write PNG using stb_image_write
    // stb_image_write expects data in format: R, G, B, R, G, B, ...
    int stride = frame->linesize[0];
    int result = stbi_write_png(filename, width_, height_, 3,
                                frame->data[0], stride);

    if (result == 0) {
        std::cerr << "Error: Failed to write PNG file " << filename << std::endl;
        return false;
    }

    return true;
}

void FrameWriter::cleanup() {
    if (sws_ctx_) {
        sws_freeContext(sws_ctx_);
        sws_ctx_ = nullptr;
    }

    if (rgb_buffer_) {
        av_free(rgb_buffer_);
        rgb_buffer_ = nullptr;
    }

    if (frame_rgb_) {
        av_frame_free(&frame_rgb_);
    }

    initialized_ = false;
}

