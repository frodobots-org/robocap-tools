#include "video_decoder.h"
#include <iostream>
#include <cmath>

VideoDecoder::VideoDecoder()
    : base_timestamp_us_(0)
    , target_fps_(0)
    , format_ctx_(nullptr)
    , codec_ctx_(nullptr)
    , frame_(nullptr)
    , packet_(nullptr)
    , video_stream_index_(-1)
    , video_stream_(nullptr)
    , last_extracted_pts_(-1)
    , frame_interval_(0.0)
    , initialized_(false)
{
    time_base_ = {0, 1};
}

VideoDecoder::~VideoDecoder() {
    cleanup();
}

bool VideoDecoder::initialize(const std::string& file_path, int64_t base_timestamp_us) {
    file_path_ = file_path;
    base_timestamp_us_ = base_timestamp_us;
    
    // Open input file
    if (avformat_open_input(&format_ctx_, file_path.c_str(), nullptr, nullptr) < 0) {
        std::cerr << "Error: Could not open file " << file_path << std::endl;
        return false;
    }

    // Read stream information
    if (avformat_find_stream_info(format_ctx_, nullptr) < 0) {
        std::cerr << "Error: Could not find stream info" << std::endl;
        cleanup();
        return false;
    }

    // Find video stream
    video_stream_index_ = -1;
    for (unsigned int i = 0; i < format_ctx_->nb_streams; i++) {
        if (format_ctx_->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
            video_stream_index_ = i;
            break;
        }
    }

    if (video_stream_index_ < 0) {
        std::cerr << "Error: No video stream found" << std::endl;
        cleanup();
        return false;
    }

    // Get time base and store video stream
    video_stream_ = format_ctx_->streams[video_stream_index_];
    time_base_ = video_stream_->time_base;

    // Get codec parameters
    AVCodecParameters* codecpar = format_ctx_->streams[video_stream_index_]->codecpar;
    
    // Find decoder
    const AVCodec* codec = avcodec_find_decoder(codecpar->codec_id);
    if (codec == nullptr) {
        std::cerr << "Error: Codec not found" << std::endl;
        cleanup();
        return false;
    }

    // Allocate codec context
    codec_ctx_ = avcodec_alloc_context3(codec);
    if (codec_ctx_ == nullptr) {
        std::cerr << "Error: Could not allocate codec context" << std::endl;
        cleanup();
        return false;
    }

    // Copy codec parameters to context
    if (avcodec_parameters_to_context(codec_ctx_, codecpar) < 0) {
        std::cerr << "Error: Could not copy codec parameters" << std::endl;
        cleanup();
        return false;
    }

    // Enable multi-threaded decoding for better performance
    // Use 0 to let FFmpeg automatically determine optimal thread count
    codec_ctx_->thread_count = 0;  // Auto-detect CPU cores
    codec_ctx_->thread_type = FF_THREAD_FRAME | FF_THREAD_SLICE;  // Use both frame and slice threading
    
    // Additional performance optimizations
    codec_ctx_->flags |= AV_CODEC_FLAG_LOW_DELAY;  // Low latency decoding
    codec_ctx_->err_recognition = 0;  // Disable error recognition for speed

    // Open codec
    if (!open_codec()) {
        cleanup();
        return false;
    }

    // Allocate frame (only decode frame, RGB conversion happens in writer)
    frame_ = av_frame_alloc();
    if (frame_ == nullptr) {
        std::cerr << "Error: Could not allocate frame" << std::endl;
        cleanup();
        return false;
    }

    // Allocate packet
    packet_ = av_packet_alloc();
    if (packet_ == nullptr) {
        std::cerr << "Error: Could not allocate packet" << std::endl;
        cleanup();
        return false;
    }

    // Calculate frame interval for target FPS
    if (target_fps_ > 0) {
        // Calculate interval in time base units
        // For 10 fps, interval = 1/10 seconds
        // In time base units: (1/10) / time_base
        double interval_seconds = 1.0 / target_fps_;
        frame_interval_ = interval_seconds / av_q2d(time_base_);
    }

    initialized_ = true;
    return true;
}

bool VideoDecoder::open_codec() {
    if (avcodec_open2(codec_ctx_, nullptr, nullptr) < 0) {
        std::cerr << "Error: Could not open codec" << std::endl;
        return false;
    }
    return true;
}

bool VideoDecoder::decode_frames(FrameCallback callback, int target_fps) {
    if (!initialized_) {
        std::cerr << "Error: Decoder not initialized" << std::endl;
        return false;
    }

    target_fps_ = target_fps;
    last_extracted_pts_ = -1;

    // Calculate frame interval if target FPS is specified
    if (target_fps_ > 0) {
        double interval_seconds = 1.0 / target_fps_;
        frame_interval_ = interval_seconds / av_q2d(time_base_);
    }

    // Read frames
    while (av_read_frame(format_ctx_, packet_) >= 0) {
        if (packet_->stream_index == video_stream_index_) {
            // Early skip: if target FPS is set, check packet PTS before decoding
            // This can save significant time by skipping unnecessary decoding
            if (target_fps_ > 0 && packet_->pts != AV_NOPTS_VALUE) {
                if (!should_extract_frame(packet_->pts)) {
                    av_packet_unref(packet_);
                    continue;  // Skip this packet without decoding
                }
            }
            
            // Send packet to decoder
            if (avcodec_send_packet(codec_ctx_, packet_) < 0) {
                std::cerr << "Error: Error sending packet to decoder" << std::endl;
                av_packet_unref(packet_);
                continue;
            }

            // Receive frames from decoder
            while (avcodec_receive_frame(codec_ctx_, frame_) >= 0) {
                // Double-check frame PTS (in case packet PTS was not available)
                if (target_fps_ > 0 && !should_extract_frame(frame_->pkt_dts != AV_NOPTS_VALUE ? frame_->pkt_dts : frame_->pts)) {
                    continue;
                }

                // Create decoded frame structure
                DecodedFrame decoded_frame;
                decoded_frame.frame = frame_;
                decoded_frame.pkt_pts = frame_->pkt_dts != AV_NOPTS_VALUE ? frame_->pkt_dts : frame_->pts;
                
                // Convert PTS from 100us units to microseconds (multiply by 10)
                // Then add base timestamp (in microseconds)
                // Finally convert to nanoseconds (multiply by 1000) for filename
                int64_t relative_timestamp_us = pts_to_microseconds(decoded_frame.pkt_pts);
                int64_t total_timestamp_us = base_timestamp_us_ + relative_timestamp_us;
                decoded_frame.timestamp_ns = total_timestamp_us * 1000;
                decoded_frame.valid = true;

                // Call callback
                if (!callback(decoded_frame)) {
                    av_packet_unref(packet_);
                    return true;  // Callback requested to stop
                }

                last_extracted_pts_ = decoded_frame.pkt_pts;
            }
        }
        av_packet_unref(packet_);
    }

    // Flush decoder
    avcodec_send_packet(codec_ctx_, nullptr);
    while (avcodec_receive_frame(codec_ctx_, frame_) >= 0) {
        if (target_fps_ > 0 && !should_extract_frame(frame_->pkt_dts != AV_NOPTS_VALUE ? frame_->pkt_dts : frame_->pts)) {
            continue;
        }

        DecodedFrame decoded_frame;
        decoded_frame.frame = frame_;
        decoded_frame.pkt_pts = frame_->pkt_dts != AV_NOPTS_VALUE ? frame_->pkt_dts : frame_->pts;
        int64_t relative_timestamp_us = pts_to_microseconds(decoded_frame.pkt_pts);
        int64_t total_timestamp_us = base_timestamp_us_ + relative_timestamp_us;
        decoded_frame.timestamp_ns = total_timestamp_us * 1000;
        decoded_frame.valid = true;

        if (!callback(decoded_frame)) {
            return true;
        }
    }

    return true;
}

int64_t VideoDecoder::pts_to_microseconds(int64_t pts) const {
    if (pts == AV_NOPTS_VALUE) {
        return 0;
    }
    
    // pkt_pts is in units of 100us (time base is 1/10000, meaning 100us per unit)
    // So we multiply by 10 to convert from 100us units to microseconds
    // Example: if pts = 100, that means 100 * 100us = 10000us, so we multiply by 10
    return pts * 10;
}

double VideoDecoder::get_original_fps() const {
    if (!initialized_ || video_stream_ == nullptr) {
        return 0.0;
    }
    
    // Try to get frame rate from stream
    AVRational frame_rate = video_stream_->r_frame_rate;
    
    // If r_frame_rate is not valid, try avg_frame_rate
    if (frame_rate.num == 0 || frame_rate.den == 0) {
        frame_rate = video_stream_->avg_frame_rate;
    }
    
    // If still not valid, return 0
    if (frame_rate.num == 0 || frame_rate.den == 0) {
        return 0.0;
    }
    
    // Convert to double FPS
    return av_q2d(frame_rate);
}

bool VideoDecoder::should_extract_frame(int64_t current_pts) {
    if (target_fps_ <= 0) {
        return true;  // Extract all frames
    }

    if (last_extracted_pts_ < 0) {
        return true;  // Extract first frame
    }

    // Check if enough time has passed since last extraction
    int64_t pts_diff = current_pts - last_extracted_pts_;
    return pts_diff >= static_cast<int64_t>(frame_interval_ * 0.9);  // 90% threshold to account for rounding
}

void VideoDecoder::cleanup() {
    if (frame_) {
        av_frame_free(&frame_);
    }

    if (packet_) {
        av_packet_free(&packet_);
    }

    if (codec_ctx_) {
        avcodec_free_context(&codec_ctx_);
    }

    if (format_ctx_) {
        avformat_close_input(&format_ctx_);
    }

    initialized_ = false;
}

