#include <iostream>
#include <string>
#include <cstring>
#include <sys/stat.h>
#include <sys/types.h>
#ifdef _WIN32
#include <direct.h>
#include <io.h>
#define mkdir(path, mode) _mkdir(path)
#define PATH_SEP "\\"
#else
#include <unistd.h>
#define PATH_SEP "/"
#endif
#include "metadata/metadata_extractor.h"
#include "decoder/video_decoder.h"
#include "frame/frame_writer.h"

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
}

/**
 * @brief Print usage information
 */
void print_usage(const char* program_name) {
    std::cerr << "Usage: " << program_name << " -i <input_mp4> -o <output_dir> [-r <target_fps>]" << std::endl;
    std::cerr << "Options:" << std::endl;
    std::cerr << "  -i <file>    Input MP4 file path (required)" << std::endl;
    std::cerr << "  -o <dir>     Output directory for PNG files (required)" << std::endl;
    std::cerr << "  -r <fps>     Target FPS for frame extraction (optional, default: extract all frames)" << std::endl;
    std::cerr << "               If 0 or greater than original FPS, extracts all frames" << std::endl;
    std::cerr << "               If > 0 and < original FPS, extracts frames at target FPS" << std::endl;
    std::cerr << "Example: " << program_name << " -i video.mp4 -o output/ -r 10" << std::endl;
}

/**
 * @brief Parse command line arguments
 */
bool parse_arguments(int argc, char* argv[], int& target_fps, std::string& input_file, std::string& output_dir) {
    target_fps = 0;
    input_file.clear();
    output_dir.clear();
    
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        
        if (arg == "-r" && i + 1 < argc) {
            target_fps = std::stoi(argv[++i]);
            if (target_fps < 0) {
                std::cerr << "Error: Target FPS must be >= 0" << std::endl;
                return false;
            }
        } else if (arg == "-i" && i + 1 < argc) {
            input_file = argv[++i];
        } else if (arg == "-o" && i + 1 < argc) {
            output_dir = argv[++i];
        } else if (arg == "-h" || arg == "--help") {
            print_usage(argv[0]);
            return false;
        } else {
            std::cerr << "Error: Unknown option " << arg << std::endl;
            return false;
        }
    }
    
    if (input_file.empty()) {
        std::cerr << "Error: -i (input file) is required" << std::endl;
        return false;
    }
    
    if (output_dir.empty()) {
        std::cerr << "Error: -o (output directory) is required" << std::endl;
        return false;
    }
    
    return true;
}

/**
 * @brief Main function for extracting frames from MP4 files
 */
int main(int argc, char* argv[]) {
    // Parse command line arguments
    int target_fps;
    std::string mp4_file;
    std::string output_dir;
    
    if (!parse_arguments(argc, argv, target_fps, mp4_file, output_dir)) {
        print_usage(argv[0]);
        return 1;
    }

    // Initialize FFmpeg with minimal logging for better performance
    av_log_set_level(AV_LOG_PANIC);  // Only show critical errors

    // Extract base timestamp from metadata
    MetadataExtractor metadata_extractor;
    int64_t base_timestamp_us = 0;

    std::cout << "Extracting base timestamp from metadata..." << std::endl;
    if (!metadata_extractor.extract_base_timestamp(mp4_file, base_timestamp_us)) {
        std::cerr << "Error: Failed to extract base timestamp" << std::endl;
        return 1;
    }

    std::cout << "Base timestamp: " << base_timestamp_us << " microseconds" << std::endl;

    // Get time base for verification
    AVRational time_base;
    if (!metadata_extractor.get_time_base(mp4_file, time_base)) {
        std::cerr << "Error: Failed to get time base" << std::endl;
        return 1;
    }

    std::cout << "Time base: " << time_base.num << "/" << time_base.den 
              << " (units of " << (1000000.0 * av_q2d(time_base)) << " microseconds)" << std::endl;

    // Initialize video decoder
    VideoDecoder decoder;
    std::cout << "Initializing video decoder..." << std::endl;
    if (!decoder.initialize(mp4_file, base_timestamp_us)) {
        std::cerr << "Error: Failed to initialize decoder" << std::endl;
        return 1;
    }

    // Get original video FPS
    double original_fps = decoder.get_original_fps();
    std::cout << "Original video FPS: " << original_fps << std::endl;

    // Determine actual target FPS for extraction
    // If target_fps is 0, or >= original_fps, extract all frames (use 0)
    // Otherwise, use target_fps for frame extraction
    int actual_target_fps = 0;
    if (target_fps > 0 && original_fps > 0 && target_fps < original_fps) {
        actual_target_fps = target_fps;
        std::cout << "Extracting frames at " << actual_target_fps << " FPS (downsampled from " << original_fps << " FPS)" << std::endl;
    } else {
        std::cout << "Extracting all frames at original FPS (" << original_fps << " FPS)" << std::endl;
    }

    // Create output directory
    mkdir(output_dir.c_str(), 0755);
    std::cout << "Output directory: " << output_dir << std::endl;

    // Initialize frame writer (we'll get dimensions from first frame)
    FrameWriter frame_writer;
    bool frame_writer_initialized = false;
    int frame_count = 0;

    // Decode frames
    std::cout << "Starting frame extraction..." << std::endl;
    
    bool success = decoder.decode_frames([&](const DecodedFrame& decoded_frame) -> bool {
        if (!decoded_frame.valid) {
            return true;
        }

        // Initialize frame writer on first frame
        if (!frame_writer_initialized) {
            AVFrame* frame = decoded_frame.frame;
            if (!frame_writer.initialize(output_dir, frame->width, frame->height, 
                                        (AVPixelFormat)frame->format)) {
                std::cerr << "Error: Failed to initialize frame writer" << std::endl;
                return false;
            }
            frame_writer_initialized = true;
        }

        // Write frame (timestamp is already in nanoseconds)
        if (!frame_writer.write_frame(decoded_frame.frame, decoded_frame.timestamp_ns)) {
            std::cerr << "Error: Failed to write frame" << std::endl;
            return false;
        }

        frame_count++;
        if (frame_count % 10 == 0) {
            std::cout << "Processed " << frame_count << " frames..." << std::endl;
        }

        return true;  // Continue decoding
    }, actual_target_fps);

    if (!success) {
        std::cerr << "Error: Frame decoding failed" << std::endl;
        decoder.cleanup();
        frame_writer.cleanup();
        return 1;
    }

    std::cout << "Successfully extracted " << frame_count << " frames to " << output_dir << std::endl;

    // Cleanup
    decoder.cleanup();
    frame_writer.cleanup();

    return 0;
}

