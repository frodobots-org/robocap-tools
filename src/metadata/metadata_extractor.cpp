#include "metadata_extractor.h"
#include <iostream>
#include <sstream>
#include <cstdlib>

MetadataExtractor::MetadataExtractor() {
}

MetadataExtractor::~MetadataExtractor() {
}

bool MetadataExtractor::extract_base_timestamp(const std::string& file_path, int64_t& base_timestamp_us) {
    AVFormatContext* format_ctx = nullptr;
    
    // Open input file
    if (avformat_open_input(&format_ctx, file_path.c_str(), nullptr, nullptr) < 0) {
        std::cerr << "Error: Could not open file " << file_path << std::endl;
        return false;
    }

    // Read metadata
    if (avformat_find_stream_info(format_ctx, nullptr) < 0) {
        std::cerr << "Error: Could not find stream info" << std::endl;
        avformat_close_input(&format_ctx);
        return false;
    }

    // Extract comment from metadata
    AVDictionaryEntry* comment_entry = av_dict_get(format_ctx->metadata, "comment", nullptr, 0);
    
    if (comment_entry == nullptr || comment_entry->value == nullptr) {
        std::cerr << "Error: Comment metadata not found" << std::endl;
        avformat_close_input(&format_ctx);
        return false;
    }

    std::string comment(comment_entry->value);
    bool success = parse_comment_to_timestamp(comment, base_timestamp_us);

    avformat_close_input(&format_ctx);
    return success;
}

bool MetadataExtractor::get_time_base(const std::string& file_path, AVRational& time_base) {
    AVFormatContext* format_ctx = nullptr;
    
    // Open input file
    if (avformat_open_input(&format_ctx, file_path.c_str(), nullptr, nullptr) < 0) {
        std::cerr << "Error: Could not open file " << file_path << std::endl;
        return false;
    }

    // Find video stream
    int video_stream_index = -1;
    for (unsigned int i = 0; i < format_ctx->nb_streams; i++) {
        if (format_ctx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
            video_stream_index = i;
            break;
        }
    }

    if (video_stream_index < 0) {
        std::cerr << "Error: No video stream found" << std::endl;
        avformat_close_input(&format_ctx);
        return false;
    }

    // Get time base from video stream
    time_base = format_ctx->streams[video_stream_index]->time_base;

    avformat_close_input(&format_ctx);
    return true;
}

bool MetadataExtractor::parse_comment_to_timestamp(const std::string& comment, int64_t& timestamp_us) {
    try {
        // Parse string to int64_t
        timestamp_us = std::stoll(comment);
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error: Failed to parse comment to timestamp: " << e.what() << std::endl;
        return false;
    }
}

