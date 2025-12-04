#ifndef VIDEO_DECODER_H
#define VIDEO_DECODER_H

#include <string>
#include <functional>
#include <cstdint>

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
}

/**
 * @brief Frame data structure containing decoded frame and timestamp information
 */
struct DecodedFrame {
    AVFrame* frame;              // Decoded frame
    int64_t pkt_pts;             // Packet PTS in stream time base units (100us)
    int64_t timestamp_ns;        // Final timestamp in nanoseconds (base + relative) * 1000
    bool valid;                   // Whether this frame is valid
};

/**
 * @brief Callback function type for processing decoded frames
 * @param frame Decoded frame data
 * @return true to continue decoding, false to stop
 */
using FrameCallback = std::function<bool(const DecodedFrame& frame)>;

/**
 * @brief Video decoder class for decoding MP4 files and extracting frames
 */
class VideoDecoder {
public:
    /**
     * @brief Constructor
     */
    VideoDecoder();

    /**
     * @brief Destructor
     */
    ~VideoDecoder();

    /**
     * @brief Initialize decoder with video file
     * @param file_path Path to the MP4 file
     * @param base_timestamp_us Base timestamp in microseconds from metadata
     * @return true if initialization successful, false otherwise
     */
    bool initialize(const std::string& file_path, int64_t base_timestamp_us);

    /**
     * @brief Decode all frames and call callback for each frame
     * @param callback Function to call for each decoded frame
     * @param target_fps Target FPS for frame extraction (0 = extract all frames)
     * @return true if decoding successful, false otherwise
     */
    bool decode_frames(FrameCallback callback, int target_fps = 0);

    /**
     * @brief Clean up resources
     */
    void cleanup();

    /**
     * @brief Get video stream time base
     * @return Time base of the video stream
     */
    AVRational get_time_base() const { return time_base_; }

    /**
     * @brief Get original video frame rate (FPS)
     * @return Frame rate in frames per second, or 0 if not available
     */
    double get_original_fps() const;

private:
    /**
     * @brief Open codec context
     * @return true if successful, false otherwise
     */
    bool open_codec();

    /**
     * @brief Convert PTS to microseconds
     * @param pts Packet PTS in stream time base units
     * @return Timestamp in microseconds
     */
    int64_t pts_to_microseconds(int64_t pts) const;

    /**
     * @brief Check if frame should be extracted based on target FPS
     * @param current_pts Current frame PTS
     * @return true if frame should be extracted, false otherwise
     */
    bool should_extract_frame(int64_t current_pts);

    std::string file_path_;
    int64_t base_timestamp_us_;
    int target_fps_;
    
    AVFormatContext* format_ctx_;
    AVCodecContext* codec_ctx_;
    AVFrame* frame_;
    AVPacket* packet_;
    
    int video_stream_index_;
    AVRational time_base_;
    AVStream* video_stream_;  // Video stream pointer
    
    int64_t last_extracted_pts_;
    double frame_interval_;  // In time base units
    
    bool initialized_;
};

#endif // VIDEO_DECODER_H

