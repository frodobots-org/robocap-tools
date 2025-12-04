#ifndef FRAME_WRITER_H
#define FRAME_WRITER_H

#include <string>
#include <cstdint>

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
}

/**
 * @brief Writes decoded video frames to PNG files
 */
class FrameWriter {
public:
    /**
     * @brief Constructor
     */
    FrameWriter();

    /**
     * @brief Destructor
     */
    ~FrameWriter();

    /**
     * @brief Initialize frame writer
     * @param output_dir Output directory for PNG files
     * @param width Frame width
     * @param height Frame height
     * @param pix_fmt Source pixel format
     * @return true if initialization successful, false otherwise
     */
    bool initialize(const std::string& output_dir, int width, int height, AVPixelFormat pix_fmt);

    /**
     * @brief Write frame to PNG file
     * @param frame Frame to write
     * @param timestamp_ns Timestamp in nanoseconds for filename
     * @return true if successful, false otherwise
     */
    bool write_frame(AVFrame* frame, int64_t timestamp_ns);

    /**
     * @brief Clean up resources
     */
    void cleanup();

private:
    /**
     * @brief Convert frame to RGB format
     * @param frame Input frame
     * @return Converted RGB frame, or nullptr on error
     */
    AVFrame* convert_to_rgb(AVFrame* frame);

    /**
     * @brief Write RGB frame to PNG file using stb_image_write or similar
     * @param frame RGB frame
     * @param filename Output filename
     * @return true if successful, false otherwise
     */
    bool write_png(AVFrame* frame, const std::string& filename);
    bool write_png(AVFrame* frame, const char* filename);  // Overload for better performance

    std::string output_dir_;
    int width_;
    int height_;
    AVPixelFormat src_pix_fmt_;
    
    SwsContext* sws_ctx_;
    AVFrame* frame_rgb_;
    uint8_t* rgb_buffer_;
    int rgb_buffer_size_;
    
    bool initialized_;
};

#endif // FRAME_WRITER_H

