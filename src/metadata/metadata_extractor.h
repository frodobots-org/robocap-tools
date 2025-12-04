#ifndef METADATA_EXTRACTOR_H
#define METADATA_EXTRACTOR_H

#include <string>
#include <cstdint>

extern "C" {
#include <libavformat/avformat.h>
}

/**
 * @brief Extracts metadata from video files, specifically the comment field
 *        which contains the base timestamp in microseconds.
 */
class MetadataExtractor {
public:
    /**
     * @brief Constructor
     */
    MetadataExtractor();

    /**
     * @brief Destructor
     */
    ~MetadataExtractor();

    /**
     * @brief Extract the base timestamp from MP4 comment metadata
     * @param file_path Path to the MP4 file
     * @param base_timestamp_us Output parameter for base timestamp in microseconds
     * @return true if extraction successful, false otherwise
     */
    bool extract_base_timestamp(const std::string& file_path, int64_t& base_timestamp_us);

    /**
     * @brief Get the time base of the video stream
     * @param file_path Path to the MP4 file
     * @param time_base Output parameter for time base (numerator/denominator)
     * @return true if successful, false otherwise
     */
    bool get_time_base(const std::string& file_path, AVRational& time_base);

private:
    /**
     * @brief Parse comment string to int64_t timestamp
     * @param comment Comment string from metadata
     * @param timestamp_us Output timestamp in microseconds
     * @return true if parsing successful, false otherwise
     */
    bool parse_comment_to_timestamp(const std::string& comment, int64_t& timestamp_us);
};

#endif // METADATA_EXTRACTOR_H

