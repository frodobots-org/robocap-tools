cmake_minimum_required(VERSION 3.8)
project(MP4FrameExtractor)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find FFmpeg components
# Try multiple methods to find FFmpeg

# Method 1: Try pkg-config (Linux/Mac, if available)
find_package(PkgConfig QUIET)
set(FFMPEG_FOUND FALSE)

if(PKG_CONFIG_FOUND)
    pkg_check_modules(FFMPEG libavformat libavcodec libavutil libswscale)
    if(FFMPEG_FOUND)
        message(STATUS "Found FFmpeg via pkg-config")
    endif()
endif()

# Method 2: Try common system paths (Linux)
if(NOT FFMPEG_FOUND)
    find_path(FFMPEG_INCLUDE_DIRS
        NAMES libavformat/avformat.h
        PATHS
            /usr/include
            /usr/local/include
            /opt/local/include
            /sw/include
    )
    
    if(FFMPEG_INCLUDE_DIRS)
        find_library(AVFORMAT_LIB
            NAMES avformat
            PATHS
                /usr/lib
                /usr/local/lib
                /opt/local/lib
                /sw/lib
                /usr/lib/x86_64-linux-gnu
        )
        
        find_library(AVCODEC_LIB
            NAMES avcodec
            PATHS
                /usr/lib
                /usr/local/lib
                /opt/local/lib
                /sw/lib
                /usr/lib/x86_64-linux-gnu
        )
        
        find_library(AVUTIL_LIB
            NAMES avutil
            PATHS
                /usr/lib
                /usr/local/lib
                /opt/local/lib
                /sw/lib
                /usr/lib/x86_64-linux-gnu
        )
        
        find_library(SWSCALE_LIB
            NAMES swscale
            PATHS
                /usr/lib
                /usr/local/lib
                /opt/local/lib
                /sw/lib
                /usr/lib/x86_64-linux-gnu
        )
        
        if(AVFORMAT_LIB AND AVCODEC_LIB AND AVUTIL_LIB AND SWSCALE_LIB)
            set(FFMPEG_LIBRARIES ${AVFORMAT_LIB} ${AVCODEC_LIB} ${AVUTIL_LIB} ${SWSCALE_LIB})
            set(FFMPEG_FOUND TRUE)
            message(STATUS "Found FFmpeg in system paths")
        endif()
    endif()
endif()

# Method 3: Try FFMPEG_ROOT (user-specified path)
if(NOT FFMPEG_FOUND)
    set(FFMPEG_ROOT "$ENV{FFMPEG_ROOT}" CACHE PATH "FFmpeg root directory")
    
    if(FFMPEG_ROOT)
        set(FFMPEG_INCLUDE_DIRS "${FFMPEG_ROOT}/include")
        set(FFMPEG_LIBRARY_DIRS "${FFMPEG_ROOT}/lib")
        
        find_library(AVFORMAT_LIB avformat PATHS ${FFMPEG_LIBRARY_DIRS} NO_DEFAULT_PATH)
        find_library(AVCODEC_LIB avcodec PATHS ${FFMPEG_LIBRARY_DIRS} NO_DEFAULT_PATH)
        find_library(AVUTIL_LIB avutil PATHS ${FFMPEG_LIBRARY_DIRS} NO_DEFAULT_PATH)
        find_library(SWSCALE_LIB swscale PATHS ${FFMPEG_LIBRARY_DIRS} NO_DEFAULT_PATH)
        
        if(AVFORMAT_LIB AND AVCODEC_LIB AND AVUTIL_LIB AND SWSCALE_LIB)
            set(FFMPEG_LIBRARIES ${AVFORMAT_LIB} ${AVCODEC_LIB} ${AVUTIL_LIB} ${SWSCALE_LIB})
            set(FFMPEG_FOUND TRUE)
            message(STATUS "Found FFmpeg in FFMPEG_ROOT: ${FFMPEG_ROOT}")
        endif()
    endif()
endif()

# Final check
if(NOT FFMPEG_FOUND)
    message(FATAL_ERROR "\n"
        "FFmpeg not found!\n"
        "Please install FFmpeg development libraries:\n"
        "  Ubuntu/Debian: sudo apt-get install libavformat-dev libavcodec-dev libavutil-dev libswscale-dev\n"
        "  Fedora: sudo dnf install ffmpeg-devel\n"
        "  macOS: brew install ffmpeg\n"
        "  Or set FFMPEG_ROOT environment variable to point to your FFmpeg installation\n"
    )
endif()

# Include directories
include_directories(
    ${FFMPEG_INCLUDE_DIRS}
    src
)

# Source files
set(SOURCES
    src/main.cpp
    src/metadata/metadata_extractor.cpp
    src/decoder/video_decoder.cpp
    src/frame/frame_writer.cpp
)

# Add executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Link libraries
target_link_libraries(${PROJECT_NAME}
    ${FFMPEG_LIBRARIES}
)

# Add stb_image_write
# Download stb_image_write.h if not present
set(STB_IMAGE_WRITE_H "${CMAKE_CURRENT_SOURCE_DIR}/third_party/stb_image_write.h")
if(NOT EXISTS "${STB_IMAGE_WRITE_H}")
    message(STATUS "Downloading stb_image_write.h...")
    file(DOWNLOAD
        https://raw.githubusercontent.com/nothings/stb/master/stb_image_write.h
        ${STB_IMAGE_WRITE_H}
        SHOW_PROGRESS
        TLS_VERIFY ON
    )
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/third_party)

# Compiler flags
target_compile_options(${PROJECT_NAME} PRIVATE ${FFMPEG_CFLAGS_OTHER})

# Set output directory
set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

