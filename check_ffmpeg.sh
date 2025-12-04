#!/bin/bash

echo "Checking FFmpeg installation..."
echo ""

# Check if ffmpeg command exists
if command -v ffmpeg &> /dev/null; then
    echo "✓ ffmpeg command found"
    ffmpeg -version | head -1
else
    echo "✗ ffmpeg command not found"
fi

echo ""
echo "Checking FFmpeg development libraries..."

# Check pkg-config
if command -v pkg-config &> /dev/null; then
    echo "✓ pkg-config found"
    
    echo ""
    echo "Checking FFmpeg libraries via pkg-config:"
    pkg-config --exists libavformat && echo "  ✓ libavformat" || echo "  ✗ libavformat"
    pkg-config --exists libavcodec && echo "  ✓ libavcodec" || echo "  ✗ libavcodec"
    pkg-config --exists libavutil && echo "  ✓ libavutil" || echo "  ✗ libavutil"
    pkg-config --exists libswscale && echo "  ✓ libswscale" || echo "  ✗ libswscale"
    
    echo ""
    echo "Library versions:"
    pkg-config --modversion libavformat 2>/dev/null && echo "  libavformat: $(pkg-config --modversion libavformat)" || echo "  libavformat: not found"
    pkg-config --modversion libavcodec 2>/dev/null && echo "  libavcodec: $(pkg-config --modversion libavcodec)" || echo "  libavcodec: not found"
    pkg-config --modversion libavutil 2>/dev/null && echo "  libavutil: $(pkg-config --modversion libavutil)" || echo "  libavutil: not found"
    pkg-config --modversion libswscale 2>/dev/null && echo "  libswscale: $(pkg-config --modversion libswscale)" || echo "  libswscale: not found"
    
    echo ""
    echo "Include directories:"
    pkg-config --cflags libavformat 2>/dev/null | grep -oP '(?<=-I)[^\s]+' | head -1 && echo "  $(pkg-config --cflags libavformat | grep -oP '(?<=-I)[^\s]+' | head -1)" || echo "  not found"
    
    echo ""
    echo "Library directories:"
    pkg-config --libs libavformat 2>/dev/null | grep -oP '(?<=-L)[^\s]+' | head -1 && echo "  $(pkg-config --libs libavformat | grep -oP '(?<=-L)[^\s]+' | head -1)" || echo "  not found"
else
    echo "✗ pkg-config not found"
fi

echo ""
echo "Checking system library paths:"
for lib in avformat avcodec avutil swscale; do
    if find /usr/lib* /usr/local/lib* -name "lib${lib}.so*" 2>/dev/null | head -1 > /dev/null; then
        echo "  ✓ lib${lib} found: $(find /usr/lib* /usr/local/lib* -name "lib${lib}.so*" 2>/dev/null | head -1)"
    else
        echo "  ✗ lib${lib} not found in standard paths"
    fi
done

echo ""
echo "Checking include files:"
for header in libavformat/avformat.h libavcodec/avcodec.h libavutil/avutil.h libswscale/swscale.h; do
    if find /usr/include /usr/local/include -name "$(basename $header)" 2>/dev/null | grep -q "$header"; then
        echo "  ✓ $header found"
    else
        echo "  ✗ $header not found"
    fi
done

echo ""
echo "Checking available decoders (if ffmpeg is installed):"
if command -v ffmpeg &> /dev/null; then
    echo "  H.264: $(ffmpeg -codecs 2>/dev/null | grep -i h264 | head -1 || echo 'not found')"
    echo "  H.265: $(ffmpeg -codecs 2>/dev/null | grep -i hevc | head -1 || echo 'not found')"
fi

