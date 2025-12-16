#!/bin/bash

get_mp4_comment_uint64() {
    if [ $# -ne 1 ]; then
        echo "ERROR: Invalid arguments! Usage: get_mp4_comment_uint64 <MP4_FILE_PATH>"
        return 1
    fi

    local mp4_path="$1"
    if [ ! -f "$mp4_path" ]; then
        echo "ERROR: File not found: $mp4_path"
        return 1
    fi

    if ! command -v ffprobe &> /dev/null; then
        echo "ERROR: ffprobe not found! Install ffmpeg first."
        return 1
    fi

    local comment_value
    comment_value=$(ffprobe -v error -show_format -i "$mp4_path" 2>/dev/null | \
                   grep -E "^TAG:comment=" | \
                   cut -d "=" -f 2 | \
                   sed -e 's/^[[:space:]]*//' -e 's/[[:space:]]*$//')

    if [ -z "$comment_value" ]; then
        echo "ERROR: TAG:comment field not found in $mp4_path"
        return 1
    fi

    local uint64_max="18446744073709551615"
    if ! [[ "$comment_value" =~ ^[0-9]+$ ]]; then
        echo "ERROR: TAG:comment is not an integer! Value: $comment_value"
        return 1
    fi

    if [ $(echo "$comment_value > $uint64_max" | bc) -eq 1 ]; then
        echo "ERROR: TAG:comment exceeds uint64 range! Value: $comment_value (Max: $uint64_max)"
        return 1
    fi

    echo "$comment_value"
    return 0
}

get_mp4_comment_uint64 $1
