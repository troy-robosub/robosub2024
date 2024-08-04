#!/bin/bash

./mediamtmx

if dpkg -l | grep -q "^ii  ffmpeg "; then
    echo "Package ffmpeg is installed."
else
    echo "Package ffmpeg is not installed."
    exit 1
fi

# Check if an IP address is provided as an argument
if [ -z "$1" ]; then
    echo "Usage: $0 <RTSP_IP_ADDRESS>"
    exit 1
fi

RTSP_IP=$1

# command kind of depends on the IP address 
sudo ffmpeg -f v4l2 -input_format h264 -timestamps abs -video_size hd720 -r 15 -i /dev/video2 -c:v copy -c:a none -f rtsp rtsp://$RTSP_IP:8554/cameraTx1

echo "Connect to the video stream with the link rtsp://$RTSP_IP:8554/cameraTx1"