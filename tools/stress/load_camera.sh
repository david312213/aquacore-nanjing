#!/bin/sh
# Ground-only: publish the down camera on /down_camera/image_raw in a format the camera
# supports, for cameras that cannot do the DF200 MJPEG 1280x720 mode. load_red_point.sh then
# reuses this publisher instead of starting df200_camera.launch.
# Usage: sh load_camera.sh [device] [width] [height] [pixel_format]
set -e
project_dir=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
device=${1:-/dev/v4l/by-id/usb-HD_Camera_Manufacturer_USB_2.0_Camera-video-index0}
. /opt/ros/noetic/setup.sh
. "$project_dir/build_ws/devel/setup.sh"
export ROS_MASTER_URI=http://127.0.0.1:11341
. "$project_dir/scripts/ros_network.sh"
exec rosrun usb_cam usb_cam_node __name:=down_camera "_video_device:=$device" \
    "_image_width:=${2:-1280}" "_image_height:=${3:-720}" "_pixel_format:=${4:-mjpeg}" \
    _framerate:=30 _io_method:=mmap _camera_frame_id:=down_camera_optical \
    /usb_cam/image_raw:=/down_camera/image_raw /usb_cam/camera_info:=/down_camera/camera_info
