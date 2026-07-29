from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Camera 1: stellarHD - /dev/video4 (1600x1200 @ 60fps) - usb_cam
    stellarHD = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='stellarHD',
        namespace='stellarHD',
        parameters=[{
            'video_device': '/dev/video4',
            'image_width': 1600,
            'image_height': 1200,
            'pixel_format': 'mjpeg2rgb',
            'framerate': 60.0,
            'camera_name': 'stellarHD',
            'camera_info_url': 'file:///home/hero/.ros/camera_info/default_cam.yaml',
        }]
    )

    # Camera 2: exploreHD - /dev/video0 (1920x1080 @ 30fps) - gscam
    # image_encoding=jpeg → JPEG 디코딩 없이 CompressedImage로 직접 퍼블리시
    exploreHD = Node(
        package='gscam',
        executable='gscam_node',
        name='exploreHD',
        namespace='exploreHD',
        parameters=[{
            'camera_info_url': 'file:///home/hero/.ros/camera_info/explorehd_usb_camera:_explorehd.yaml',
            'gscam_config': 'v4l2src device=/dev/video0 ! image/jpeg,width=1920,height=1080,framerate=30/1',
            'image_encoding': 'jpeg',
            'sync_sink': True,
            'frame_id': 'exploreHD_frame',
        }]
    )

    return LaunchDescription([stellarHD, exploreHD])
