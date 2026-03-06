from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    # Node 1: Camera
    camera_node = Node(
        package='aavc',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[{
            # ตัวอย่าง param กล้อง (ถ้ามี)
            # 'video_device': '/dev/video0',
            # 'image_width': 640,
            # 'image_height': 480,
        }]
    )

    # Node 2: YOLO Detection (เริ่มหลังกล้อง 5 วินาที)
    detection_node = Node(
        package='aavc',
        executable='detection_node',
        name='detection_node',
        output='screen',
        parameters=[{
            # topic ของภาพจากกล้อง
            'image_topic': '/camera/image_raw',
            # topic ของ GPS
            'gps_topic': '/mavros/global_position/global',
            # path ของ weight แบบ absolute
            'model_path': '/home/ciimav/ros2_ws/src/aavc/aavc/weight/bestv8.pt',
        }]
    )

    # Node 3: Geolocate
    geolocate_node = Node(
        package='aavc',
        executable='geolocate_node',
        name='geolocate_node',
        output='screen',
    )

    # Node 4: Cluster
    cluster_node = Node(
        package='aavc',
        executable='cluster_node',
        name='cluster_node',
        output='screen',
    )

    return LaunchDescription([
        # เริ่มกล้องก่อน
        camera_node,

        # YOLO เริ่มหลัง 5 วินาที
        TimerAction(
            period=5.0,
            actions=[detection_node]
        ),

        # node อื่น ๆ
        geolocate_node,
        cluster_node
    ])
