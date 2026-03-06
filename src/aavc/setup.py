from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'aavc'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ✅ เพิ่มบรรทัดนี้ให้ถูกต้อง
        (os.path.join('share', package_name, 'launch'), glob('aavc/launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'ultralytics', 'opencv-python'],
    zip_safe=True,
    maintainer='ciimav',
    maintainer_email='ciimav@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_node = aavc.camera_node:main',
            'geolocate_node = aavc.geolocate_node:main',
            'cluster_node = aavc.cluster_node:main',
            'color_node = aavc.colorclassification_node:main',
            'mission_manager_node = aavc.mission_manager_node:main',
            'detection_node = aavc.detection_node:main',
            'image_sink_node = aavc.anno_node:main',
            'GPSCheck_node = aavc.GPSCheck_Node:main', 'auto_guided_arm_takeoff_ros2=aavc.takeoff_auto:main',
            'mavros_commander=aavc.setmode:main',
            'low_latency_viewer=aavc.lowlatency:main'
            
            
        ],
    },
)

