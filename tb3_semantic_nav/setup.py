import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'tb3_semantic_nav'


def data_files_for(subdir, patterns):
    """Install every file matching `patterns` under share/<pkg>/<subdir>."""
    out = []
    for pattern in patterns:
        matched = glob(os.path.join(subdir, pattern))
        if matched:
            out.append((os.path.join('share', package_name, subdir), matched))
    return out


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        *data_files_for('launch', ['*.launch.py']),
        *data_files_for('config', ['*.yaml']),
        *data_files_for('rviz', ['*.rviz']),
        *data_files_for('maps', ['*.yaml', '*.pgm']),
        *data_files_for('worlds', ['*.world', '*.sdf']),
        *data_files_for('urdf', ['*.urdf', '*.xacro']),
        # Gazebo model dirs are nested, so they are installed explicitly below.
        *[
            (os.path.join('share', package_name, os.path.dirname(p)), [p])
            for p in glob('models/**/*', recursive=True)
            if os.path.isfile(p)
        ],
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Samuel Liu',
    maintainer_email='samuelyliu2006@gmail.com',
    description='TurtleBot3 SLAM + Nav2 with a YOLO/depth/LiDAR semantic perception pipeline.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # verification / bring-up tools
            'explore_drive = tb3_semantic_nav.explore_drive:main',
            'nav_goal_test = tb3_semantic_nav.nav_goal_test:main',
            'check_health = tb3_semantic_nav.check_health:main',
            # Phase 4+ perception nodes are registered here.
        ],
    },
)
