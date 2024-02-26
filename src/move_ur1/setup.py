from setuptools import setup
import os
from glob import glob

package_name = 'move_ur1'
submodules = "move_ur1/submodules"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='orima',
    maintainer_email='ori.mana@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_arm = move_ur1.move_ur5:main',
            'get_tf = move_ur1.get_state_ur5:main',
            'marker = move_ur1.markers:main',
            'marker_arr = move_ur1.multi_marker:main',
            'tf_global = move_ur1.tf_broadcaster:main',
            'collision_detector = move_ur1.sphere_tf:main',
            'depricated_move_joint_trajectory = move_ur1.move_ur5_joint_trajectory:main',
            'move_joint_trajectory = move_ur1.plan_execution:main',
            'show_arm_state = move_ur1.get_joint_states:main',
            
        ],
    },
)
