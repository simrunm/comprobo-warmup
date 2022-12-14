from setuptools import setup

package_name = 'warmup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='simrun',
    maintainer_email='simrun.mutha@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={        
        'console_scripts': [
            'test_viz = warmup.test_viz:main',
            'teleop = warmup.teleop:main',
            'wall_follower = warmup.wall_follower:main',
            'drive_square = warmup.drive_square:main',
            'obstacle_avoidance = warmup.obstacle_avoidance:main',
            'person_follower = warmup.person_follower:main',
            'finite_state_controller = warmup.finite_state_controller:main'
        ],
    },
)
