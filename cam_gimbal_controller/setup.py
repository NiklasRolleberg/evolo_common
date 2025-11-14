from setuptools import find_packages, setup

package_name = 'cam_gimbal_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='benjamin',
    maintainer_email='BenjaminSimon.studie@gmail.com',
    description='This package is used to control the camera gimbal. It reads and publish data from, and to, the gimbal. ' \
    'The gimbal angles are published to topic gimbal_euler, and is controlled by the topic desired_gimbal_euler.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'read_and_publish = cam_gimbal_controller.read_and_publish:main',
            'order_test_euler = cam_gimbal_controller.order_test_euler:main',
        ],
    },
)
