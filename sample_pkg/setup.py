from setuptools import find_packages, setup

package_name = 'sample_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='koushik',
    maintainer_email='koushik20040804@gmail.com',
    description='Sample nodes: pub/sub demo and an OpenCV color-blob detection pipeline.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ["cam_node=sample_pkg.video_publisher:main","cam_view_node=sample_pkg.video_subscriber:main","pub_node=sample_pkg.simple_pub:main","sub_node=sample_pkg.simple_sub:main",
        ],
    },
)
