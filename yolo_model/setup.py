from setuptools import find_packages, setup

package_name = 'yolo_model'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools',
                    'ultralytics',
                    'rclpy',
                    'numpy',
                    'opencv-python',
                    'scipy',
                    'ultralytics',
                    'cv_bridge'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_node = yolo_model.yolo_node:main',
            'image_view = yolo_model.image_view:main',
        ],
    },
)
