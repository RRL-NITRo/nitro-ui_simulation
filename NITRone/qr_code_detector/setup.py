from setuptools import find_packages, setup

package_name = 'qr_code_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    py_modules=[
        'qr_code_detector.qr_code_detector_node',
        'qr_code_detector.qr_code_switcher_nodecopy', # ここを追加
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'opencv-python', 'pyzbar'],
    zip_safe=True,
    maintainer='yoshiko',
    maintainer_email='yoshiko@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'qr_code_detector_node = qr_code_detector.qr_code_detector_node:main',
            'qr_code_switchercopy_node = qr_code_detector.qr_code_switcher_nodecopy:main', # ここを追加
        ],
    },
)
