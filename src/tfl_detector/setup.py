from setuptools import setup

package_name = 'tfl_detector'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['best_m.pt', 'best_s.pt'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mehdi Testouri',
    maintainer_email='mehdi.testouri@uni.lu',
    description='Yolov8 based traffic light detection.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'tfl_detector = tfl_detector.detector:main'
        ],
    },
)