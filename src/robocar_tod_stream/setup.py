from setuptools import setup

package_name = 'robocar_tod_stream'

setup(
    name=package_name,
    version='0.9.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mehdi Testouri',
    maintainer_email='mehdi.testouri@uni.lu',
    description='Vehicle camera streaming for teleoperated driving',
    license='MIT',
    entry_points={
        'console_scripts': [
            'robocar_tod_stream = robocar_tod_stream.tod_stream:main'
        ],
    },
)