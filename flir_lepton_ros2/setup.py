from setuptools import setup

package_name = 'flir_lepton_ros2'
submodules = "flir_lepton_ros2/submodules"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/flir_lepton.launch.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hungtc',
    maintainer_email='hungtc@solab.me.ntu.edu.tw',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'flir_lepton_publisher = flir_lepton_ros2.flir_lepton_publisher:main',
            'flir_lepton_subscriber = flir_lepton_ros2.flir_lepton_subscriber:main',
            'flir_boson_subscriber = flir_lepton_ros2.flir_boson_subscriber:main',
            'ximea_subscriber = flir_lepton_ros2.ximea_subscriber:main',

        ],
    },
)
