from glob import glob
from setuptools import setup


package_name = 'openpilot_ros2_bridge'


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
        (f'share/{package_name}/scripts', glob('openpilot/*.py')),
        (f'share/{package_name}/cereal', glob('cereal/*.capnp')),
        (f'share/{package_name}/docker', ['docker/Dockerfile', 'docker/run_openpilot_bridge_container.sh', 'docker/run_openpilot_bridge_host.sh']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jkk-research',
    maintainer_email='opensource@jkk-research.org',
    description='ROS 2 launch and bridge utilities for openpilot messaging.',
    license='TODO: License declaration',
)
