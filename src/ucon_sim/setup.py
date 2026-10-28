from setuptools import find_packages, setup

package_name = 'ucon_sim'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sim.launch.py', 'launch/original_sim.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='UCON Maintainer',
    maintainer_email='maintainer@example.com',
    description='Self-contained ROS 2 simulation for UCON.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'kinematic_sim = ucon_sim.kinematic_sim:main',
            'sim_monitor = ucon_sim.sim_monitor:main',
            'static_map_display = ucon_sim.static_map_display:main',
        ],
    },
)
