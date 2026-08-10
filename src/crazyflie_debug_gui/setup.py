from setuptools import find_packages, setup

package_name = 'crazyflie_debug_gui'

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
    maintainer='noah',
    maintainer_email='noah.ginzburg@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'crazyflie_debug_gui = crazyflie_debug_gui.debug_gui:main',
            'crazyflie_debug_gui_window = crazyflie_debug_gui.window:main',
            'crazyflie_debug_gui_telemetry = crazyflie_debug_gui.telemetry_model:main',
        ],
    },
)
