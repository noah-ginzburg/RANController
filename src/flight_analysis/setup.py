from setuptools import find_packages, setup

package_name = 'flight_analysis'

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
    description='Read a flight bag, print the raw messages, plot vicon vs onboard position.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'flight_data = flight_analysis.flight_data:main',
        ],
    },
)
