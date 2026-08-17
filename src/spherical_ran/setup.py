from setuptools import find_packages, setup

package_name = 'spherical_ran'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyvista', 'scipy'],
    zip_safe=True,
    maintainer='noah',
    maintainer_email='noah.ginzburg@gmail.com',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'spherical_RAN_server = spherical_ran.spherical_RAN_server:main',
            'generate_kernel_cache = spherical_ran.generate_kernel_cache:main',
        ],
    },
)
