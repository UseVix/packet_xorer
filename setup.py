from setuptools import find_packages, setup

package_name = 'packet_xorer'

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
    maintainer='airlinux',
    maintainer_email='jozef.franciszek.wiatr@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'packet_xorer = packet_xorer.packet_xorer:main',
            'packet_xorer_robosense = packet_xorer.packet_xorer_robosense:main',
            'packet_xorer_ouster = packet_xorer.packet_xorer_ouster:main',
        ],
    },
)
