from setuptools import find_packages, setup

package_name = 'p_controller'

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
    maintainer='vilma',
    maintainer_email='vilma.muco@exail.com',
    description='TODO: Package description',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'circle_controller = p_controller.circle_controller:main',
            'controller_sub = p_controller.controller_subscriber:main',
        ],
    },
)
