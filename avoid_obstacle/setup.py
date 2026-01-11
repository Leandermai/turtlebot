from setuptools import find_packages, setup

package_name = 'avoid_obstacle'

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
    maintainer='Vinicius & Leander',
    maintainer_email='vz-235166@rwu.de & lm-223994@rwu.de',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'avoid_obstacle = avoid_obstacle.avoid_obstacle:main'
        ],
    },
)
