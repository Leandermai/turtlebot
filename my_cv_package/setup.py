from setuptools import find_packages, setup

package_name = 'my_cv_package'

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
    description='Line Follower',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cv_view = my_cv_package.cv_view:main',
            'cv_color_detect = my_cv_package.cv_color_detect:main',
            'cv_read_centers = my_cv_package.cv_read_centers:main',
            'cv_back_on_track = my_cv_package.cv_back_on_track:main'
        ],
    },
)
