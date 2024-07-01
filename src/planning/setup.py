from setuptools import find_packages, setup

package_name = 'planning'

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
    maintainer='toni',
    maintainer_email='toni.sand@student.tu-freiberg.de',
    description='Package for nodes connected to turtlebot motion planning',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'defaultDriving     = planning.defaultDriving:main',
            'stateController    = planning.stateController:main',
            'obstruction        = planning.obstruction:main',
            'parking            = planning.parking:main',
            'crossroad          = planning.crossroad:main',
            'img                = planning.img:main'
        ],
    },
)
