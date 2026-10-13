from setuptools import find_packages, setup

package_name = 'practica3'

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
    maintainer='ros2',
    maintainer_email='otto@ua.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'color_detector = practica3.color_detector:main',
            'hola_yasmin   = practica3.hola_yasmin:main',
            'caso2 = practica3.caso2:main',
            'caso1 = practica3.caso1:main',
            'waypoints = practica3.waypoints:main',
            'yasmin_node = practica3.yasmin_node:main',
            'caso3= practica3.caso3:main',
        ],
    },
)
