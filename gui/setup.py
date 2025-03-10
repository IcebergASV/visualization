from setuptools import find_packages, setup

package_name = 'gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch.py']), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gracepearcey',
    maintainer_email='pearcey.grace@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui = gui.gui:main',
            'detection_visualizer = gui.detection_visualizer:main',
            'params_window = gui.params_window:main'
        ],
    },
)
