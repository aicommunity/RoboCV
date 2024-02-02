from setuptools import setup

package_name = 'perception_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eddyswens',
    maintainer_email='eddyswens@todo.todo',
    description='Perception system for autonomous driving',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_detector = perception_system.fake_detector:main',
        ],
    },
)