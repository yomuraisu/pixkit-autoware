from setuptools import setup

package_name = 'pubsub'
submodules = 'pubsub/python_can'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aw22',
    maintainer_email='aw22@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'commander = pubsub.fromAutoware:main',
            'reporter = pubsub.toAutoware:main',
            'mode_publisher = pubsub.modePub:main',
        ],
    },
)
