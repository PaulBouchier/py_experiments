from setuptools import find_packages, setup

package_name = 'py_experiments'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add this line to install the yaml file into share/py_experiments/resource
        ('share/' + package_name + '/resource', ['resource/coordinates.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bouchier',
    maintainer_email='paul.bouchier@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node1 = py_experiments.node1:main',
            'callback_group_demo = py_experiments.callback_group_demo:main',
            'timer_test = py_experiments.timer_test:main',
            'nav2pose = py_experiments.nav2pose:main',
            'nav_to_point_simple = py_experiments.nav_to_point_simple:main',
            'example_nav_to_pose = py_experiments.example_nav_to_pose:main',
            'scripted_bot_roto = py_experiments.scripted_bot_roto:main',
            'yaml_parser_node = py_experiments.yaml_parser_node:main', # Add this line
        ],
    },
)
