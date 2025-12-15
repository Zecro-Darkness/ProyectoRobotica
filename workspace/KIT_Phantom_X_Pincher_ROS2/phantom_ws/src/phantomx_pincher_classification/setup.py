from setuptools import find_packages, setup

package_name = 'phantomx_pincher_classification'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/clasificador.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='juan',
    maintainer_email='juan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'clasificador_node = phantomx_pincher_classification.clasificador_node:main',
            'teleop_node = phantomx_pincher_classification.teleop_node:main',
            'teleop_joint_node = phantomx_pincher_classification.teleop_joint_node:main',
        ],
    },
)
