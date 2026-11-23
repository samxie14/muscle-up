from setuptools import setup
from glob import glob

package_name = 'muscleup_update1'

setup(
    name=package_name,
    version='0.0.2',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Samuel Xie',
    maintainer_email='samuel@todo.todo',
    description='ME 133a final project: G1 humanoid pull-up and typewriter on a bar',
    license='TODO',
    entry_points={
        'console_scripts': [
            'typewriter = muscleup_update1.typewriter:main',
        ],
    },
)
