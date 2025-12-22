from setuptools import find_packages, setup

package_name = 'primo-arm-r00'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/display.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/arm.urdf.xacro']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeff',
    maintainer_email='jeff@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
