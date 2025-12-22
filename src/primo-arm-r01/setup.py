from setuptools import setup

package_name = 'primo-arm-r01'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/display.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/primio-arm-r01.urdf.xacro']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeff',
    maintainer_email='jeff@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)

