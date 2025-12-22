from setuptools import setup

package_name = 'primo_dynamixel_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[('share/' + package_name, ['package.xml'])],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeff',
    maintainer_email='jeff@todo.todo',
    description='Publishes joint states from Dynamixel motors',
    license='MIT',
    entry_points={
        'console_scripts': [
            'joint_publisher = primo_dynamixel_driver.joint_publisher:main',
        ],
    },
)