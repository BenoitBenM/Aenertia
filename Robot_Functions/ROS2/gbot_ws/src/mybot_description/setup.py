from setuptools import setup

package_name = 'mybot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', ['urdf/mybot.urdf']),
        ('share/' + package_name + '/launch', ['launch/description.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Mybot URDF',
    license='MIT',
    tests_require=[],
    entry_points={},
)
