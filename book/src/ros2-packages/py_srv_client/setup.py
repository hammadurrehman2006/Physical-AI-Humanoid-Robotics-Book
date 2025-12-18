from setuptools import setup

package_name = 'py_srv_client'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=['py_srv_client.service_member_function', 'py_srv_client.client_member_function'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name,
            ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Author Name',
    maintainer_email='author@example.com',
    description='Python service and client examples for Physical AI & Humanoid Robotics Book',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'add_two_ints_server = py_srv_client.service_member_function:main',
            'add_two_ints_client = py_srv_client.client_member_function:main',
        ],
    },
)