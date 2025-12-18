from setuptools import setup

package_name = 'py_action_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=['py_action_server.fibonacci_action_server', 'py_action_server.fibonacci_action_client'],
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
    description='Python action server and client examples for Physical AI & Humanoid Robotics Book',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fibonacci_action_server = py_action_server.fibonacci_action_server:main',
            'fibonacci_action_client = py_action_server.fibonacci_action_client:main',
        ],
    },
)