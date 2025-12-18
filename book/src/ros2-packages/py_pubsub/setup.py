from setuptools import setup

package_name = 'py_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=['py_pubsub.publisher_member_function', 'py_pubsub.subscriber_member_function'],
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
    description='Python publisher and subscriber examples for Physical AI & Humanoid Robotics Book',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = py_pubsub.publisher_member_function:main',
            'listener = py_pubsub.subscriber_member_function:main',
        ],
    },
)