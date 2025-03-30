from setuptools import setup

package_name = 'my_generic_subscriber'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sumit',
    maintainer_email='sumit@example.com',
    description='A generic ROS 2 subscriber',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'generic_subscriber = my_generic_subscriber.generic_subscriber:main',
            'string_publisher = my_generic_subscriber.string_publisher:main',
            'int_publisher = my_generic_subscriber.int_publisher:main',
        ],
    },
)
