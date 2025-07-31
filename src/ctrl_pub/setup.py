from setuptools import setup

package_name = 'ctrl_pub'

setup(
    name=package_name,
    version='0.0.0',
    packages=['ctrl_pub'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pynput'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Keyboard + mouse control publisher for remote robot driving',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_publisher = ctrl_pub.publisher:main',
        ],
    },
)
