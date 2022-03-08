from setuptools import setup

package_name = 'demo_python_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sorawit',
    maintainer_email='sorawit.inp@gmail.com',
    description='demo basic ros2 structure',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker2 = demo_python_pkg.publisher:main',
            'listener = demo_python_pkg.subscriber:main',
        ],
    },
)
