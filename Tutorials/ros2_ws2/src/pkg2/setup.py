from setuptools import find_packages, setup

package_name = 'pkg2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shaun A',
    maintainer_email='saltmann@deakin.edu.au',
    description='Python client server tutorial',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'service = pkg2.service_member_function:main',
        	'client = pkg2.client_member_function:main',
        ],
    },
)
