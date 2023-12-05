from setuptools import setup

package_name = 'baxter_dev'

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
    maintainer='Shaun Altmann',
    maintainer_email='saltmann@deakin.edu.au',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'listener = baxter_dev.basic_subscriber:main',
            'talker = baxter_dev.basic_publisher:main',
            'digital_io_test = baxter_dev.digital_io_test:main',
            'camera_test = baxter_dev.camera_test:main',
            'client_stream_image = baxter_dev.clients:stream_image',
        ],
    },
)
