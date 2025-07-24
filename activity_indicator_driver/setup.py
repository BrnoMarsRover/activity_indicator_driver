from setuptools import find_packages, setup

package_name = 'activity_indicator_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'activity_indicator_msgs'],
    zip_safe=True,
    maintainer='Stanislav Svědiroh',
    maintainer_email='stanislav.svediroh@vut.cz',
    description='Freya neopixel activity indicator driver using the I2C to Neopixel Driver Board',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'activity_indicator_driver = activity_indicator_driver.activity_indicator_driver:main'
        ],
    },
)
