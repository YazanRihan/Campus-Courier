from setuptools import find_packages, setup

package_name = 'camco_mission'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['resource/lookup/address_book.yaml']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'camco_kobuki_interface'],
    zip_safe=True,
    maintainer='patrolscouts',
    maintainer_email='yazanrihan@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camco_mission = camco_mission.camco_mission:main',
        ],
    },
)
