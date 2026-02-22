from setuptools import find_packages, setup

package_name = 'flight_control'

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
    maintainer='taeyoungkim',
    maintainer_email='goldrunty@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'offboard_test = flight_control.offboard_test:main',
            'turret_sim = flight_control.turret_sim:main',
            'turret_sim_new = flight_control.turret_sim_new:main',
            'train_only_dodge = flight_control.train_only_dodge:main',
            'turret_sim_new_fast = flight_control.turret_sim_new_fast:main',
            'turret_sim_new_fast_v2 = flight_control.turret_sim_new_fast_v2:main',
        ],
    },
)
