from setuptools import setup

package_name = 'turtle_tag_simulator'

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
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "players_spawner = turtle_tag_simulator.players_spawner:main",
            "tagger_control_system = turtle_tag_simulator.tagger_control_system:main"

        ],
    },
)
