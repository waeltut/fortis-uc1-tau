from setuptools import find_packages, setup

package_name = 'reasoning_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ros@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'motivation_work = reasoning_controller.motivation_work:main',
            'motivation_respond = reasoning_controller.motivation_respond:main',
            'motivation_replan = reasoning_controller.motivation_replan:main',
            'motivation_inform = reasoning_controller.motivation_inform:main',
            'motivation_capability = reasoning_controller.motivation_capability:main',
            'aggregate_motivation = reasoning_controller.aggregate_motivation:main',
        ],
    },
)
