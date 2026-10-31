from glob import glob
from setuptools import find_packages, setup


package_name = 'fr5v6_weld_planning'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/meshes', glob('meshes/*')),
    ],
    install_requires=['setuptools', 'numpy', 'PyYAML'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='gsx',
    maintainer_email='gsx@localhost.local',
    description='FR5 V6 STL weld seam planning and simulation execution.',
    license='BSD-3-Clause',
    entry_points={'console_scripts': [
        'extract_seam = fr5v6_weld_planning.extract_seam:main',
        'run_weld_demo = fr5v6_weld_planning.run_weld_demo:main',
        'weld_task_manager = fr5v6_weld_planning.run_weld_demo:main',
        'weld_seam_detector = fr5v6_weld_planning.algorithm_nodes:detector_main',
        'weld_path_generator = fr5v6_weld_planning.algorithm_nodes:generator_main',
        'external_weld_task_manager = fr5v6_weld_planning.external_runner:main',
    ]},
)
