from glob import glob
import os
import warnings

from setuptools import setup, SetuptoolsDeprecationWarning

# https://github.com/colcon/colcon-core/issues/454
warnings.filterwarnings('ignore', category=SetuptoolsDeprecationWarning)
package_name = 'wombat_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    data_files=[
        (os.path.join('share', package_name),
            ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'params'),
            glob(os.path.join('params', '*.yaml'))),
        (os.path.join('share', package_name, 'rviz'),
            glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alberto Soragna',
    maintainer_email='alberto.soragna@gmail.com',
    description='Bringup scripts for Wombat software',
    license='Proprietary',
    tests_require=['pytest']
)
