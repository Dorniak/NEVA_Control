from glob import glob
from setuptools import setup, find_packages

package_name = 'NEVA_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=['control'],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/{}'.format(package_name), glob('launch/*.launch.py')),
        ('share/conf/', glob('conf/*'))
    ],
    install_requires=['setuptools', 'PyYAML', 'numpy'],
    zip_safe=True,
    maintainer='',
    maintainer_email='',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control = control.NEVA_Control:main',
        ],
    },
)
