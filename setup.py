from setuptools import find_packages, setup
setup(
    name='carlatools',
    packages=find_packages(include=['carlatools']),
    version='0.1.1',
    description='Utils for training agents and collecting data in CARLA',
    author='Enes M. Erciyes',
    license='MIT',
    install_requires=['carla', 'numpy', 'pandas', 'pillow'],
    setup_requires=['pytest-runner'],
    tests_require=['pytest==4.4.1'],
    test_suite='tests'
)
