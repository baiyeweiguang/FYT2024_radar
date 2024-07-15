from setuptools import setup, find_packages

package_name = 'radar_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/model', ['model/armor.onnx', 'model/armor.pt', 'model/car.onnx']),],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zcf',
    maintainer_email='baiyeweiguang@163.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'radar_detector_node = radar_detector.detector_node:main' 
        ],
    },
)
