from setuptools import setup

package_name = 'decision_maker'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hannibal',
    maintainer_email='cds730@naver.com',
    description='Decision node for selecting steering and throttle commands.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'decision = decision_maker.decision:main',
            'decision2 = decision_maker.decision2:main',
            'decision3 = decision_maker.decision3:main',
            'visualizer = decision_maker.visualizer:main',
            'visualizer2 = decision_maker.visualizer2:main',
            'visualizer_flowchart = decision_maker.visualizer_flowchart:main',
        ],
    },
)
