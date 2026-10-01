from setuptools import setup
from glob import glob
setup(name='drone_follow', version='0.1.0', packages=['drone_follow'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/drone_follow']),
                  ('share/drone_follow', ['package.xml'] + glob('*.md')),
                  ('share/drone_follow/launch', glob('launch/*.py')),
                  ('share/drone_follow/config', glob('config/*.yaml')),
                  ('share/drone_follow/worlds', glob('worlds/*')),
                  ('share/drone_follow/scripts', glob('scripts/*')),
                  ('share/drone_follow/models/x500_follower', glob('models/x500_follower/*')),
                  ('share/drone_follow/models/x500_target', glob('models/x500_target/*'))],
      install_requires=['setuptools', 'numpy'], zip_safe=True,
      maintainer='Example maintainer', maintainer_email='example@example.com',
      description='Monocular Gazebo/PX4 image-centring baseline', license='MIT',
      entry_points={'console_scripts': [
          'detector = drone_follow.detector:main',
          'follower = drone_follow.follower:main',
          'leader = drone_follow.leader:main']})
