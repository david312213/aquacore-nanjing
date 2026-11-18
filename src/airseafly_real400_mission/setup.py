from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
setup(**generate_distutils_setup(packages=['airseafly_real400_mission'], package_dir={'': 'src'}))
