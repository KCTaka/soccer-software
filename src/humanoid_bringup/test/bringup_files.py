"""The files humanoid_bringup's tests read: this package's configs and launch files, and the
robot descriptions humanoid_description installs, the ones a launch hands ros2_control."""
import importlib.util
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import yaml

PACKAGE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG = os.path.join(PACKAGE, 'config')
ROBOTS = os.path.join(get_package_share_directory('humanoid_description'), 'robots')


def load_yaml(*path):
    with open(os.path.join(*path)) as f:
        return yaml.safe_load(f)


def configured_robots():
    """Every robot this package configures: those with a config/<robot>/controllers.yaml."""
    return sorted(robot for robot in os.listdir(CONFIG)
                  if os.path.isfile(os.path.join(CONFIG, robot, 'controllers.yaml')))


def controller(robot):
    """The humanoid_mit_controller parameters of `robot`'s controllers.yaml."""
    return load_yaml(CONFIG, robot, 'controllers.yaml')['humanoid_mit_controller'][
        'ros__parameters']


def description_dir(robot):
    return os.path.join(ROBOTS, robot)


def urdf_text(robot):
    with open(os.path.join(description_dir(robot), 'urdf', 'robot.urdf')) as f:
        return f.read()


def urdf(robot):
    return ET.fromstring(urdf_text(robot))


def commanded_joints(root):
    """The joints the <ros2_control> block of the URDF `root` commands, in its order."""
    return [j.get('name') for j in root.find('ros2_control').findall('joint')
            if j.find('command_interface') is not None]


def transmissions(root):
    return root.find('ros2_control').findall('transmission')


def load_launch(name):
    """The launch file launch/`name`.launch.py as a module, for its pure helpers."""
    spec = importlib.util.spec_from_file_location(
        f'{name}_launch', os.path.join(PACKAGE, 'launch', f'{name}.launch.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module
