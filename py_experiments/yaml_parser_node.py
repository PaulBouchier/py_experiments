import rclpy
from rclpy.node import Node
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class YamlParserNode(Node):
    def __init__(self):
        super().__init__('yaml_parser_node')
        self.declare_parameter('yaml_file', 'coordinates.yaml')
        self.parse_yaml()

    def parse_yaml(self):
        yaml_file_name = self.get_parameter('yaml_file').get_parameter_value().string_value
        # Construct the full path to the YAML file within the package's share directory
        package_share_directory = get_package_share_directory('py_experiments')
        # Assuming the yaml file is in a 'resource' or similar folder within the share directory
        # Adjust the path construction if your YAML file is located elsewhere
        # For this example, let's assume it's directly in a 'resource' folder accessible via share
        # A common practice is to place such files in install/your_package_name/share/your_package_name/resource
        # However, get_package_share_directory points to the install space share directory.
        # If the file is in the source 'resource' folder, you might need a different approach
        # during development vs. installed deployment.
        # For installed packages, placing data files in share/package_name is standard.
        # Let's assume the file is intended to be found in the 'resource' directory relative to the package root
        # A better approach for accessing data files is using package resources, but let's stick to path construction for now.

        # Correct path assuming the file is installed in share/py_experiments/resource
        # This requires the file to be correctly specified in setup.py's data_files
        # Let's refine the path construction assuming it's in a 'resource' subdir of the package share
        # This might require adjusting setup.py data_files to install it there.
        # A simpler approach for demonstration might be to place it directly in the share directory,
        # or use an absolute path for testing, though not recommended for deployment.

        # Let's assume the file is located in a 'resource' directory *within* the installed share directory
        # This means setup.py should install it like ('share/' + package_name + '/resource', ['resource/coordinates.yaml'])
        # If setup.py installs it to 'share/' + package_name, the path is simpler.

        # Assuming setup.py installs 'resource/coordinates.yaml' to 'share/py_experiments/resource/coordinates.yaml'
        # The path needs to reflect this structure. Let's adjust setup.py's data_files pattern first.
        # If data_files = [ ..., ('share/' + package_name + '/resource', ['resource/coordinates.yaml']) ], then:
        # yaml_file_path = os.path.join(package_share_directory, 'resource', yaml_file_name)

        # If data_files = [ ..., ('share/' + package_name, ['resource/coordinates.yaml']) ], then:
        # This puts coordinates.yaml directly into share/py_experiments/
        # yaml_file_path = os.path.join(package_share_directory, yaml_file_name)

        # Let's assume the second case for simplicity, matching the current setup.py structure
        # where 'resource/coordinates.yaml' is installed to 'share/py_experiments/coordinates.yaml'
        # This requires adding ('share/' + package_name, ['resource/coordinates.yaml']) to data_files in setup.py

        # *** Correction based on typical ROS 2 practice and setup.py structure ***
        # Usually, data files like YAML are placed in a directory (e.g., 'config' or 'resource')
        # and installed to 'share/package_name/config' or 'share/package_name/resource'.
        # Let's assume the YAML file is in the 'resource' directory of the source package
        # and we want to install it to 'share/py_experiments/resource'.
        # We need to adjust setup.py data_files for this.
        # Assuming setup.py is updated like this:
        # ('share/' + package_name + '/resource', ['resource/coordinates.yaml']),
        # Then the path construction should be:
        yaml_file_path = os.path.join(package_share_directory, 'resource', yaml_file_name)

        # If the file is simply in the source 'resource' folder and setup.py installs it to
        # 'share/py_experiments' like ('share/' + package_name, ['resource/coordinates.yaml']),
        # then the path is:
        # yaml_file_path = os.path.join(package_share_directory, yaml_file_name)
        # Let's proceed with the assumption it's installed to share/py_experiments/resource/

        self.get_logger().info(f"Attempting to load YAML file from: {yaml_file_path}")

        try:
            with open(yaml_file_path, 'r') as file:
                data = yaml.safe_load(file)
                if data and 'coordinates' in data:
                    self.get_logger().info('Successfully parsed YAML file:')
                    for i, coord in enumerate(data['coordinates']):
                        self.get_logger().info(f"Coordinate {i+1}: {coord}")
                else:
                    self.get_logger().warn(f"YAML file '{yaml_file_path}' is empty or does not contain 'coordinates' key.")
        except FileNotFoundError:
            self.get_logger().error(f"YAML file not found at {yaml_file_path}. "
                                    f"Ensure the file exists and the path is correct. "
                                    f"Check setup.py data_files installation path.")
        except yaml.YAMLError as e:
            self.get_logger().error(f"Error parsing YAML file {yaml_file_path}: {e}")
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred: {e}")


def main(args=None):
    rclpy.init(args=args)
    yaml_parser_node = YamlParserNode()
    # We just want to parse the file, so we don't need to spin indefinitely.
    # However, logging might be asynchronous, so give it a moment.
    # Alternatively, make parsing happen on a timer or service call if needed dynamically.
    # For this example, parsing happens at init. We can destroy the node after logging.
    try:
        # Keep the node alive briefly to ensure logs are flushed.
        # A better approach in a real application might involve spinning until shutdown
        # or integrating the parsing logic differently.
        rclpy.spin_once(yaml_parser_node, timeout_sec=1.0) # Spin briefly
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        yaml_parser_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
