import os
import sys
import argparse
import re
from textwrap import dedent

# Append current directory to path to import deploy_defines
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(script_dir)
from deploy_defines import branch_version

def get_eol_distros():
    """Extracts the EOL versions from conf.py without executing Sphinx imports."""
    conf_path = os.path.join(os.path.dirname(script_dir), "conf.py")
    try:
        with open(conf_path, "r") as f:
            content = f.read()
        match = re.search(r'smv_eol_versions\s*=\s*\[(.*?)\]', content, re.DOTALL)
        if match:
            raw_string = match.group(1).replace('"', '').replace("'", "")
            return [d.strip() for d in raw_string.split(",") if d.strip()]
        print(f"Warning: 'smv_eol_versions' not found in {conf_path}. Defaulting to empty EOL list.")
        return []
    except FileNotFoundError:
        print(f"Warning: Could not find {conf_path}. Defaulting to empty EOL list.")
        return []

def main():
    parser = argparse.ArgumentParser(description="Generate llms.txt file")
    parser.add_argument("--output", required=True, type=str, default="llms.txt", help="Output file name")
    args = parser.parse_args()

    static_header = dedent("""\
        # ros2_control Documentation

        > ros2_control is a framework for real-time control of robots using ROS 2. It provides hardware abstraction, a controller manager, and a set of standard controllers for common robot hardware.
        
        ## Tools and APIs
        - [OpenAPI Specification](openapi.yaml): Machine-readable schema for programmatic page fetching.
        
        ## Key Sections
        - [Getting Started](doc/getting_started/getting_started.html)
        - [Controllers](doc/ros2_controllers/doc/controllers_index.html)
        - [Demos](doc/ros2_control_demos/doc/index.html)
        - [Hardware Components](doc/ros2_control/doc/index.html)
        - [Utilities](doc/utilities.html)
        - [Simulator Integration](doc/simulators/simulators.html)
        - [Supported Robotics](doc/supported_robots/supported_robots.html)
        - [API Reference](doc/api_list/api_list.html)
        """)
    
    # Dynamically fetch EOL distributions from the official source of truth
    eol_distros = get_eol_distros()
    
    # Filter out EOL distros, the base_branch duplicate, and lyrical
    active_distros = [
        distro for distro in branch_version.keys() 
        if distro not in eol_distros and distro != "lyrical"
    ]

    dynamic_version = "## Supported ROS 2 Versions\n"
    dynamic_migrations = "## Versioning and Migration\n"
    dynamic_migrations += "APIs and hardware interfaces differ significantly between ROS 2 distributions. Always consult the version-specific changelogs and migration paths before generating C++ configurations:\n"

    for distro in sorted(active_distros):
        dynamic_version += f"- [{distro.capitalize()}](https://control.ros.org/{distro}/)\n"
        dynamic_migrations += f"- [{distro.capitalize()} Release Notes](https://control.ros.org/{distro}/doc/release_notes/release_notes.html)\n"
        dynamic_migrations += f"- [{distro.capitalize()} Migration Guide](https://control.ros.org/{distro}/doc/migration/migration.html)\n"
    
    dynamic_migrations += "- [Differences to ROS 1 (ros_control)](doc/migration/differences_to_ros1.html)\n"

    final_content = static_header + "\n" + dynamic_version + "\n" + dynamic_migrations

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, "w") as f:
        f.write(final_content)

if __name__ == "__main__":
    main()