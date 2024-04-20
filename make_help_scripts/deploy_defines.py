# Copyright (c) 2023 ros2_control maintainers
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import shutil

# This file includes all the definitions used in the other scripts
script_base_dir = os.path.dirname(os.path.abspath(__file__))
# should by control.ros.org folder
base_dir = os.path.dirname(script_base_dir)
# branch from which is started to checkout other branches
base_branch = "master"
build_dir = "_build"

# pr stats
pr_stats_files = [
    "reviewers_maintainers_stats_recent.html",
    "reviewers_stats_recent.html",
    "reviewers_maintainers_stats.html",
    "reviewers_stats.html",
    "contributors_maintainers_stats_recent.html",
    "contributors_stats_recent.html",
    "contributors_maintainers_stats.html",
    "contributors_stats.html"
]
pr_stats_cache_folder = "reviews"
pr_stats_target_folder = "./doc/acknowledgements"

def add_pr_stats_file():
    for pr_stats_filename in pr_stats_files:
        orig_file = os.path.join(os.environ['HOME'], pr_stats_cache_folder, pr_stats_filename)
        if os.path.isfile(orig_file):
            print(f"Copy PR stats file '{pr_stats_filename}' to '{pr_stats_target_folder}'")
            shutil.copy(orig_file, pr_stats_target_folder)
        else:
            print(f"Create empty PR stats file '{pr_stats_filename}' in '{pr_stats_target_folder}'")
            with open(os.path.join(pr_stats_target_folder, pr_stats_filename), 'w') as f:
                f.write("<i>No pr statistics available yet.</i>")

# definition single html
# the branch from which the api is checked out and built
api_branch = "master"

# definitions for multiversion
# branches on which the temporary commits are created and which branch should be checked out in the subrepositories
# {"branch checked out for multiversion": "branch checked out for all subrepos"}
branch_version = {
    "foxy": "foxy",
    "galactic": "galactic",
    "humble": "humble",
    "iron": "iron",
    "master": "master"  # master is rolling
}

# the subrepos which are cloned into the branches and, optionally, their corresponding PR for checkout
subrepo_url = {
    "ros2_control": "https://github.com/ros-controls/ros2_control",
    "ros2_controllers": "https://github.com/ros-controls/ros2_controllers",
    "ros2_control_demos": "https://github.com/ros-controls/ros2_control_demos",
    "gazebo_ros2_control": "https://github.com/ros-controls/gazebo_ros2_control",
    "gz_ros2_control": "https://github.com/ros-controls/gz_ros2_control",
    "control_toolbox": "https://github.com/ros-controls/control_toolbox",
    "control_msgs": "https://github.com/ros-controls/control_msgs",
    "realtime_tools": "https://github.com/ros-controls/realtime_tools",
    "kinematics_interface": "https://github.com/ros-controls/kinematics_interface"
}
subrepo_pr = {
    "ros2_control": os.environ.get('ROS2_CONTROL_PR'),
    "ros2_controllers": os.environ.get('ROS2_CONTROLLERS_PR'),
    "ros2_control_demos": os.environ.get('ROS2_CONTROL_DEMOS_PR'),
    "gazebo_ros2_control": os.environ.get('GAZEBO_ROS2_CONTROL_PR'),
    "gz_ros2_control": os.environ.get('GZ_ROS2_CONTROL_PR')
}
