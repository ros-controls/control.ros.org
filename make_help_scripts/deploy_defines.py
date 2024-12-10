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
# should be control.ros.org folder
base_dir = os.path.dirname(script_base_dir)
# branch from which is started to checkout other branches
if os.environ.get('BASE_BRANCH_PR') is not None:
  base_branch = os.environ.get('BASE_BRANCH_PR')
elif os.environ.get('BASE_BRANCH') is not None:
  base_branch = os.environ.get('BASE_BRANCH')
else:
  base_branch = "rolling"
print(f"Using base_branch: {base_branch}")

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

# definitions for multiversion
# branches on which the temporary commits are created
# {"branch checked out for multiversion": "branch checked out for all subrepos with maps below"}
branch_version = {
    "foxy": "foxy",
    "galactic": "galactic",
    "humble": "humble",
    "iron": "iron",
    "jazzy": "jazzy",
    base_branch: "rolling"  # PRs are tested on rolling
}

# the subrepos which are cloned into the branches and, optionally, their corresponding PR for checkout
# and a dict, which branch should be checked out in the subrepositories

repos = {
    "ros2_control": {
        "url": "https://github.com/ros-controls/ros2_control",
        "branch_version": {
            "foxy": "foxy",
            "galactic": "galactic",
            "humble": "humble",
            "iron": "iron",
            "jazzy": "master",
            "rolling": "master"
        },
        "pr": os.environ.get('ROS2_CONTROL_PR')
    },
    "ros2_controllers": {
        "url": "https://github.com/ros-controls/ros2_controllers",
        "branch_version": {
            "foxy": "foxy",
            "galactic": "galactic",
            "humble": "humble",
            "iron": "iron",
            "jazzy": "master",
            "rolling": "master"
        },
        "pr": os.environ.get('ROS2_CONTROLLERS_PR')
    },
    "ros2_control_demos": {
        "url": "https://github.com/ros-controls/ros2_control_demos",
        "branch_version": {
            "foxy": "foxy",
            "galactic": "galactic",
            "humble": "humble",
            "iron": "iron",
            "jazzy": "master",
            "rolling": "master"
        },
        "pr": os.environ.get('ROS2_CONTROL_DEMOS_PR')
    },
    "gazebo_ros2_control": {
        "url": "https://github.com/ros-controls/gazebo_ros2_control",
        "branch_version": {
            "foxy": "foxy",
            "galactic": "galactic",
            "humble": "humble",
            "iron": "iron",
            "jazzy": "master",
            "rolling": "master"
        },
        "pr": os.environ.get('GAZEBO_ROS2_CONTROL_PR')
    },
    "gz_ros2_control": {
        "url": "https://github.com/ros-controls/gz_ros2_control",
        "branch_version": {
            "foxy": "foxy",
            "galactic": "galactic",
            "humble": "humble",
            "iron": "iron",
            "jazzy": "jazzy",
            "rolling": "rolling"
        },
        "pr": os.environ.get('GZ_ROS2_CONTROL_PR')
    },
    "control_toolbox": {
        "url": "https://github.com/ros-controls/control_toolbox",
        "branch_version": {
            "foxy": "foxy",
            "galactic": "ros2-master",
            "humble": "ros2-master",
            "iron": "ros2-master",
            "jazzy": "ros2-master",
            "rolling": "ros2-master"
        },
        "pr": None
    },
    "control_msgs": {
        "url": "https://github.com/ros-controls/control_msgs",
        "branch_version": {
            "foxy": "foxy-devel",
            "galactic": "galactic-devel",
            "humble": "humble",
            "iron": "master",
            "jazzy": "master",
            "rolling": "master"
        },
        "pr": None
    },
    "realtime_tools": {
        "url": "https://github.com/ros-controls/realtime_tools",
        "branch_version": {
            "foxy": "foxy-devel",
            "galactic": "master",
            "humble": "master",
            "iron": "master",
            "jazzy": "master",
            "rolling": "master"
        },
        "pr": None
    },
    "kinematics_interface": {
        "url": "https://github.com/ros-controls/kinematics_interface",
        "branch_version": {
            "foxy": "humble",
            "galactic": "humble",
            "humble": "humble",
            "iron": "master",
            "jazzy": "master",
            "rolling": "master"
        },
        "pr": None
    }
}
