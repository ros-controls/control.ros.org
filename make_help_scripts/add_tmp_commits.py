#!/usr/bin/env python3
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
import subprocess
import sys
import shutil
import deploy_defines

def check_repositories():
    os.chdir(deploy_defines.base_dir)
    # Check for uncommitted changes in control.ros.org repository
    if subprocess.run(["git", "status", "--porcelain"], capture_output=True, text=True).stdout.strip():
        print("control.ros.org repository has uncommitted changes, which will be deleted!")
        subprocess.run(["git", "status", "--porcelain"])
        sys.exit(2)
    # Check if subrepos exist or are symlinks
    for repo_name in deploy_defines.repos.keys():
        repo_path = os.path.join("doc", repo_name)
        if os.path.islink(repo_path):
            print(f"{repo_name} is a symlink, delete link or repository will be broken!")
            sys.exit(2)
        if os.path.isdir(repo_path) and subprocess.run(["git", "-C", repo_path, "status", "--porcelain"], capture_output=True, text=True).stdout.strip():
            print(f"{repo_name} repository exists already, save your changes because it will be deleted!")
            sys.exit(2)

def add_sub_repositories_and_commit():
    os.chdir(deploy_defines.base_dir)
    # Checkout the base branch
    subprocess.run(["git", "checkout", deploy_defines.base_branch], check=True)
    # For each branch from multi version, checkout branch, clone sub repositories with docs add as tmp commit and remove
    for branch, version in deploy_defines.branch_version.items():
        print("----------------------------------------------------")
        print(f"Switch to branch: {branch} with version: {version}")
        subprocess.run(["git", "checkout", branch], check=True)
        # Modify .gitignore to include subrepositories
        subprocess.run(["sed", "-i", "s/doc\/ros2_control/\#doc\/ros2_control/g", ".gitignore"], check=True)
        subprocess.run(["sed", "-i", "s/doc\/gz_ros2_control/\#doc\/gz_ros2_control/g", ".gitignore"], check=True)
        subprocess.run(["sed", "-i", "s/doc\/gazebo_ros2_control/\#doc\/gazebo_ros2_control/g", ".gitignore"], check=True)
        # Clone all subrepositories and add as tmp commit to branch of multi version
        print(f"Clone repositories for {branch} and checkout branches for {version}")
        for repo_name, repo_details in deploy_defines.repos.items():
            branch = repo_details["branch_version"][version]
            print(f"Create doc/{repo_name}")
            subprocess.run(["git", "clone", "-b", branch, repo_details["url"], f"doc/{repo_name}"], check=True, stdout=subprocess.DEVNULL)
            os.chdir(f"doc/{repo_name}")
            # Remove git repo so that doc files of subrepo can be added to tmp commit
            shutil.rmtree(".git")
            os.chdir("../../")
        deploy_defines.add_pr_stats_file()
        subprocess.run(["git", "add", "."], check=True)
        # We don't want to use-precommit to check if subrepos are correct
        subprocess.run(["git", "commit", "-m", "Add temporary changes for multi version", "--no-verify"], check=True, stdout=subprocess.DEVNULL)
        subprocess.run(["git", "checkout", deploy_defines.base_branch], check=True)
        # Remove leftover folders if existing
        for repo_name in deploy_defines.repos.keys():
            shutil.rmtree(f"doc/{repo_name}", ignore_errors=True)
    print("---------- end add_sub_repositories_and_commit ----------------")

if __name__ == "__main__":
    check_repositories()
    add_sub_repositories_and_commit()
