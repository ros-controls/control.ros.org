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
import deploy_defines

def add_sub_repositories():
    # checkout a base for defined starting point
    os.chdir(deploy_defines.base_dir)
    for repo_name, repo_url in deploy_defines.subrepo_url.items():
        repo_path = os.path.join("doc", repo_name)
        if not os.path.isdir(repo_path):
            print(f"Create {repo_path} and checkout {deploy_defines.base_branch} branch")
            subprocess.run(["git", "clone", "-b", deploy_defines.base_branch, repo_url, repo_path], check=True)
        else:
            print(f"Update {repo_path} and checkout {deploy_defines.base_branch} branch")
            os.chdir(repo_path)
            subprocess.run(["git", "fetch", "origin"], check=True)
            subprocess.run(["git", "checkout", deploy_defines.base_branch], check=True)
            subprocess.run(["git", "pull"], check=True)
            os.chdir(deploy_defines.base_dir)
        if deploy_defines.subrepo_pr.get(repo_name):
            print(f"checkout PR: {deploy_defines.subrepo_pr[repo_name]}")
            os.chdir(repo_path)
            subprocess.run(["git", "fetch", "origin", f"{deploy_defines.subrepo_pr[repo_name]}:PR"], check=True)
            subprocess.run(["git", "checkout", "PR"], check=True)
            os.chdir(deploy_defines.base_dir)

if __name__ == "__main__":
    add_sub_repositories()
    deploy_defines.add_pr_stats_file()
