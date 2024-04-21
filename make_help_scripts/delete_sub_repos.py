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

def delete_sub_repositories(base_branch):
    # checkout a base for defined starting point
    os.chdir(deploy_defines.base_dir)
    for repo_name in deploy_defines.repos.keys():
        repo_path = os.path.join("doc", repo_name)
        if os.path.isdir(repo_path):
            print(f"Delete {repo_path}")
            subprocess.run(["rm", "-rf", repo_path], check=True)

if __name__ == "__main__":
    delete_sub_repositories(deploy_defines.base_branch)
