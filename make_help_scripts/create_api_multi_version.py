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
import shutil
import subprocess
import deploy_defines
import add_sub_repos

def create_apis():
    os.chdir("doc/")
    for branch, version in deploy_defines.branch_version.items():
        subprocess.run(["git", "checkout", version], check=True)
        add_sub_repos.add_sub_repositories(branch)
        subprocess.run(["doxygen", "Doxyfile"], check=True)
        api_output_dir = os.path.join(deploy_defines.base_dir, deploy_defines.build_dir, "html", branch, "doc", "api")
        os.makedirs(api_output_dir, exist_ok=True)
        shutil.copytree("tmp/html/", api_output_dir, dirs_exist_ok=True)
        shutil.rmtree("tmp")

if __name__ == "__main__":
    os.chdir(deploy_defines.base_dir)
    create_apis()
