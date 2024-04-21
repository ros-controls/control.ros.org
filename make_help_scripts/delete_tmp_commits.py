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

def revert_temporary_commits():
    os.chdir(deploy_defines.base_dir)
    subprocess.run(["git", "checkout", deploy_defines.base_branch], check=True)

    for branch, _ in deploy_defines.branch_version.items():
        print(f"Revert to last changes before temporary multiversion commit on: {branch}.")
        subprocess.run(["git", "checkout", branch], check=True)
        subprocess.run(["git", "reset", "--hard", "HEAD~1"], check=True)
        subprocess.run(["git", "checkout", deploy_defines.base_branch], check=True)

if __name__ == "__main__":
    revert_temporary_commits()
