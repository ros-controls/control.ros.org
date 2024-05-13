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

import argparse
import os
import shutil
import deploy_defines

def fix_index(base_branch, builddir):

    # Create the index.html file in the html directory
    with open(os.path.join(builddir, 'html', 'index.html'), 'w') as f:
        f.write(f'<html><head><meta http-equiv="refresh" content="0; url={base_branch}/index.html" /></head></html>')

    # Copy the contents of the base_branch directory to the master directory
    shutil.copytree(os.path.join(builddir, 'html', base_branch), os.path.join(builddir, 'html', 'master'), dirs_exist_ok=True)

    # Patch the index.html file in the master directory
    with open(os.path.join(builddir, 'html', 'master', 'index.html'), 'w') as f:
        f.write(f'<html><head><meta http-equiv="refresh" content="0; url=../{base_branch}/index.html" /></head></html>')

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Fix index.')
    parser.add_argument('--builddir', required=True, help='Build directory.')

    args = parser.parse_args()
    fix_index(deploy_defines.base_branch, args.builddir)
