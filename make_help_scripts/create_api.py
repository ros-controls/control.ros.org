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
import deploy_defines

def create_api():
    os.chdir(os.path.join(deploy_defines.base_dir, "doc"))
    os.system("doxygen Doxyfile")
    os.makedirs(os.path.join(deploy_defines.base_dir, deploy_defines.build_dir, "html/doc/api/"), exist_ok=True)
    shutil.copytree("tmp/html/", os.path.join(deploy_defines.base_dir, deploy_defines.build_dir, "html/doc/api/"), dirs_exist_ok=True)
    shutil.rmtree("tmp")

os.chdir(deploy_defines.base_dir)
create_api()
