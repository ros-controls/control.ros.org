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

# This script is used to check the links in the ROS documentation.
# due to a bug in the sphinx linkchecker, github anchors are false positives and
# have to be explicitly ignored
# https://github.com/sphinx-doc/sphinx/issues/9016

import os
import shutil
import subprocess
import sys

LOGFILE = "linkcheck.log"


def cleanup():
    os.remove(LOGFILE)
    shutil.rmtree("doc/api/", ignore_errors=True)


def main():
    if len(sys.argv) != 2:
        print("Usage: python check_links.py <path_to_html_dir>")
        sys.exit(1)

    html_dir = sys.argv[1]

    # Copy API documentation to local directory
    shutil.copytree(os.path.join(html_dir, "doc", "api"),
                    "doc/api/", dirs_exist_ok=True)

    # Run linkcheck
    with open(LOGFILE, "w") as logfile:
        subprocess.run(["make", "linkcheck"], stdout=logfile,
                       stderr=subprocess.PIPE)

    # Check for broken links
    with open(LOGFILE, "r") as logfile:
        broken_links = [
          line for line in logfile if 'broken' in line]

    if broken_links:
        num_broken = len(broken_links)
        print(f"Broken links found: {num_broken}")
        for link in broken_links:
            print(link)
        cleanup()
        sys.exit(1)

    cleanup()
    sys.exit(0)


if __name__ == "__main__":
    main()
