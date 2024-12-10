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

import requests
import os
from datetime import datetime, timedelta, timezone
import time

def get_api_response(url):
  """
  Sends a GET request to the specified URL with the necessary headers and returns the JSON response.

  Args:
    url (str): The URL to send the GET request to.

  Returns:
    tuple: A tuple containing the JSON response as a dictionary and the response object.
  """
  global_header = {
    "Accept": "application/vnd.github.v3+json",
    "Authorization": f"Bearer {os.environ['GITHUB_TOKEN']}"
  }
  # TODO(anyone): add error handling
  response = requests.get(url, headers=global_header)
  if response.status_code != 200:
    print(f"Error {response.status_code}: {response.json()['message']}")
    return [], response
  json = response.json()

  return json, response

def get_api_response_wait(url):
  """
  Waits for the API rate limit reset if the remaining requests are zero,
  then returns the API response for the given URL.

  Args:
    url (str): The URL to send the API request to.

  Returns:
    tuple: A tuple containing the JSON response as a dictionary and the response object.
  """
  remaining, reset = get_api_limit()
  if remaining == 0:
    wait_time = datetime.fromtimestamp(reset) - datetime.utcnow()
    print(f"Waiting {wait_time.total_seconds()} seconds for API rate limit reset")
    time.sleep(wait_time.total_seconds() + 60) # add 60 seconds to be sure

  return get_api_response(url)

def get_api_limit():
  """
  Retrieves the remaining API rate limit and reset time from the GitHub API.

  Returns:
    A tuple containing the remaining API rate limit and the reset time.
  """
  url = "https://api.github.com/rate_limit"
  json, response = get_api_response(url)

  if json:
    return json["rate"]["remaining"], json["rate"]["reset"]
  else:
    return 0, 0


def get_all_pages(url):
  """
  Generator function, retrieves all pages of data from the given URL.

  Args:
    url (str): The URL to retrieve data from.

  Yields:
    dict: A JSON object representing a page of data.

  Returns:
    None
  """
  while url:
    json, response = get_api_response_wait(url)

    yield json

    if 'next' in response.links:
      url = response.links['next']['url']
    else:
      url = None


def get_user_details(user):
  """
  Retrieves the name of a GitHub user.

  Args:
    user (str): The GitHub login name.

  Returns:
    str: The name of the GitHub user, or an empty string if the name is not available.
    avatar_url: The name of the GitHub user, or an empty string if the name is not available.
  """
  url = f"https://api.github.com/users/{user}"
  json, _ = get_api_response_wait(url)

  if json:
    return json["name"], json["avatar_url"]
  else:
    return ""


def get_pr_stats(owner, repos, branches, whitelist, blacklist, earliest_date=""):
  """
  Retrieves statistics about PRs: contributors' and reviewers' activity on pull requests.

  Args:
    owner (str): The owner of the repositories.
    repos (list): The list of repositories.
    branches (dict): The dictionary mapping repositories to their branches.
    whitelist (list): The list of whitelisted users.
    blacklist (list): The list of blacklisted users.
    earliest_date (str, optional): The earliest date to consider for reviews. Defaults to "".

  Returns:
    tuple: A tuple containing the following elements:
      - reviewers (dict): A dictionary containing statistics for all reviewers not in whitelist.
      - reviewers_whitelist (dict): A dictionary containing statistics for whitelisted reviewers.
      - reviewers_filter (dict): A dictionary containing filtered statistics for all reviewers not in whitelist.
      - reviewers_filter_whitelist (dict): A dictionary containing filtered statistics for whitelisted reviewers.
      - contributors (dict): A dictionary containing statistics for all contributors not in whitelist.
      - contributors_whitelist (dict): A dictionary containing statistics for whitelisted contributors.
      - contributors_filter (dict): A dictionary containing filtered statistics for all contributors not in whitelist.
      - contributors_filter_whitelist (dict): A dictionary containing filtered statistics for whitelisted contributors.
      - ct_pull (int): The total number of pull requests processed.
  """

  reviewers = {}
  reviewers_whitelist = {}
  reviewers_filter = {}
  reviewers_filter_whitelist = {}
  contributors = {}
  contributors_whitelist = {}
  contributors_filter = {}
  contributors_filter_whitelist = {}
  ct_pull = 0

  for repo in repos:
    print(f"Getting reviewers' stats for {owner}/{repo} on branch {branches[repo]}")

    url = f"https://api.github.com/repos/{owner}/{repo}/pulls?state=closed&base={branches[repo]}&per_page=100"

    for pulls in get_all_pages(url):

      for pull in pulls:
          ct_pull += 1

          # parse requested reviewers
          reviewers_list = pull["requested_reviewers"]
          for reviewer in reviewers_list:
            reviewer_login = reviewer["login"]

            if reviewer_login not in blacklist:
              if reviewer_login in whitelist:
                  current_dict = reviewers_whitelist
              else:
                  current_dict = reviewers

              if reviewer_login in current_dict:
                current_dict[reviewer_login]["assigned_reviews"] += 1
              else:
                current_dict[reviewer_login] = {
                  "assigned_reviews": 1,
                  "finished_reviews": 0,
                  "last_review_date": "0000-00-00T00:00:00Z"
                }

              # if filter is set, only count reviews after earliest_date
              if earliest_date and pull["created_at"] > earliest_date:
                if reviewer_login in whitelist:
                    current_dict = reviewers_filter_whitelist
                else:
                    current_dict = reviewers_filter

                if reviewer_login in current_dict:
                  current_dict[reviewer_login]["assigned_reviews"] += 1
                else:
                  current_dict[reviewer_login] = {
                    "assigned_reviews": 1,
                    "finished_reviews": 0,
                    "last_review_date": "0000-00-00T00:00:00Z"
                  }

          # Get reviews for the pull request, but count only once per PR
          pull_reviews_url = pull["url"] + "/reviews"
          pull_reviews, _ = get_api_response_wait(pull_reviews_url)
          local_reviewers = {} # prevent double counting
          local_reviewers_filter = {} # prevent double counting

          for review in pull_reviews:
            reviewer_login = review["user"]["login"]
            date = review["submitted_at"]

            if reviewer_login in blacklist:
              continue
            if reviewer_login in whitelist:
                current_dict = reviewers_whitelist
            else:
                current_dict = reviewers

            if reviewer_login in current_dict:
              if reviewer_login not in local_reviewers:
                current_dict[reviewer_login]["assigned_reviews"] += 1
                current_dict[reviewer_login]["finished_reviews"] += 1
                local_reviewers[reviewer_login] = True
              if date > current_dict[reviewer_login]["last_review_date"]:
                current_dict[reviewer_login]["last_review_date"] = date
            else:
              current_dict[reviewer_login] = {
                "assigned_reviews": 1,
                "finished_reviews": 1,
                "last_review_date": date
              }
              local_reviewers[reviewer_login] = True

            # if filter is set, only count reviews after earliest_date
            if earliest_date and pull["created_at"] > earliest_date:
              if reviewer_login in whitelist:
                  current_dict = reviewers_filter_whitelist
              else:
                  current_dict = reviewers_filter

              if reviewer_login in current_dict:
                if reviewer_login not in local_reviewers_filter:
                  current_dict[reviewer_login]["assigned_reviews"] += 1
                  current_dict[reviewer_login]["finished_reviews"] += 1
                  local_reviewers_filter[reviewer_login] = True
                if date > current_dict[reviewer_login]["last_review_date"]:
                  current_dict[reviewer_login]["last_review_date"] = date
              else:
                current_dict[reviewer_login] = {
                  "assigned_reviews": 1,
                  "finished_reviews": 1,
                  "last_review_date": date
                }
                local_reviewers_filter[reviewer_login] = True

    # parse line contributions from commits
    print(f"Getting contributors' stats for {owner}/{repo} on branch {branches[repo]}")
    commits_url = f"https://api.github.com/repos/{owner}/{repo}/commits?branch={branches[repo]}"

    for commits in get_all_pages(commits_url):
      for commit in commits:
        if commit['author'] is None:
          # print('No author in commit: ' + commit['url'])
          continue
        contributor_login = commit['author']['login']
        if contributor_login in blacklist:
            continue
        if contributor_login in whitelist:
            current_dict = contributors_whitelist
        else:
            current_dict = contributors

        commit_details, _ = get_api_response_wait(commit['url'])
        if 'stats' in commit_details.keys():
          additions = commit_details['stats']['additions']
          deletions = commit_details['stats']['deletions']
          total_changes = additions + deletions
        else:
          # print('No stats in commit details: ' + commit['url'])
          total_changes = 0
        date = commit_details['commit']['author']['date']
        if contributor_login in current_dict:
          current_dict[contributor_login]["total_changes"] += total_changes
          current_dict[contributor_login]["ct_commit"] += 1
          current_dict[contributor_login]["last_commit_date"] = date
        else:
          current_dict[contributor_login] = {
            "total_changes": total_changes,
            "ct_commit": 1,
            "last_commit_date": date
          }
        # if filter is set, only count reviews after earliest_date
        if earliest_date and date > earliest_date:
          if contributor_login in whitelist:
              current_dict = contributors_filter_whitelist
          else:
              current_dict = contributors_filter

          if contributor_login in current_dict:
            current_dict[contributor_login]["total_changes"] += total_changes
            current_dict[contributor_login]["ct_commit"] += 1
            current_dict[contributor_login]["last_commit_date"] = date
          else:
            current_dict[contributor_login] = {
            "total_changes": total_changes,
            "ct_commit": 1,
            "last_commit_date": date
            }

  return reviewers, reviewers_whitelist, reviewers_filter, reviewers_filter_whitelist, contributors, contributors_whitelist, contributors_filter, contributors_filter_whitelist, ct_pull


def create_reviewers_table_with_graph(reviewers_stats, user_details, table_name):
  """
  Creates an HTML table with reviewer statistics and graphs.

  Args:
    reviewers_stats (dict): A dictionary containing reviewer statistics.
      The keys are reviewer names and the values are dictionaries
      containing the following keys:
        - 'assigned_reviews' (int): The number of reviews assigned to the reviewer.
        - 'finished_reviews' (int): The number of reviews finished by the reviewer.
        - 'last_review_date' (str): The date of the last review by the reviewer.
    user_details (dict): A dictionary mapping login names to
        - 'name' (str): The name of the reviewer.
        - 'avatar_url' (str): The URL of the reviewer's avatar image.
    table_name (str): The ID of the HTML table.

  Returns:
    str: The HTML content of the table with reviewer statistics and graphs.

  style sheet for the table, copy into css file:

        <style>
            table {{
                font-family: Arial, sans-serif;
                border-collapse: collapse;
                width: 100%;
            }}

            th, td {{
                border: 1px solid #dddddd;
                text-align: left;
                padding: 8px;
            }}

            tr:nth-child(even) {{
                background-color: #f2f2f2;
            }}

            .progress-bar {{
                width: 100%;
                height: 20px;
                margin: 0;
                background-color: #ddd;
                border-radius: 5px;
                overflow: hidden;
            }}

            .progress-value-reviews {{
                display: block;
                height: 100%;
                width: 0;
                background-color: #2980b9;
                border-radius: 5px;
            }}

            .progress-value-ratio {{
                display: block;
                height: 100%;
                width: 0;
                background-color: rgba(47, 64, 95, 0.5); /* Adjusted to 50% transparent */
                border-radius: 5px;
            }}
        </style>
  """

  html_content = f"""
  <!DOCTYPE html>
  <html lang="en">
  <head>
      <meta charset="UTF-8">
      <title>Reviewers' Stats</title>
      <link rel="stylesheet" type="text/css" href="https://cdn.datatables.net/1.11.5/css/jquery.dataTables.css">
      <script src="https://code.jquery.com/jquery-3.6.0.min.js"></script>
      <script src="https://cdn.datatables.net/1.11.5/js/jquery.dataTables.js"></script>
  </head>
  <body>
    <p>
      <table id="{table_name}" class="display">
          <thead>
              <tr>
                  <th></th>
                  <th>Reviewer</th>
                  <th>Assigned</th>
                  <th>Finished</th>
                  <th>Rate</th>
                  <!--<th>Last Review Date</th>-->
              </tr>
          </thead>
          <tbody>
  """
  if reviewers_stats:
    # Find the reviewer with the highest number of finished reviews
    max_finished_reviews = max(stats['finished_reviews'] for stats in reviewers_stats.values())

    # Sort reviewers by finished reviews
    sorted_reviewers = sorted(reviewers_stats.items(), key=lambda x: x[1]['finished_reviews'], reverse=True)

    for idx, (reviewer, stats) in enumerate(sorted_reviewers):
        finished_reviews_bar_len = (stats['finished_reviews'] / max_finished_reviews) * 100
        finished_reviews_ratio = stats['finished_reviews']/stats['assigned_reviews']
        finished_reviews_ratio_bar_len = (finished_reviews_ratio) * 100

        # Add emojis for the first three reviewers
        medal = ""
        if idx == 0:
            medal = "🥇"
        elif idx == 1:
            medal = "🥈"
        elif idx == 2:
            medal = "🥉"

        html_content += f"""
            <tr>
                <td style=" text-align: center;">{medal}</td>
                <td>
                  <div style="display: flex; align-items: center;">
                    <div style="width: 40px;">
                      <img src="{user_details[reviewer]['avatar_url']}" width="36" height="36" alt="{reviewer}" style="border-radius: 50%;">
                    </div>
                    <div>
                      <div>
                        <b>{user_details[reviewer]['name']}</b> <br>
                        <a href="https://github.com/{reviewer}" target="_blank"><img src="https://github.githubassets.com/assets/GitHub-Mark-ea2971cee799.png" width="16" height="16" alt="{reviewer}">{reviewer}</a>
                      </div>
                    </div>
                  </div>
                </td>
                <td>{stats['assigned_reviews']}</td>
                <td>{stats['finished_reviews']}
                    <div class="progress-bar">
                        <div class="progress-value-reviews" style="width: {finished_reviews_bar_len}%;"></div>
                    </div>
                </td>
                <td>{finished_reviews_ratio:.2f}
                    <div class="progress-bar">
                        <div class="progress-value-ratio" style="width: {finished_reviews_ratio_bar_len}%;"></div>
                    </div>
                </td>
                <!--<td>{stats['last_review_date']}</td>-->
            </tr>
        """

  html_content += f"""
          </tbody>
      </table>
      Fetched on {current_date.strftime("%Y-%m-%d %H:%M:%S")} UTC
    </p>
  """
  html_content +=f"""
      <script>
          $('#{table_name}').DataTable({{
              "order": [[3, "desc"]]
          }});
      </script>
  </body>
  </html>
  """

  return html_content


def create_contributors_table_with_graph(contributors_stats, user_details, table_name):
  """
  Creates an HTML table with contributors statistics and graphs.

  Args:
    contributors_stats (dict): A dictionary containing contributors statistics.
      The keys are contributors names and the values are dictionaries
      containing the following keys:
        - 'total_changes' (int): The number of line changes by the contributor.
        - 'ct_commit' (int): The number of reviews finished by the contributor.
        - 'last_commit_date' (str): The date of the last pr by the contributor.
    user_details (dict): A dictionary mapping login names to
        - 'name' (str): The name of the reviewer.
        - 'avatar_url' (str): The URL of the reviewer's avatar image.
    table_name (str): The ID of the HTML table.

  Returns:
    str: The HTML content of the table with contributors statistics and graphs.

  style sheet for the table, copy into css file:

        <style>
            table {{
                font-family: Arial, sans-serif;
                border-collapse: collapse;
                width: 100%;
            }}

            th, td {{
                border: 1px solid #dddddd;
                text-align: left;
                padding: 8px;
            }}

            tr:nth-child(even) {{
                background-color: #f2f2f2;
            }}

            .progress-bar {{
                width: 100%;
                height: 20px;
                margin: 0;
                background-color: #ddd;
                border-radius: 5px;
                overflow: hidden;
            }}

            .progress-value-reviews {{
                display: block;
                height: 100%;
                width: 0;
                background-color: #2980b9;
                border-radius: 5px;
            }}

            .progress-value-ratio {{
                display: block;
                height: 100%;
                width: 0;
                background-color: rgba(47, 64, 95, 0.5); /* Adjusted to 50% transparent */
                border-radius: 5px;
            }}
        </style>
  """

  html_content = f"""
  <!DOCTYPE html>
  <html lang="en">
  <head>
      <meta charset="UTF-8">
      <title>Contributors' Stats</title>
      <link rel="stylesheet" type="text/css" href="https://cdn.datatables.net/1.11.5/css/jquery.dataTables.css">
      <script src="https://code.jquery.com/jquery-3.6.0.min.js"></script>
      <script src="https://cdn.datatables.net/1.11.5/js/jquery.dataTables.js"></script>
  </head>
  <body>
    <p>
      <table id="{table_name}" class="display">
          <thead>
              <tr>
                  <th></th>
                  <th>Contributor</th>
                  <th>Commit Count</th>
                  <th>Line Changes</th>
                  <!--<th>Last Review Date</th>-->
              </tr>
          </thead>
          <tbody>
  """
  if contributors_stats:
    # Find the contributors with the highest number of line_changes
    max_total_changes = max(stats['total_changes'] for stats in contributors_stats.values())
    max_pr_count = max(stats['ct_commit'] for stats in contributors_stats.values())

    # Sort contributors by total change
    sorted_contributors = sorted(contributors_stats.items(), key=lambda x: x[1]['total_changes'], reverse=True)

    for idx, (contributor, stats) in enumerate(sorted_contributors):
        total_changes_bar_len = (stats['total_changes'] / max_total_changes) * 100
        ct_commit_bar_len = (stats['ct_commit'] / max_pr_count) * 100

        # Add emojis for the first three contributors
        medal = ""
        if idx == 0:
            medal = "🥇"
        elif idx == 1:
            medal = "🥈"
        elif idx == 2:
            medal = "🥉"

        html_content += f"""
            <tr>
                <td style=" text-align: center;">{medal}</td>
                <td>
                  <div style="display: flex; align-items: center;">
                    <div style="width: 40px;">
                      <img src="{user_details[contributor]['avatar_url']}" width="36" height="36" alt="{contributor}" style="border-radius: 50%;">
                    </div>
                    <div>
                      <div>
                        <b>{user_details[contributor]['name']}</b> <br>
                        <a href="https://github.com/{contributor}" target="_blank"><img src="https://github.githubassets.com/assets/GitHub-Mark-ea2971cee799.png" width="16" height="16" alt="{contributor}">{contributor}</a>
                      </div>
                    </div>
                  </div>
                </td>
                <td>{stats['ct_commit']}
                    <div class="progress-bar">
                        <div class="progress-value-reviews" style="width: {ct_commit_bar_len}%;"></div>
                    </div>
                <td>{stats['total_changes']}
                    <div class="progress-bar">
                        <div class="progress-value-reviews" style="width: {total_changes_bar_len}%;"></div>
                    </div>
                </td>
                <!--<td>{stats['last_commit_date']}</td>-->
            </tr>
        """

  html_content += f"""
          </tbody>
      </table>
      Fetched on {current_date.strftime("%Y-%m-%d %H:%M:%S")} UTC
    </p>
  """
  html_content +=f"""
      <script>
          $('#{table_name}').DataTable({{
              "order": [[3, "desc"]]
          }});
      </script>
  </body>
  </html>
  """

  return html_content


def print_reviewers_stats(reviewers_stats):
  """
  Prints the statistics of the 10 reviewers with most finished reviews.

  Args:
    reviewers_stats (dict): A dictionary containing the statistics of the reviewers.

  Returns:
    None
  """
  for reviewer, stats in sorted(reviewers_stats.items(), key=lambda x: x[1]['finished_reviews'], reverse=True)[:10]:
    print(f"Reviewer: {reviewer}, Assigned Reviews: {stats['assigned_reviews']}, Finished Reviews: {stats['finished_reviews']}, rate of finished: {stats['finished_reviews']/stats['assigned_reviews']:.2f}, Last Review Date: {stats['last_review_date']}")

def print_contributors_stats(contributors_stats):
  """
  Prints the statistics of the 10 contributors with the most line changes.

  Args:
    contributors_stats (dict): A dictionary containing the statistics of the contributors.

  Returns:
    None
  """
  for contributor, stats in sorted(contributors_stats.items(), key=lambda x: x[1]['total_changes'], reverse=True)[:10]:
    print(f"Contributor: {contributor}, Number of PRs: {stats['ct_commit']}, Total Line Change: {stats['total_changes']}, Last PR Date: {stats['last_commit_date']}")


# Replace with your GitHub repository owner and name
owner = "ros-controls"
repos = [
  "ros2_control",
  "ros2_controllers",
  "ros2_control_demos",
  "control_toolbox",
  "realtime_tools",
  "control_msgs",
  "control.ros.org",
  "gazebo_ros2_control",
  "gz_ros2_control",
  "kinematics_interface",
  "ros2_control_ci",
  "ros2_rhel",
  "ros2_debian"
]

branches = {
  "ros2_control": "master",
  "ros2_controllers": "master",
  "ros2_control_demos": "master",
  "control_toolbox": "ros2-master",
  "realtime_tools": "master",
  "control_msgs": "master",
  "control.ros.org": "master",
  "gazebo_ros2_control": "master",
  "gz_ros2_control": "rolling",
  "kinematics_interface": "master",
  "ros2_control_ci": "master",
  "ros2_rhel": "main",
  "ros2_debian": "main"
}

maintainers = ["bmagyar", "destogl", "christophfroehlich", "saikishor"]
blacklist = ["dependabot[bot]", "mergify[bot]"]

# Get the current date and time
current_date = datetime.now(timezone.utc)

# Calculate one year ago from the current date
one_year_ago = current_date - timedelta(days=365)

# Format the date string as "YYYY-MM-DDTHH:MM:SSZ"
formatted_date = one_year_ago.strftime("%Y-%m-%dT%H:%M:%SZ")

print("----------------------------------")
print("------------  Start  -------------")
limit, reset = get_api_limit();
print(f"API limit: {limit}, next reset: {datetime.fromtimestamp(reset)}")
print("----------------------------------")
print(f"Fetch pull requests, all-time and after {formatted_date}:")
reviewers_stats, maintainers_stats, reviewers_stats_recent, maintainers_stats_recent, contributors_stats, contributors_maintainers_stats, contributors_stats_recent, contributors_maintainers_stats_recent, ct_pulls = get_pr_stats(owner, repos, branches, maintainers, blacklist, formatted_date)
print("----------------------------------")
print("------------ Get User ------------")
print("----------------------------------")
unique_reviewers = set(
  list(reviewers_stats_recent.keys())
  + list(maintainers_stats_recent.keys())
  + list(reviewers_stats.keys())
  + list(maintainers_stats.keys())
  + list(contributors_stats.keys())
  + list(contributors_maintainers_stats.keys())
  )
user_details = {}
for reviewer_login in unique_reviewers:
    name, avatar_url = get_user_details(reviewer_login)
    user_details[reviewer_login] = {
                  "name": name,
                  "avatar_url": avatar_url
                }


print(f"Got {len(unique_reviewers)} user names")

# Print the reviewers' stats in a nice format
print(f"---------------------------------")
print(f"------ Results from {ct_pulls} PRs ------")
print(f"---------------------------------")
print(f"Reviewers' Stats, after {formatted_date}:")
print("---------- maintainers -----------")
print_reviewers_stats(maintainers_stats_recent)

print("-------- not maintainers ---------")
print_reviewers_stats(reviewers_stats_recent)

print(f"Reviewers' Stats, all-time:")
print("---------- maintainers -----------")
print_reviewers_stats(maintainers_stats)

print("-------- not maintainers ---------")
print_reviewers_stats(reviewers_stats)
print("----------------------------------")

print(f"Contributors' Stats, after {formatted_date}:")
print("---------- maintainers -----------")
print_contributors_stats(contributors_maintainers_stats_recent)

print("-------- not maintainers ---------")
print_contributors_stats(contributors_stats_recent)

print(f"Contributors' Stats, all-time:")
print("---------- maintainers -----------")
print_contributors_stats(contributors_maintainers_stats)

print("-------- not maintainers ---------")
print_contributors_stats(contributors_stats)
print("----------------------------------")

limit, reset = get_api_limit();
print(f"API limit remaining: {limit}, next reset: {datetime.fromtimestamp(reset)}")
print("----------------------------------")
print("--------------- END --------------")
print("----------------------------------")

# Create the HTML content
html_maintainers_stats_recent = create_reviewers_table_with_graph(maintainers_stats_recent, user_details, "maintainers_stats_recent")
html_reviewers_stats_recent = create_reviewers_table_with_graph(reviewers_stats_recent, user_details, "reviewers_stats_recent")
html_maintainers_stats = create_reviewers_table_with_graph(maintainers_stats, user_details, "maintainers_stats")
html_reviewers_stats = create_reviewers_table_with_graph(reviewers_stats, user_details, "reviewers_stats")

html_contributors_maintainers_stats_recent = create_contributors_table_with_graph(contributors_maintainers_stats_recent, user_details, "contributors_maintainers_stats_recent")
html_contributors_stats_recent = create_contributors_table_with_graph(contributors_stats_recent, user_details, "contributors_stats_recent")
html_contributors_maintainers_stats = create_contributors_table_with_graph(contributors_maintainers_stats, user_details, "contributors_maintainers_stats")
html_contributors_stats = create_contributors_table_with_graph(contributors_stats, user_details, "contributors_stats")

# Save the HTML content to a file named "reviewers_stats_with_graph.html"
home_directory = os.path.expanduser( '~' )
filename_maintainers_stats_recent = os.path.join(home_directory, 'reviews', 'reviewers_maintainers_stats_recent.html')
filename_reviewers_stats_recent = os.path.join(home_directory, 'reviews', 'reviewers_stats_recent.html')
filename_maintainers_stats = os.path.join(home_directory, 'reviews', 'reviewers_maintainers_stats.html')
filename_reviewers_stats = os.path.join(home_directory, 'reviews', 'reviewers_stats.html')
filename_contributors_maintainers_stats_recent = os.path.join(home_directory, 'reviews', 'contributors_maintainers_stats_recent.html')
filename_contributors_stats_recent = os.path.join(home_directory, 'reviews', 'contributors_stats_recent.html')
filename_contributors_maintainers_stats = os.path.join(home_directory, 'reviews', 'contributors_maintainers_stats.html')
filename_contributors_stats = os.path.join(home_directory, 'reviews', 'contributors_stats.html')
os.makedirs(os.path.dirname(filename_maintainers_stats_recent), exist_ok=True)

with open(filename_maintainers_stats_recent, 'w') as file:
    file.write(html_maintainers_stats_recent)
print(f"HTML file {filename_maintainers_stats_recent} has been created.")

with open(filename_reviewers_stats_recent, 'w') as file:
    file.write(html_reviewers_stats_recent)
print(f"HTML file {filename_reviewers_stats_recent} has been created.")

with open(filename_maintainers_stats, 'w') as file:
    file.write(html_maintainers_stats)
print(f"HTML file {filename_maintainers_stats} has been created.")

with open(filename_reviewers_stats, 'w') as file:
    file.write(html_reviewers_stats)
print(f"HTML file {filename_reviewers_stats} has been created.")

with open(filename_contributors_maintainers_stats_recent, 'w') as file:
    file.write(html_contributors_maintainers_stats_recent)
print(f"HTML file {filename_contributors_maintainers_stats_recent} has been created.")

with open(filename_contributors_stats_recent, 'w') as file:
    file.write(html_contributors_stats_recent)
print(f"HTML file {filename_contributors_stats_recent} has been created.")

with open(filename_contributors_maintainers_stats, 'w') as file:
    file.write(html_contributors_maintainers_stats)
print(f"HTML file {filename_contributors_maintainers_stats} has been created.")

with open(filename_contributors_stats, 'w') as file:
    file.write(html_contributors_stats)
print(f"HTML file {filename_contributors_stats} has been created.")
