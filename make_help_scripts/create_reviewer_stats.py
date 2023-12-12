import requests
import os
from datetime import datetime, timedelta

def get_api_response(url):
  global_header = {
    "Accept": "application/vnd.github.v3+json",
    "Authorization": f"Bearer {os.environ['GITHUB_TOKEN']}"
  }
  # TODO(anyone): add error handling
  response = requests.get(url, headers=global_header)
  if response.status_code != 200:
    print(f"Error: {response.json()['message']}")
    return [], response
  json = response.json()

  return json, response

def get_all_pages(url):
    while url:
        json, response = get_api_response(url)

        yield json

        if 'next' in response.links:
            url = response.links['next']['url']
        else:
            url = None


def get_user_name(user):
  url = f"https://api.github.com/users/{user}"
  json, response = get_api_response(url)

  if json:
    return json["name"]
  else:
    return ""


def get_reviewers_stats(owner, repos, branches, whitelist, earliest_date="0000-00-00T00:00:00Z"):

  reviewers = {}
  reviewers_whitelist = {}
  ct_pull = 0

  for repo in repos:
    print(f"Getting reviewers' stats for {owner}/{repo} on branch {branches[repo]}")

    url = f"https://api.github.com/repos/{owner}/{repo}/pulls?state=closed&base={branches[repo]}&per_page=100"

    for pulls in get_all_pages(url):

      for pull in pulls:
        if pull["created_at"] > earliest_date:
          ct_pull += 1

          # parse requested reviewers
          reviewers_list = pull["requested_reviewers"]
          for reviewer in reviewers_list:
            reviewer_login = reviewer["login"]

            if reviewer_login not in whitelist:
              if reviewer_login in reviewers:
                reviewers[reviewer_login]["assigned_reviews"] += 1
              else:
                reviewers[reviewer_login] = {
                  "avatar_url": reviewer["avatar_url"],
                  "assigned_reviews": 1,
                  "finished_reviews": 0,
                  "last_review_date": "0000-00-00T00:00:00Z"
                }

            else:
              if reviewer_login in reviewers_whitelist:
                reviewers_whitelist[reviewer_login]["assigned_reviews"] += 1
              else:
                reviewers_whitelist[reviewer_login] = {
                  "avatar_url": reviewer["avatar_url"],
                  "assigned_reviews": 1,
                  "finished_reviews": 0,
                  "last_review_date": "0000-00-00T00:00:00Z"
                }

          # Get reviews for the pull request, but count only once per PR
          pull_reviews_url = pull["url"] + "/reviews"
          pull_reviews, _ = get_api_response(pull_reviews_url)
          local_reviewers = {} # prevent double counting

          for review in pull_reviews:
            reviewer_login = review["user"]["login"]
            date = review["submitted_at"]

            if reviewer_login not in whitelist:
              if reviewer_login in reviewers:
                if reviewer_login not in local_reviewers:
                  reviewers[reviewer_login]["assigned_reviews"] += 1
                  reviewers[reviewer_login]["finished_reviews"] += 1
                  local_reviewers[reviewer_login] = True
                if date > reviewers[reviewer_login]["last_review_date"]:
                  reviewers[reviewer_login]["last_review_date"] = date
              else:
                reviewers[reviewer_login] = {
                  "avatar_url": review["user"]["avatar_url"],
                  "assigned_reviews": 1,
                  "finished_reviews": 1,
                  "last_review_date": date
                }
                local_reviewers[reviewer_login] = True

            else:
              if reviewer_login in reviewers_whitelist:
                if reviewer_login not in local_reviewers:
                  reviewers_whitelist[reviewer_login]["assigned_reviews"] += 1
                  reviewers_whitelist[reviewer_login]["finished_reviews"] += 1
                local_reviewers[reviewer_login] = True
                if date > reviewers_whitelist[reviewer_login]["last_review_date"]:
                  reviewers_whitelist[reviewer_login]["last_review_date"] = date
              else:
                reviewers_whitelist[reviewer_login] = {
                  "avatar_url": review["user"]["avatar_url"],
                  "assigned_reviews": 1,
                  "finished_reviews": 1,
                  "last_review_date": date
                }
                local_reviewers[reviewer_login] = True

  return reviewers, reviewers_whitelist, ct_pull

def create_reviewers_table_with_graph(reviewers_stats, user_names, table_name):
    """ style sheet for the table, copy into css file
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
                        <img src="{stats['avatar_url']}" width="36" height="36" alt="{reviewer}" style="border-radius: 50%;">
                      </div>
                      <div>
                        <div>
                          <b>{user_names[reviewer]}</b> <br>
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

def print_reviewers_stats(reviewers_stats):
  for reviewer, stats in sorted(reviewers_stats.items(), key=lambda x: x[1]['finished_reviews'], reverse=True)[:10]:
    print(f"Reviewer: {reviewer}, Assigned Reviews: {stats['assigned_reviews']}, Finished Reviews: {stats['finished_reviews']}, rate of finished: {stats['finished_reviews']/stats['assigned_reviews']:.2f}, Last Review Date: {stats['last_review_date']}")

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
  "kinematics_interface"
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
  "gz_ros2_control": "master",
  "kinematics_interface": "master"
}

maintainers = ["bmagyar", "destogl", "christophfroehlich"]

# Get the current date and time
current_date = datetime.utcnow()

# Calculate one year ago from the current date
one_year_ago = current_date - timedelta(days=365)

# Format the date string as "YYYY-MM-DDTHH:MM:SSZ"
formatted_date = one_year_ago.strftime("%Y-%m-%dT%H:%M:%SZ")

print("--------------------------")
print("--------  Start  ---------")
print("--------------------------")
print(f"Fetch pull requests after {formatted_date}:")
reviewers_stats_recent, maintainers_stats_recent, ct_pulls_recent = get_reviewers_stats(owner, repos, branches, maintainers, formatted_date)
print("--------------------------")
print(f"Fetch pull requests, all-time:")
reviewers_stats, maintainers_stats, ct_pulls = get_reviewers_stats(owner, repos, branches, maintainers)

print("--------------------------")
print("-------- Get User --------")
print("--------------------------")
unique_reviewers = set(
  list(reviewers_stats_recent.keys())
  + list(maintainers_stats_recent.keys())
  + list(reviewers_stats.keys())
  + list(maintainers_stats.keys())
  )
user_names = {}
for reviewer_login in unique_reviewers:
    user_names[reviewer_login] = get_user_name(reviewer_login)

print(f"Got {len(unique_reviewers)} user names")

# Print the reviewers' stats in a nice format
print("--------------------------")
print("-------- Results ---------")
print("--------------------------")
print(f"Reviewers' Stats from {ct_pulls_recent} pull requests after {formatted_date}:")
print("-------- maintainers ---------")
print_reviewers_stats(maintainers_stats_recent)

print("-------- not maintainers ---------")
print_reviewers_stats(reviewers_stats_recent)

print(f"Reviewers' Stats from {ct_pulls} pull requests, all-time:")
print("-------- maintainers ---------")
print_reviewers_stats(maintainers_stats)

print("-------- not maintainers ---------")
print_reviewers_stats(reviewers_stats)

# Create the HTML content
html_maintainers_stats_recent = create_reviewers_table_with_graph(maintainers_stats_recent, user_names, "maintainers_stats_recent")
html_reviewers_stats_recent = create_reviewers_table_with_graph(reviewers_stats_recent, user_names, "reviewers_stats_recent")
html_maintainers_stats = create_reviewers_table_with_graph(maintainers_stats, user_names, "maintainers_stats")
html_reviewers_stats = create_reviewers_table_with_graph(reviewers_stats, user_names, "reviewers_stats")

# Save the HTML content to a file named "reviewers_stats_with_graph.html"
home_directory = os.path.expanduser( '~' )
filename_maintainers_stats_recent = os.path.join(home_directory, 'reviews', 'maintainers_stats_recent.html')
filename_reviewers_stats_recent = os.path.join(home_directory, 'reviews', 'reviewers_stats_recent.html')
filename_maintainers_stats = os.path.join(home_directory, 'reviews', 'maintainers_stats.html')
filename_reviewers_stats = os.path.join(home_directory, 'reviews', 'reviewers_stats.html')
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
