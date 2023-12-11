import requests
from datetime import datetime, timedelta

def get_reviewers_stats(owner, repos, branches, earliest_date="0000-00-00T00:00:00Z"):

  reviewers = {}
  ct_pull = 0
  token = os.environ['GITHUB_TOKEN']
  headers = {
    "Accept": "application/vnd.github.v3+json",
    "Authorization": f"Bearer {token}"
  }

  for repo in repos:
    print(f"Getting reviewers' stats for {owner}/{repo} on branch {branches[repo]}")

    url = f"https://api.github.com/repos/{owner}/{repo}/pulls?state=closed&base={branches[repo]}&per_page=100"

    while url:
      response = requests.get(url, headers=headers)
      if response.status_code != 200:
        print(f"Error: {response.json()['message']}")
        break

      pulls = response.json()

      for pull in pulls:
        if pull["created_at"] > earliest_date:
          ct_pull += 1

          # parse requested reviewers
          reviewers_list = pull["requested_reviewers"]
          for reviewer in reviewers_list:
            reviewer_login = reviewer["login"]
            if reviewer_login in reviewers:
              reviewers[reviewer_login]["assigned_reviews"] += 1
            else:
              reviewers[reviewer_login] = {
                "assigned_reviews": 1,
                "finished_reviews": 0,
                "last_review_date": "0000-00-00T00:00:00Z"
              }

          # Get reviews for the pull request
          pull_reviews_url = pull["url"] + "/reviews"
          pull_reviews_response = requests.get(pull_reviews_url, headers=headers)
          if pull_reviews_response.status_code != 200:
            print(f"Error: {pull_reviews_response.json()['message']}")
            break
          pull_reviews = pull_reviews_response.json()

          local_reviewers = {}
          for review in pull_reviews:
            reviewer_login = review["user"]["login"]
            date = review["submitted_at"]
            if reviewer_login in reviewers:
              if reviewer_login not in local_reviewers:
                reviewers[reviewer_login]["assigned_reviews"] += 1
                reviewers[reviewer_login]["finished_reviews"] += 1
                local_reviewers[reviewer_login] = True
              if date > reviewers[reviewer_login]["last_review_date"]:
                reviewers[reviewer_login]["last_review_date"] = date
            else:
              reviewers[reviewer_login] = {
                "assigned_reviews": 1,
                "finished_reviews": 1,
                "last_review_date": date
              }
              local_reviewers[reviewer_login] = True

      # Check if there are more pages
      if "Link" in response.headers:
        links = response.headers["Link"].split(", ")
        for link in links:
          if "rel=\"next\"" in link:
            url = link[link.index("<") + 1 : link.index(">")]
            break
        else:
          url = None
      else:
        url = None

  return reviewers, ct_pull

def create_reviewers_table_with_graph(reviewers_stats):
    html_content = f"""
    <!DOCTYPE html>
    <html lang="en">
    <head>
        <meta charset="UTF-8">
        <title>Reviewers' Stats</title>
        <link rel="stylesheet" type="text/css" href="https://cdn.datatables.net/1.11.5/css/jquery.dataTables.css">
        <script src="https://code.jquery.com/jquery-3.6.0.min.js"></script>
        <script src="https://cdn.datatables.net/1.11.5/js/jquery.dataTables.js"></script>
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
    </head>
    <body>
        <h2>Reviewers' Stats</h2>
        <table id="reviewersTable" class="display">
            <thead>
                <tr>
                    <th>Reviewer</th>
                    <th>Assigned Reviews</th>
                    <th>Finished Reviews</th>
                    <th>Rate of Finished</th>
                    <th>Last Review Date</th>
                </tr>
            </thead>
            <tbody>
    """

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
                <td><a href="https://github.com/{reviewer}">{reviewer}</a> {medal}</td>
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
                <td>{stats['last_review_date']}</td>
            </tr>
        """

    html_content += """
            </tbody>
        </table>
        <script>
            $('#reviewersTable').DataTable({
                "order": [[2, "desc"]]
            });
        </script>
    </body>
    </html>
    """

    return html_content

# Replace with your GitHub repository owner and name
owner = "ros-controls"
repos = ["ros2_control", "ros2_controllers", "ros2_control_demos", "control_toolbox", "realtime_tools", "control_msgs", "control.ros.org", "gazebo_ros2_control", "gz_ros2_control", "kinematics_interface"]

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

# Get the current date and time
current_date = datetime.utcnow()

# Calculate one year ago from the current date
one_year_ago = current_date - timedelta(days=365)

# Format the date string as "YYYY-MM-DDTHH:MM:SSZ"
formatted_date = one_year_ago.strftime("%Y-%m-%dT%H:%M:%SZ")

reviewers_stats, ct_pulls = get_reviewers_stats(owner, repos, branches, formatted_date)

# Print the reviewers' stats in a nice format
print(f"Reviewers' Stats from {ct_pulls} pull requests after {formatted_date}:")
print("-----------------")
for reviewer, stats in sorted(reviewers_stats.items(), key=lambda x: x[1]['finished_reviews'], reverse=True)[:10]:
  print(f"Reviewer: {reviewer}, Assigned Reviews: {stats['assigned_reviews']}, Finished Reviews: {stats['finished_reviews']}, rate of finished: {stats['finished_reviews']/stats['assigned_reviews']}, Last Review Date: {stats['last_review_date']}")

# Create the HTML content
html_content_with_graph = create_reviewers_table_with_graph(reviewers_stats)

# Save the HTML content to a file named "reviewers_stats_with_graph.html"
with open('reviewers_stats_with_graph.html', 'w') as file:
    file.write(html_content_with_graph)

print("HTML file 'reviewers_stats_with_graph.html' has been created.")
