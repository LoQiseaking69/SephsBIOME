from flask import Flask, request, jsonify
import logging
import sqlite3
import subprocess
import requests
import os

app = Flask(__name__)
logging.basicConfig(filename='api.log', level=logging.INFO)

# Configure your database connection here
DATABASE_URL = os.getenv('SEPHSBIOME_DATABASE_URL', 'sephsbiome.db')
conn = sqlite3.connect(DATABASE_URL, check_same_thread=False)

@app.route('/github/dispatch-workflow', methods=['POST'])
def dispatch_github_workflow():
    try:
        workflow = request.json.get('workflow', 'main.yml')
        github_token = os.getenv('GITHUB_TOKEN')
        repo = os.getenv('GITHUB_REPOSITORY', 'username/repo')
        response = requests.post(
            f'https://api.github.com/repos/{repo}/actions/workflows/{workflow}/dispatches',
            headers={'Authorization': f'token {github_token}'},
            json={'ref': 'main'}
        )
        return jsonify(response.json())
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/vscode/command', methods=['POST'])
def vscode_command():
    try:
        command = request.json.get('command')
        result = subprocess.run(["code", "--command", command], capture_output=True, text=True)
        return jsonify({"stdout": result.stdout, "stderr": result.stderr})
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/sentry/issues', methods=['GET'])
def sentry_issues():
    try:
        sentry_auth_token = os.getenv('SENTRY_AUTH_TOKEN')
        organization_slug = os.getenv('SENTRY_ORG_SLUG', 'your-org')
        project_slug = os.getenv('SENTRY_PROJECT_SLUG', 'your-project')
        response = requests.get(
            f'https://sentry.io/api/0/projects/{organization_slug}/{project_slug}/issues/',
            headers={'Authorization': f'Bearer {sentry_auth_token}'}
        )
        return jsonify(response.json())
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/sqlite/query', methods=['POST'])
def sqlite_query():
    try:
        query = request.json.get('query')
        cursor = conn.cursor()
        cursor.execute(query)
        rows = cursor.fetchall()
        return jsonify({"rows": rows})
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/docker/run-container', methods=['POST'])
def docker_run_container():
    try:
        image = request.json.get('image')
        command = request.json.get('command', '')
        result = subprocess.run(["docker", "run", image, command], capture_output=True, text=True)
        return jsonify({"stdout": result.stdout, "stderr": result.stderr})
    except Exception as e:
        return jsonify({"error": str(e)}), 500

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5000)
