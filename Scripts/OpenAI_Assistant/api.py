from flask import Flask, request, jsonify
import logging
import sqlite3
import subprocess
import requests
import os
from flask_jwt_extended import JWTManager, create_access_token, jwt_required
import json

app = Flask(__name__)
logging.basicConfig(level=logging.INFO)

# Database and JWT configuration
DATABASE_URL = os.getenv('SEPHSBIOME_DATABASE_URL', 'sephsbiome.db')
app.config['JWT_SECRET_KEY'] = os.getenv('JWT_SECRET_KEY', 'super-secret-key')
jwt = JWTManager(app)

# Connect to SQLite database
def get_db_connection():
    conn = sqlite3.connect(DATABASE_URL, check_same_thread=False)
    conn.row_factory = sqlite3.Row
    return conn

@app.route('/api/v1/github/dispatch-workflow', methods=['POST'])
def dispatch_github_workflow():
    # Implementation for dispatching GitHub workflow
    workflow = request.json['workflow']
    github_token = os.getenv('GITHUB_TOKEN')
    repo = os.getenv('GITHUB_REPOSITORY', 'username/repo')
    response = requests.post(
        f'https://api.github.com/repos/{repo}/actions/workflows/{workflow}/dispatches',
        headers={'Authorization': f'token {github_token}'},
        json={'ref': 'main'}
    )
    if response.status_code != 204:
        logging.error("Failed to dispatch GitHub workflow")
        return jsonify({"error": "Failed to dispatch workflow"}), 500
    return jsonify({"message": "Workflow dispatched successfully"})

@app.route('/api/v1/vscode/command', methods=['POST'])
def vscode_command():
    # Implementation for executing VSCode command
    command = request.json['command']
    result = subprocess.run(["code", "--command", command], capture_output=True, text=True)
    if result.returncode != 0:
        logging.error("Failed to execute VSCode command")
        return jsonify({"error": "Failed to execute command", "stderr": result.stderr}), 500
    return jsonify({"stdout": result.stdout})

@app.route('/api/v1/sentry/issues', methods=['GET'])
def sentry_issues():
    # Implementation for retrieving Sentry issues
    sentry_auth_token = os.getenv('SENTRY_AUTH_TOKEN')
    organization_slug = os.getenv('SENTRY_ORG_SLUG', 'your-org')
    project_slug = os.getenv('SENTRY_PROJECT_SLUG', 'your-project')
    response = requests.get(
        f'https://sentry.io/api/0/projects/{organization_slug}/{project_slug}/issues/',
        headers={'Authorization': f'Bearer {sentry_auth_token}'}
    )
    if response.status_code != 200:
        logging.error("Failed to retrieve Sentry issues")
        return jsonify({"error": "Failed to retrieve Sentry issues"}), 500
    return jsonify(response.json())

@app.route('/api/v1/sqlite/query', methods=['POST'])
def sqlite_query():
    # Implementation for executing SQLite query
    query = request.json['query']
    conn = get_db_connection()
    cursor = conn.cursor()
    cursor.execute(query)
    rows = cursor.fetchall()
    conn.close()
    return jsonify({"rows": [dict(ix) for ix in rows]})

@app.route('/api/v1/docker/run-container', methods=['POST'])
def docker_run_container():
    # Implementation for running Docker container
    image = request.json['image']
    command = request.json.get('command', '')
    result = subprocess.run(["docker", "run", image, command], capture_output=True, text=True)
    if result.returncode != 0:
        logging.error("Failed to run Docker container")
        return jsonify({"error": "Failed to run Docker container", "stderr": result.stderr}), 500
    return jsonify({"stdout": result.stdout})

@app.route('/api/v1/webhooks/github', methods=['POST'])
def handle_github_webhook():
    # Logic to handle GitHub webhook
    event = request.json.get('event')
    payload = request.json.get('payload')
    # Process the GitHub webhook event and payload
    return jsonify({"message": f"GitHub event {event} processed"}), 200

@app.route('/api/v1/ci/run-pipeline', methods=['POST'])
def run_pipeline():
    # Logic to trigger the CI/CD pipeline
    pipeline_name = request.json.get('pipeline')
    # Placeholder for pipeline triggering logic
    return jsonify({"message": f"Pipeline {pipeline_name} triggered"}), 200

@app.route('/api/v1/logs', methods=['GET'])
def get_logs():
    # Logic to fetch log data
    log_level = request.args.get('level')
    # Placeholder for log fetching logic based on log_level
    return jsonify({"logs": f"Logs for level {log_level}"}), 200

@app.route('/api/v1/auth/login', methods=['POST'])
def login_user():
    # User authentication logic
    username = request.json.get('username')
    password = request.json.get('password')
    # Placeholder for real authentication check
    if username == "admin" and password == "password":  # Example check
        access_token = create_access_token(identity=username)
        return jsonify(access_token=access_token), 200
    else:
        return jsonify({"msg": "Bad username or password"}), 401

@app.route('/api/v1/cache/clear', methods=['POST'])
def clear_cache():
    # Logic to clear cache
    # Placeholder for cache clearing logic
    return jsonify({"message": "Cache cleared"}), 200

@app.route('/api/v1/tests/run', methods=['POST'])
def run_tests():
    # Logic to run specified test suites
    test_suite = request.json.get('suite')
    # Placeholder for test execution logic
    return jsonify({"message": f"Test suite {test_suite} executed"}), 200

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5000)
