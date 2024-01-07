from flask import Flask, request, jsonify
import os
import subprocess
import base64
import logging
import sqlite3
import hashlib
from dotenv import load_dotenv, set_key
from flask_jwt_extended import JWTManager, create_access_token, jwt_required
import requests

# Attempt to import project-specific modules if they exist
try:
    from .src import simulator
    from .src import performanceViz
    from .src import evolution
except ImportError:
    simulator = performanceViz = evolution = None

app = Flask(__name__)
logging.basicConfig(level=logging.INFO)

# Dynamic JWT Secret Generation and Saving to .env File
def generate_jwt_secret():
    return base64.b64encode(os.urandom(32)).decode('utf-8')

def create_or_update_env_file(key, value):
    env_file = '.env'
    if not os.path.isfile(env_file):
        with open(env_file, 'w') as file:
            file.write(f'{key}={value}\n')
    else:
        set_key(env_file, key, value)

def install_vscode_cli():
    try:
        subprocess.run(["code", "--version"], check=True)
    except subprocess.CalledProcessError:
        logging.info("VSCode CLI not found. Installing VSCode...")
        subprocess.run(["sudo", "apt", "update"])
        subprocess.run(["sudo", "apt", "install", "-y", "code"])
        logging.info("VSCode installed successfully")

# Load existing env vars, if any
load_dotenv('.env')

# Generate and save JWT secret if not present
if 'JWT_SECRET_KEY' not in os.environ:
    jwt_secret = generate_jwt_secret()
    create_or_update_env_file('JWT_SECRET_KEY', jwt_secret)
    os.environ['JWT_SECRET_KEY'] = jwt_secret
    logging.info("Generated and saved a new JWT secret key")

install_vscode_cli()

# Database and JWT configuration
DATABASE_URL = os.getenv('SEPHSBIOME_DATABASE_URL', 'sephsbiome.db')
app.config['JWT_SECRET_KEY'] = os.getenv('JWT_SECRET_KEY')
jwt = JWTManager(app)

def get_db_connection():
    conn = sqlite3.connect(DATABASE_URL, check_same_thread=False)
    conn.row_factory = sqlite3.Row
    return conn

def create_tables():
    conn = get_db_connection()
    cursor = conn.cursor()
    create_table_query = """
    CREATE TABLE IF NOT EXISTS users (
        id INTEGER PRIMARY KEY,
        username TEXT,
        password TEXT
    );
    """
    cursor.execute(create_table_query)
    conn.commit()
    conn.close()

# GitHub Workflow Dispatch
@app.route('/api/v1/github/dispatch-workflow', methods=['POST'])
def dispatch_github_workflow():
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

# VSCode Command
@app.route('/api/v1/vscode/command', methods=['POST'])
def vscode_command():
    command = request.json['command']
    result = subprocess.run(["code", "--command", command], capture_output=True, text=True)
    if result.returncode != 0:
        logging.error("Failed to execute VSCode command")
        return jsonify({"error": "Failed to execute command", "stderr": result.stderr}), 500
    return jsonify({"stdout": result.stdout})

# Sentry Issues
@app.route('/api/v1/sentry/issues', methods=['GET'])
def sentry_issues():
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

# SQLite Query
@app.route('/api/v1/sqlite/query', methods=['POST'])
def sqlite_query():
    query = request.json['query']
    conn = get_db_connection()
    cursor = conn.cursor()
    try:
        cursor.execute(query)
        rows = cursor.fetchall()
    except sqlite3.Error as e:
        return jsonify({"error": str(e)}), 500
    finally:
        conn.close()
    return jsonify({"rows": [dict(ix) for ix in rows]})

# Docker Run Container
@app.route('/api/v1/docker/run-container', methods=['POST'])
def docker_run_container():
    image = request.json['image']
    command = request.json.get('command', '')
    result = subprocess.run(["docker", "run", image, command], capture_output=True, text=True)
    if result.returncode != 0:
        logging.error("Failed to run Docker container")
        return jsonify({"error": "Failed to run Docker container", "stderr": result.stderr}), 500
    return jsonify({"stdout": result.stdout})

# GitHub Webhooks
@app.route('/api/v1/webhooks/github', methods=['POST'])
def handle_github_webhook():
    event = request.json.get('event')
    payload = request.json.get('payload')
    logging.info(f"Received GitHub event: {event}")
    return jsonify({"message": f"GitHub event {event} processed"}), 200

# CI/CD Run Pipeline
@app.route('/api/v1/ci/run-pipeline', methods=['POST'])
def run_pipeline():
    pipeline_name = request.json.get('pipeline')
    logging.info(f"Triggering pipeline: {pipeline_name}")
    return jsonify({"message": f"Pipeline {pipeline_name} triggered"}), 200

# Logs Retrieval
@app.route('/api/v1/logs', methods=['GET'])
def get_logs():
    log_level = request.args.get('level')
    return jsonify({"logs": f"Logs for level {log_level}"}), 200

# User Login
@app.route('/api/v1/auth/login', methods=['POST'])
def login_user():
    username = request.json.get('username')
    password = request.json.get('password')
    hashed_password = hashlib.sha256(password.encode()).hexdigest()
    conn = get_db_connection()
    cursor = conn.cursor()
    cursor.execute("SELECT * FROM users WHERE username = ?", (username,))
    user = cursor.fetchone()
    conn.close()
    if user and user['password'] == hashed_password:
        access_token = create_access_token(identity=username)
        return jsonify(access_token=access_token), 200
    else:
        return jsonify({"msg": "Bad username or password"}), 401

# Cache Clear
@app.route('/api/v1/cache/clear', methods=['POST'])
def clear_cache():
    logging.info("Cache cleared")
    return jsonify({"message": "Cache cleared"}), 200

# Run Tests
@app.route('/api/v1/tests/run', methods=['POST'])
def run_tests():
    test_suite = request.json.get('suite')
    logging.info(f"Executing test suite: {test_suite}")
    return jsonify({"message": f"Test suite {test_suite} executed"}), 200

# SEPHSbiome-specific endpoints

@app.route('/api/v1/sephsbiome/simulation', methods=['POST'])
@jwt_required()
def run_simulation_endpoint():
    data = request.json
    simulator_instance = simulator.Simulator(data['population_size'], data['seq_length'], data['d_model'])
    result = simulator_instance.run_simulation(data['generations'])
    return jsonify({"message": "Simulation run completed", "result": result}), 200

@app.route('/api/v1/sephsbiome/visualization', methods=['GET'])
@jwt_required()
def visualization_endpoint():
    viz = performanceViz.PerformanceVisualizer()
    visualization_result = viz.visualize_data()
    return jsonify({"visualization": visualization_result}), 200

@app.route('/api/v1/sephsbiome/genetic-algorithm', methods=['POST'])
@jwt_required()
def run_evolution_endpoint():
    data = request.json
    evolution_instance = evolution.Evolution(data['population_size'], data['seq_length'], data['d_model'])
    result = evolution_instance.run_evolution(data['generations'])
    return jsonify({"message": "Evolution process executed", "result": result}), 200

if __name__ == '__main__':
    create_tables()  # Create tables when the script is run
    app.run(debug=True, host='0.0.0.0', port=5000)
