openapi: 3.0.0
info:
  title: SEPHSbiome Development Assistant API
  description: A comprehensive API for managing SEPHSbiome project development and operations.
  version: 1.0.0
servers:
  - url: http://localhost:5000
    description: Local Development Server
paths:
  /github/dispatch-workflow:
    post:
      summary: Triggers a GitHub Actions workflow.
      requestBody:
        required: true
        content:
          application/json:
            schema:
              type: object
              properties:
                workflow:
                  type: string
                  description: The name of the workflow file to dispatch.
                  example: 'main.yml'
      responses:
        '200':
          description: Workflow dispatched successfully.
  /vscode/command:
    post:
      summary: Executes a Visual Studio Code command.
      requestBody:
        required: true
        content:
          application/json:
            schema:
              type: object
              properties:
                command:
                  type: string
                  description: The VSCode command to execute.
                  example: 'extension.install'
      responses:
        '200':
          description: VSCode command executed successfully.
  /sentry/issues:
    get:
      summary: Retrieves a list of issues from Sentry.
      responses:
        '200':
          description: List of Sentry issues.
  /sqlite/query:
    post:
      summary: Executes a query against the SQLite database.
      requestBody:
        required: true
        content:
          application/json:
            schema:
              type: object
              properties:
                query:
                  type: string
                  description: The SQLite query to execute.
                  example: 'SELECT * FROM simulation_results'
      responses:
        '200':
          description: Query executed successfully, results returned.
  /docker/run-container:
    post:
      summary: Runs a Docker container with the specified image and command.
      requestBody:
        required: true
        content:
          application/json:
            schema:
              type: object
              properties:
                image:
                  type: string
                  description: The Docker image to run.
                  example: 'python:3.8-slim'
                command:
                  type: string
                  description: The command to run inside the container.
                  example: 'python script.py'
      responses:
        '200':
          description: Docker container ran successfully.