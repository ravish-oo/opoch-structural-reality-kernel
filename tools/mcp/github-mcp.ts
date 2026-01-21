import { Server } from "@modelcontextprotocol/sdk/server/index.js";
import { StdioServerTransport } from "@modelcontextprotocol/sdk/server/stdio.js";
import {
  CallToolRequestSchema,
  ErrorCode,
  ListToolsRequestSchema,
  McpError,
} from "@modelcontextprotocol/sdk/types.js";
import { Octokit } from "@octokit/rest";
import { config } from "dotenv";
import { fileURLToPath } from "url";
import { dirname, join } from "path";

const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);

// Load environment variables
config({ path: join(dirname(__dirname), ".env.local") });

const GITHUB_TOKEN = process.env.GITHUB_TOKEN;
const REPO_OWNER = "dvcoolster";
const REPO_NAME = "opoch-website";

if (!GITHUB_TOKEN) {
  console.error("Missing GITHUB_TOKEN environment variable");
  process.exit(1);
}

// Initialize Octokit
const octokit = new Octokit({
  auth: GITHUB_TOKEN,
});

class GitHubMCPServer {
  private server: Server;

  constructor() {
    this.server = new Server(
      {
        name: "github-mcp",
        version: "1.0.0",
      },
      {
        capabilities: {
          tools: {},
        },
      }
    );

    this.setupHandlers();
  }

  private setupHandlers() {
    this.server.setRequestHandler(ListToolsRequestSchema, async () => ({
      tools: [
        {
          name: "gh_workflow_runs",
          description: "Get recent GitHub Actions workflow runs",
          inputSchema: {
            type: "object",
            properties: {
              limit: {
                type: "number",
                description: "Number of runs to fetch (default: 10)",
                default: 10,
              },
            },
          },
        },
        {
          name: "gh_workflow_run_logs",
          description: "Get logs for a specific workflow run",
          inputSchema: {
            type: "object",
            properties: {
              run_id: {
                type: "number",
                description: "The workflow run ID",
              },
            },
            required: ["run_id"],
          },
        },
        {
          name: "gh_deployments",
          description: "Get recent deployments",
          inputSchema: {
            type: "object",
            properties: {
              limit: {
                type: "number",
                description: "Number of deployments to fetch (default: 10)",
                default: 10,
              },
            },
          },
        },
        {
          name: "gh_deployment_status",
          description: "Get deployment statuses",
          inputSchema: {
            type: "object",
            properties: {
              deployment_id: {
                type: "number",
                description: "The deployment ID",
              },
            },
            required: ["deployment_id"],
          },
        },
        {
          name: "gh_commits",
          description: "Get recent commits",
          inputSchema: {
            type: "object",
            properties: {
              limit: {
                type: "number",
                description: "Number of commits to fetch (default: 10)",
                default: 10,
              },
            },
          },
        },
        {
          name: "gh_issues",
          description: "Get repository issues",
          inputSchema: {
            type: "object",
            properties: {
              state: {
                type: "string",
                description: "Issue state: open, closed, or all",
                default: "open",
              },
              limit: {
                type: "number",
                description: "Number of issues to fetch (default: 10)",
                default: 10,
              },
            },
          },
        },
        {
          name: "gh_pull_requests",
          description: "Get repository pull requests",
          inputSchema: {
            type: "object",
            properties: {
              state: {
                type: "string",
                description: "PR state: open, closed, or all",
                default: "open",
              },
              limit: {
                type: "number",
                description: "Number of PRs to fetch (default: 10)",
                default: 10,
              },
            },
          },
        },
      ],
    }));

    this.server.setRequestHandler(CallToolRequestSchema, async (request) => {
      const { name, arguments: args } = request.params;

      try {
        switch (name) {
          case "gh_workflow_runs":
            return await this.getWorkflowRuns(args);
          case "gh_workflow_run_logs":
            return await this.getWorkflowRunLogs(args);
          case "gh_deployments":
            return await this.getDeployments(args);
          case "gh_deployment_status":
            return await this.getDeploymentStatus(args);
          case "gh_commits":
            return await this.getCommits(args);
          case "gh_issues":
            return await this.getIssues(args);
          case "gh_pull_requests":
            return await this.getPullRequests(args);
          default:
            throw new McpError(
              ErrorCode.MethodNotFound,
              `Unknown tool: ${name}`
            );
        }
      } catch (error: any) {
        throw new McpError(
          ErrorCode.InternalError,
          error.message || "Operation failed"
        );
      }
    });
  }

  private async getWorkflowRuns(args: any) {
    const limit = args.limit || 10;
    const runs = await octokit.actions.listWorkflowRunsForRepo({
      owner: REPO_OWNER,
      repo: REPO_NAME,
      per_page: limit,
    });

    return {
      content: [
        {
          type: "text",
          text: JSON.stringify(
            runs.data.workflow_runs.map((run) => ({
              id: run.id,
              name: run.name,
              status: run.status,
              conclusion: run.conclusion,
              created_at: run.created_at,
              updated_at: run.updated_at,
              html_url: run.html_url,
              head_branch: run.head_branch,
              head_sha: run.head_sha,
              event: run.event,
            })),
            null,
            2
          ),
        },
      ],
    };
  }

  private async getWorkflowRunLogs(args: any) {
    if (!args.run_id) {
      throw new Error("run_id is required");
    }

    try {
      // Get workflow run details
      const run = await octokit.actions.getWorkflowRun({
        owner: REPO_OWNER,
        repo: REPO_NAME,
        run_id: args.run_id,
      });

      // Get jobs for this run
      const jobs = await octokit.actions.listJobsForWorkflowRun({
        owner: REPO_OWNER,
        repo: REPO_NAME,
        run_id: args.run_id,
      });

      return {
        content: [
          {
            type: "text",
            text: JSON.stringify(
              {
                run: {
                  id: run.data.id,
                  name: run.data.name,
                  status: run.data.status,
                  conclusion: run.data.conclusion,
                  created_at: run.data.created_at,
                  updated_at: run.data.updated_at,
                },
                jobs: jobs.data.jobs.map((job) => ({
                  id: job.id,
                  name: job.name,
                  status: job.status,
                  conclusion: job.conclusion,
                  started_at: job.started_at,
                  completed_at: job.completed_at,
                  steps: job.steps?.map((step) => ({
                    name: step.name,
                    status: step.status,
                    conclusion: step.conclusion,
                    number: step.number,
                  })),
                })),
              },
              null,
              2
            ),
          },
        ],
      };
    } catch (error: any) {
      throw new Error(`Failed to get workflow run logs: ${error.message}`);
    }
  }

  private async getDeployments(args: any) {
    const limit = args.limit || 10;
    const deployments = await octokit.repos.listDeployments({
      owner: REPO_OWNER,
      repo: REPO_NAME,
      per_page: limit,
    });

    return {
      content: [
        {
          type: "text",
          text: JSON.stringify(
            deployments.data.map((deployment) => ({
              id: deployment.id,
              ref: deployment.ref,
              task: deployment.task,
              environment: deployment.environment,
              created_at: deployment.created_at,
              updated_at: deployment.updated_at,
              creator: deployment.creator?.login,
            })),
            null,
            2
          ),
        },
      ],
    };
  }

  private async getDeploymentStatus(args: any) {
    if (!args.deployment_id) {
      throw new Error("deployment_id is required");
    }

    const statuses = await octokit.repos.listDeploymentStatuses({
      owner: REPO_OWNER,
      repo: REPO_NAME,
      deployment_id: args.deployment_id,
    });

    return {
      content: [
        {
          type: "text",
          text: JSON.stringify(
            statuses.data.map((status) => ({
              id: status.id,
              state: status.state,
              description: status.description,
              created_at: status.created_at,
              updated_at: status.updated_at,
              target_url: status.target_url,
              log_url: status.log_url,
            })),
            null,
            2
          ),
        },
      ],
    };
  }

  private async getCommits(args: any) {
    const limit = args.limit || 10;
    const commits = await octokit.repos.listCommits({
      owner: REPO_OWNER,
      repo: REPO_NAME,
      per_page: limit,
    });

    return {
      content: [
        {
          type: "text",
          text: JSON.stringify(
            commits.data.map((commit) => ({
              sha: commit.sha,
              message: commit.commit.message,
              author: commit.commit.author?.name,
              date: commit.commit.author?.date,
              html_url: commit.html_url,
            })),
            null,
            2
          ),
        },
      ],
    };
  }

  private async getIssues(args: any) {
    const limit = args.limit || 10;
    const state = args.state || "open";
    const issues = await octokit.issues.listForRepo({
      owner: REPO_OWNER,
      repo: REPO_NAME,
      state: state as "open" | "closed" | "all",
      per_page: limit,
    });

    return {
      content: [
        {
          type: "text",
          text: JSON.stringify(
            issues.data.map((issue) => ({
              number: issue.number,
              title: issue.title,
              state: issue.state,
              created_at: issue.created_at,
              updated_at: issue.updated_at,
              html_url: issue.html_url,
              user: issue.user?.login,
              labels: issue.labels.map((label) =>
                typeof label === "string" ? label : label.name
              ),
            })),
            null,
            2
          ),
        },
      ],
    };
  }

  private async getPullRequests(args: any) {
    const limit = args.limit || 10;
    const state = args.state || "open";
    const prs = await octokit.pulls.list({
      owner: REPO_OWNER,
      repo: REPO_NAME,
      state: state as "open" | "closed" | "all",
      per_page: limit,
    });

    return {
      content: [
        {
          type: "text",
          text: JSON.stringify(
            prs.data.map((pr) => ({
              number: pr.number,
              title: pr.title,
              state: pr.state,
              created_at: pr.created_at,
              updated_at: pr.updated_at,
              merged_at: pr.merged_at,
              html_url: pr.html_url,
              user: pr.user?.login,
              head: pr.head.ref,
              base: pr.base.ref,
            })),
            null,
            2
          ),
        },
      ],
    };
  }

  async run() {
    const transport = new StdioServerTransport();
    await this.server.connect(transport);
    console.error("GitHub MCP server running on stdio");
  }
}

const server = new GitHubMCPServer();
server.run().catch(console.error);