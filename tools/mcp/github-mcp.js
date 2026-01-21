"use strict";
var __awaiter = (this && this.__awaiter) || function (thisArg, _arguments, P, generator) {
    function adopt(value) { return value instanceof P ? value : new P(function (resolve) { resolve(value); }); }
    return new (P || (P = Promise))(function (resolve, reject) {
        function fulfilled(value) { try { step(generator.next(value)); } catch (e) { reject(e); } }
        function rejected(value) { try { step(generator["throw"](value)); } catch (e) { reject(e); } }
        function step(result) { result.done ? resolve(result.value) : adopt(result.value).then(fulfilled, rejected); }
        step((generator = generator.apply(thisArg, _arguments || [])).next());
    });
};
var __generator = (this && this.__generator) || function (thisArg, body) {
    var _ = { label: 0, sent: function() { if (t[0] & 1) throw t[1]; return t[1]; }, trys: [], ops: [] }, f, y, t, g = Object.create((typeof Iterator === "function" ? Iterator : Object).prototype);
    return g.next = verb(0), g["throw"] = verb(1), g["return"] = verb(2), typeof Symbol === "function" && (g[Symbol.iterator] = function() { return this; }), g;
    function verb(n) { return function (v) { return step([n, v]); }; }
    function step(op) {
        if (f) throw new TypeError("Generator is already executing.");
        while (g && (g = 0, op[0] && (_ = 0)), _) try {
            if (f = 1, y && (t = op[0] & 2 ? y["return"] : op[0] ? y["throw"] || ((t = y["return"]) && t.call(y), 0) : y.next) && !(t = t.call(y, op[1])).done) return t;
            if (y = 0, t) op = [op[0] & 2, t.value];
            switch (op[0]) {
                case 0: case 1: t = op; break;
                case 4: _.label++; return { value: op[1], done: false };
                case 5: _.label++; y = op[1]; op = [0]; continue;
                case 7: op = _.ops.pop(); _.trys.pop(); continue;
                default:
                    if (!(t = _.trys, t = t.length > 0 && t[t.length - 1]) && (op[0] === 6 || op[0] === 2)) { _ = 0; continue; }
                    if (op[0] === 3 && (!t || (op[1] > t[0] && op[1] < t[3]))) { _.label = op[1]; break; }
                    if (op[0] === 6 && _.label < t[1]) { _.label = t[1]; t = op; break; }
                    if (t && _.label < t[2]) { _.label = t[2]; _.ops.push(op); break; }
                    if (t[2]) _.ops.pop();
                    _.trys.pop(); continue;
            }
            op = body.call(thisArg, _);
        } catch (e) { op = [6, e]; y = 0; } finally { f = t = 0; }
        if (op[0] & 5) throw op[1]; return { value: op[0] ? op[1] : void 0, done: true };
    }
};
Object.defineProperty(exports, "__esModule", { value: true });
var index_js_1 = require("@modelcontextprotocol/sdk/server/index.js");
var stdio_js_1 = require("@modelcontextprotocol/sdk/server/stdio.js");
var types_js_1 = require("@modelcontextprotocol/sdk/types.js");
var rest_1 = require("@octokit/rest");
var dotenv_1 = require("dotenv");
var url_1 = require("url");
var path_1 = require("path");
var __filename = (0, url_1.fileURLToPath)(import.meta.url);
var __dirname = (0, path_1.dirname)(__filename);
// Load environment variables
(0, dotenv_1.config)({ path: (0, path_1.join)((0, path_1.dirname)(__dirname), ".env.local") });
var GITHUB_TOKEN = process.env.GITHUB_TOKEN;
var REPO_OWNER = "dvcoolster";
var REPO_NAME = "opoch-website";
if (!GITHUB_TOKEN) {
    console.error("Missing GITHUB_TOKEN environment variable");
    process.exit(1);
}
// Initialize Octokit
var octokit = new rest_1.Octokit({
    auth: GITHUB_TOKEN,
});
var GitHubMCPServer = /** @class */ (function () {
    function GitHubMCPServer() {
        this.server = new index_js_1.Server({
            name: "github-mcp",
            version: "1.0.0",
        }, {
            capabilities: {
                tools: {},
            },
        });
        this.setupHandlers();
    }
    GitHubMCPServer.prototype.setupHandlers = function () {
        var _this = this;
        this.server.setRequestHandler(types_js_1.ListToolsRequestSchema, function () { return __awaiter(_this, void 0, void 0, function () {
            return __generator(this, function (_a) {
                return [2 /*return*/, ({
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
                    })];
            });
        }); });
        this.server.setRequestHandler(types_js_1.CallToolRequestSchema, function (request) { return __awaiter(_this, void 0, void 0, function () {
            var _a, name, args, _b, error_1;
            return __generator(this, function (_c) {
                switch (_c.label) {
                    case 0:
                        _a = request.params, name = _a.name, args = _a.arguments;
                        _c.label = 1;
                    case 1:
                        _c.trys.push([1, 18, , 19]);
                        _b = name;
                        switch (_b) {
                            case "gh_workflow_runs": return [3 /*break*/, 2];
                            case "gh_workflow_run_logs": return [3 /*break*/, 4];
                            case "gh_deployments": return [3 /*break*/, 6];
                            case "gh_deployment_status": return [3 /*break*/, 8];
                            case "gh_commits": return [3 /*break*/, 10];
                            case "gh_issues": return [3 /*break*/, 12];
                            case "gh_pull_requests": return [3 /*break*/, 14];
                        }
                        return [3 /*break*/, 16];
                    case 2: return [4 /*yield*/, this.getWorkflowRuns(args)];
                    case 3: return [2 /*return*/, _c.sent()];
                    case 4: return [4 /*yield*/, this.getWorkflowRunLogs(args)];
                    case 5: return [2 /*return*/, _c.sent()];
                    case 6: return [4 /*yield*/, this.getDeployments(args)];
                    case 7: return [2 /*return*/, _c.sent()];
                    case 8: return [4 /*yield*/, this.getDeploymentStatus(args)];
                    case 9: return [2 /*return*/, _c.sent()];
                    case 10: return [4 /*yield*/, this.getCommits(args)];
                    case 11: return [2 /*return*/, _c.sent()];
                    case 12: return [4 /*yield*/, this.getIssues(args)];
                    case 13: return [2 /*return*/, _c.sent()];
                    case 14: return [4 /*yield*/, this.getPullRequests(args)];
                    case 15: return [2 /*return*/, _c.sent()];
                    case 16: throw new types_js_1.McpError(types_js_1.ErrorCode.MethodNotFound, "Unknown tool: ".concat(name));
                    case 17: return [3 /*break*/, 19];
                    case 18:
                        error_1 = _c.sent();
                        throw new types_js_1.McpError(types_js_1.ErrorCode.InternalError, error_1.message || "Operation failed");
                    case 19: return [2 /*return*/];
                }
            });
        }); });
    };
    GitHubMCPServer.prototype.getWorkflowRuns = function (args) {
        return __awaiter(this, void 0, void 0, function () {
            var limit, runs;
            return __generator(this, function (_a) {
                switch (_a.label) {
                    case 0:
                        limit = args.limit || 10;
                        return [4 /*yield*/, octokit.actions.listWorkflowRunsForRepo({
                                owner: REPO_OWNER,
                                repo: REPO_NAME,
                                per_page: limit,
                            })];
                    case 1:
                        runs = _a.sent();
                        return [2 /*return*/, {
                                content: [
                                    {
                                        type: "text",
                                        text: JSON.stringify(runs.data.workflow_runs.map(function (run) { return ({
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
                                        }); }), null, 2),
                                    },
                                ],
                            }];
                }
            });
        });
    };
    GitHubMCPServer.prototype.getWorkflowRunLogs = function (args) {
        return __awaiter(this, void 0, void 0, function () {
            var run, jobs, error_2;
            return __generator(this, function (_a) {
                switch (_a.label) {
                    case 0:
                        if (!args.run_id) {
                            throw new Error("run_id is required");
                        }
                        _a.label = 1;
                    case 1:
                        _a.trys.push([1, 4, , 5]);
                        return [4 /*yield*/, octokit.actions.getWorkflowRun({
                                owner: REPO_OWNER,
                                repo: REPO_NAME,
                                run_id: args.run_id,
                            })];
                    case 2:
                        run = _a.sent();
                        return [4 /*yield*/, octokit.actions.listJobsForWorkflowRun({
                                owner: REPO_OWNER,
                                repo: REPO_NAME,
                                run_id: args.run_id,
                            })];
                    case 3:
                        jobs = _a.sent();
                        return [2 /*return*/, {
                                content: [
                                    {
                                        type: "text",
                                        text: JSON.stringify({
                                            run: {
                                                id: run.data.id,
                                                name: run.data.name,
                                                status: run.data.status,
                                                conclusion: run.data.conclusion,
                                                created_at: run.data.created_at,
                                                updated_at: run.data.updated_at,
                                            },
                                            jobs: jobs.data.jobs.map(function (job) {
                                                var _a;
                                                return ({
                                                    id: job.id,
                                                    name: job.name,
                                                    status: job.status,
                                                    conclusion: job.conclusion,
                                                    started_at: job.started_at,
                                                    completed_at: job.completed_at,
                                                    steps: (_a = job.steps) === null || _a === void 0 ? void 0 : _a.map(function (step) { return ({
                                                        name: step.name,
                                                        status: step.status,
                                                        conclusion: step.conclusion,
                                                        number: step.number,
                                                    }); }),
                                                });
                                            }),
                                        }, null, 2),
                                    },
                                ],
                            }];
                    case 4:
                        error_2 = _a.sent();
                        throw new Error("Failed to get workflow run logs: ".concat(error_2.message));
                    case 5: return [2 /*return*/];
                }
            });
        });
    };
    GitHubMCPServer.prototype.getDeployments = function (args) {
        return __awaiter(this, void 0, void 0, function () {
            var limit, deployments;
            return __generator(this, function (_a) {
                switch (_a.label) {
                    case 0:
                        limit = args.limit || 10;
                        return [4 /*yield*/, octokit.repos.listDeployments({
                                owner: REPO_OWNER,
                                repo: REPO_NAME,
                                per_page: limit,
                            })];
                    case 1:
                        deployments = _a.sent();
                        return [2 /*return*/, {
                                content: [
                                    {
                                        type: "text",
                                        text: JSON.stringify(deployments.data.map(function (deployment) {
                                            var _a;
                                            return ({
                                                id: deployment.id,
                                                ref: deployment.ref,
                                                task: deployment.task,
                                                environment: deployment.environment,
                                                created_at: deployment.created_at,
                                                updated_at: deployment.updated_at,
                                                creator: (_a = deployment.creator) === null || _a === void 0 ? void 0 : _a.login,
                                            });
                                        }), null, 2),
                                    },
                                ],
                            }];
                }
            });
        });
    };
    GitHubMCPServer.prototype.getDeploymentStatus = function (args) {
        return __awaiter(this, void 0, void 0, function () {
            var statuses;
            return __generator(this, function (_a) {
                switch (_a.label) {
                    case 0:
                        if (!args.deployment_id) {
                            throw new Error("deployment_id is required");
                        }
                        return [4 /*yield*/, octokit.repos.listDeploymentStatuses({
                                owner: REPO_OWNER,
                                repo: REPO_NAME,
                                deployment_id: args.deployment_id,
                            })];
                    case 1:
                        statuses = _a.sent();
                        return [2 /*return*/, {
                                content: [
                                    {
                                        type: "text",
                                        text: JSON.stringify(statuses.data.map(function (status) { return ({
                                            id: status.id,
                                            state: status.state,
                                            description: status.description,
                                            created_at: status.created_at,
                                            updated_at: status.updated_at,
                                            target_url: status.target_url,
                                            log_url: status.log_url,
                                        }); }), null, 2),
                                    },
                                ],
                            }];
                }
            });
        });
    };
    GitHubMCPServer.prototype.getCommits = function (args) {
        return __awaiter(this, void 0, void 0, function () {
            var limit, commits;
            return __generator(this, function (_a) {
                switch (_a.label) {
                    case 0:
                        limit = args.limit || 10;
                        return [4 /*yield*/, octokit.repos.listCommits({
                                owner: REPO_OWNER,
                                repo: REPO_NAME,
                                per_page: limit,
                            })];
                    case 1:
                        commits = _a.sent();
                        return [2 /*return*/, {
                                content: [
                                    {
                                        type: "text",
                                        text: JSON.stringify(commits.data.map(function (commit) {
                                            var _a, _b;
                                            return ({
                                                sha: commit.sha,
                                                message: commit.commit.message,
                                                author: (_a = commit.commit.author) === null || _a === void 0 ? void 0 : _a.name,
                                                date: (_b = commit.commit.author) === null || _b === void 0 ? void 0 : _b.date,
                                                html_url: commit.html_url,
                                            });
                                        }), null, 2),
                                    },
                                ],
                            }];
                }
            });
        });
    };
    GitHubMCPServer.prototype.getIssues = function (args) {
        return __awaiter(this, void 0, void 0, function () {
            var limit, state, issues;
            return __generator(this, function (_a) {
                switch (_a.label) {
                    case 0:
                        limit = args.limit || 10;
                        state = args.state || "open";
                        return [4 /*yield*/, octokit.issues.listForRepo({
                                owner: REPO_OWNER,
                                repo: REPO_NAME,
                                state: state,
                                per_page: limit,
                            })];
                    case 1:
                        issues = _a.sent();
                        return [2 /*return*/, {
                                content: [
                                    {
                                        type: "text",
                                        text: JSON.stringify(issues.data.map(function (issue) {
                                            var _a;
                                            return ({
                                                number: issue.number,
                                                title: issue.title,
                                                state: issue.state,
                                                created_at: issue.created_at,
                                                updated_at: issue.updated_at,
                                                html_url: issue.html_url,
                                                user: (_a = issue.user) === null || _a === void 0 ? void 0 : _a.login,
                                                labels: issue.labels.map(function (label) {
                                                    return typeof label === "string" ? label : label.name;
                                                }),
                                            });
                                        }), null, 2),
                                    },
                                ],
                            }];
                }
            });
        });
    };
    GitHubMCPServer.prototype.getPullRequests = function (args) {
        return __awaiter(this, void 0, void 0, function () {
            var limit, state, prs;
            return __generator(this, function (_a) {
                switch (_a.label) {
                    case 0:
                        limit = args.limit || 10;
                        state = args.state || "open";
                        return [4 /*yield*/, octokit.pulls.list({
                                owner: REPO_OWNER,
                                repo: REPO_NAME,
                                state: state,
                                per_page: limit,
                            })];
                    case 1:
                        prs = _a.sent();
                        return [2 /*return*/, {
                                content: [
                                    {
                                        type: "text",
                                        text: JSON.stringify(prs.data.map(function (pr) {
                                            var _a;
                                            return ({
                                                number: pr.number,
                                                title: pr.title,
                                                state: pr.state,
                                                created_at: pr.created_at,
                                                updated_at: pr.updated_at,
                                                merged_at: pr.merged_at,
                                                html_url: pr.html_url,
                                                user: (_a = pr.user) === null || _a === void 0 ? void 0 : _a.login,
                                                head: pr.head.ref,
                                                base: pr.base.ref,
                                            });
                                        }), null, 2),
                                    },
                                ],
                            }];
                }
            });
        });
    };
    GitHubMCPServer.prototype.run = function () {
        return __awaiter(this, void 0, void 0, function () {
            var transport;
            return __generator(this, function (_a) {
                switch (_a.label) {
                    case 0:
                        transport = new stdio_js_1.StdioServerTransport();
                        return [4 /*yield*/, this.server.connect(transport)];
                    case 1:
                        _a.sent();
                        console.error("GitHub MCP server running on stdio");
                        return [2 /*return*/];
                }
            });
        });
    };
    return GitHubMCPServer;
}());
var server = new GitHubMCPServer();
server.run().catch(console.error);
