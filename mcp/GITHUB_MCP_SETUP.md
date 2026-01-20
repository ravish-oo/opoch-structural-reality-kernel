# GitHub MCP Setup

## Overview
The GitHub MCP server provides direct access to GitHub APIs, allowing Claude to:
- Check workflow runs and deployment status
- Read workflow logs to debug failures
- Access issues and pull requests
- View recent commits
- Analyze deployment failures

## Setup Instructions

### 1. Create a GitHub Personal Access Token

1. Go to GitHub Settings > Developer settings > Personal access tokens
2. Click "Generate new token (classic)"
3. Give it a descriptive name like "Opoch MCP Access"
4. Select the following scopes:
   - `repo` (Full control of private repositories)
   - `workflow` (Update GitHub Action workflows)
   - `read:org` (Read org and team membership)
5. Click "Generate token" and copy it

### 2. Add Token to Environment

Add the following line to your `.env.local` file:
```
GITHUB_TOKEN=your_github_personal_access_token_here
```

### 3. Build the GitHub MCP Server

```bash
cd mcp
npm run build:github
```

### 4. Configure Claude CLI

Add this to your `.mcp.json` file:
```json
{
  "mcpServers": {
    "supabase": {
      "command": "/Users/dharamveerchouhan/Downloads/opoch/mcp/run-mcp.sh"
    },
    "github": {
      "command": "/Users/dharamveerchouhan/Downloads/opoch/mcp/run-github-mcp.sh"
    }
  }
}
```

## Available Tools

Once configured, I'll have access to:

1. **gh_workflow_runs** - Get recent GitHub Actions workflow runs
2. **gh_workflow_run_logs** - Get detailed logs for a specific run
3. **gh_deployments** - Get recent deployments
4. **gh_deployment_status** - Check deployment status
5. **gh_commits** - View recent commits
6. **gh_issues** - Access repository issues
7. **gh_pull_requests** - View pull requests

## Testing

After configuration, restart Claude CLI and I should be able to:
```
- Check recent workflow runs
- Analyze deployment failures
- Debug build issues
- Monitor PR status
```

## Security Notes

- The GitHub token is stored securely in `.env.local`
- Never commit tokens to version control
- Use minimal required permissions
- Rotate tokens regularly