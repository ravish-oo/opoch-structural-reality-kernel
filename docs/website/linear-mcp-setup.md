# Linear MCP Setup for Opoch Team

## Your Linear API Key
```
YOUR_LINEAR_API_KEY_HERE
```

## Installation Steps

### 1. Install Linear MCP Server

First, let's create a directory for MCP servers:
```bash
mkdir -p ~/mcp-servers
cd ~/mcp-servers
```

Clone the Linear MCP server:
```bash
git clone https://github.com/modelcontextprotocol/servers.git mcp-servers
cd mcp-servers/src/linear
```

Install dependencies:
```bash
npm install
npm run build
```

### 2. Configure Claude Desktop

Add the Linear MCP server to your Claude Desktop config file. The config file location varies by OS:

**macOS**: `~/Library/Application Support/Claude/claude_desktop_config.json`
**Windows**: `%APPDATA%\Claude\claude_desktop_config.json`
**Linux**: `~/.config/claude/claude_desktop_config.json`

Add this configuration:
```json
{
  "mcpServers": {
    "linear": {
      "command": "node",
      "args": ["/Users/YOUR_USERNAME/mcp-servers/mcp-servers/src/linear/dist/index.js"],
      "env": {
        "LINEAR_API_KEY": "YOUR_LINEAR_API_KEY_HERE"
      }
    }
  }
}
```

### 3. Restart Claude Desktop

After adding the configuration, restart Claude Desktop for the changes to take effect.

## Available Linear MCP Commands

Once configured, you'll have access to these tools:

1. **linear_list_issues** - List issues with filters
2. **linear_create_issue** - Create new issues
3. **linear_update_issue** - Update existing issues
4. **linear_get_issue** - Get issue details
5. **linear_create_comment** - Add comments to issues
6. **linear_list_projects** - List all projects
7. **linear_list_teams** - List all teams
8. **linear_search_issues** - Search issues by query

## Example Usage

### List current issues:
```
linear_list_issues({ 
  teamId: "OPOCH", 
  includeArchived: false,
  orderBy: "updatedAt"
})
```

### Create a new issue:
```
linear_create_issue({
  title: "Fix phone input focus issue",
  description: "Users losing focus when typing in phone field",
  teamId: "OPOCH",
  priority: 3,
  labelIds: ["bug"]
})
```

### Update issue status:
```
linear_update_issue({
  issueId: "ISSUE_ID",
  stateId: "done"
})
```

## Quick API Test

You can test the API connection directly:
```bash
curl -X POST https://api.linear.app/graphql \
  -H "Authorization: YOUR_LINEAR_API_KEY_HERE" \
  -H "Content-Type: application/json" \
  -d '{"query": "{ viewer { id name email } }"}'
```

## Security Note
Remember to keep your API key secure. Consider using environment variables instead of hardcoding it in config files.