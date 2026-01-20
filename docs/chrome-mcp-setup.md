# Chrome MCP (Puppeteer) Setup Guide for Claude Code

This guide will help you set up Chrome/Puppeteer MCP server to enable web browsing capabilities in Claude Code.

## Prerequisites

1. Node.js and npm installed on your system
   - macOS: `brew install node`
   - Windows: Download from nodejs.org
   - Linux: `sudo apt install nodejs npm`

## Installation Steps

### Step 1: Install Puppeteer MCP Server

Open terminal and run:
```bash
npm install -g @modelcontextprotocol/server-puppeteer
```

### Step 2: Configure Claude Code

1. Locate your Claude Code MCP settings file:
   - **macOS**: `~/Library/Application Support/Code/User/globalStorage/saoudrizwan.claude-dev/settings/cline_mcp_settings.json`
   - **Windows**: `%APPDATA%\Code\User\globalStorage\saoudrizwan.claude-dev\settings\cline_mcp_settings.json`
   - **Linux**: `~/.config/Code/User/globalStorage/saoudrizwan.claude-dev/settings/cline_mcp_settings.json`

2. Add the Puppeteer configuration to your settings file:

```json
{
  "mcpServers": {
    "puppeteer": {
      "command": "npx",
      "args": ["-y", "@modelcontextprotocol/server-puppeteer"]
    }
  }
}
```

If you already have other MCP servers configured, add it to the existing list:

```json
{
  "mcpServers": {
    "supabase": {
      // ... existing config
    },
    "github": {
      // ... existing config
    },
    "puppeteer": {
      "command": "npx",
      "args": ["-y", "@modelcontextprotocol/server-puppeteer"]
    }
  }
}
```

### Step 3: Restart VS Code

After adding the configuration, restart VS Code for the changes to take effect.

### Step 4: Verify Installation

1. Open Claude Code in VS Code
2. Ask Claude to browse a website, for example:
   - "Can you browse to opoch.com and take a screenshot?"
   - "Please navigate to GitHub and search for Claude MCP servers"

## Available Tools

Once configured, Claude Code will have access to these Puppeteer tools:

- **puppeteer_navigate**: Navigate to URLs
- **puppeteer_screenshot**: Take screenshots of web pages
- **puppeteer_click**: Click on elements
- **puppeteer_fill**: Fill form inputs
- **puppeteer_select**: Select dropdown options
- **puppeteer_hover**: Hover over elements
- **puppeteer_evaluate**: Execute JavaScript in the browser context
- **puppeteer_get_content**: Get page HTML content

## Example Usage

```
User: "Please go to opoch.com and take a screenshot"
Claude: [Uses puppeteer_navigate to go to opoch.com]
        [Uses puppeteer_screenshot to capture the page]
        "I've navigated to opoch.com and taken a screenshot..."

User: "Can you check if the deployment on opoch.com shows our latest changes?"
Claude: [Uses puppeteer_navigate and puppeteer_get_content to inspect the page]
        "I can see the website is live with..."
```

## Troubleshooting

1. **"MCP server not found" error**:
   - Ensure npm package is installed globally
   - Check the path in your settings file is correct
   - Try using the full path to npx

2. **Browser doesn't launch**:
   - On Linux, you may need additional dependencies:
     ```bash
     sudo apt-get install -y libx11-xcb1 libxcomposite1 libxcursor1 libxdamage1 libxi6 libxtst6 libnss3 libcups2 libxss1 libxrandr2 libasound2 libpangocairo-1.0-0 libatk1.0-0 libcairo-gobject2 libgtk-3-0 libgdk-pixbuf2.0-0
     ```

3. **Debug mode**:
   - Launch Claude Code with `--mcp-debug` flag to see detailed logs

## Security Considerations

- Puppeteer runs with your user permissions
- Be cautious when automating login forms or handling sensitive data
- The browser runs in headless mode by default
- Consider using separate browser profiles for automation

## Next Steps

With Chrome MCP configured, you can now:
- Automate web testing
- Scrape dynamic content
- Generate screenshots for documentation
- Monitor live deployments
- Fill and submit web forms programmatically