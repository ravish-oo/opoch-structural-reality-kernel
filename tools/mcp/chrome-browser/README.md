# Chrome Browser MCP Server

A Model Context Protocol (MCP) server that provides Chrome browser automation capabilities to Claude Code.

## Features

- **Navigation**: Navigate to URLs, wait for page loads
- **Screenshots**: Capture full page or element screenshots
- **Interaction**: Click, fill forms, wait for elements
- **Content Extraction**: Get HTML, extract data with selectors
- **JavaScript Execution**: Run scripts in page context
- **Tab Management**: Create, switch, and close tabs
- **State Management**: Access cookies, localStorage, sessionStorage

## Available Tools

### Navigation & Control
- `browser_navigate` - Navigate to a URL
- `browser_screenshot` - Take screenshots
- `browser_get_state` - Get browser state (URL, title, cookies)
- `browser_list_tabs` - List all open tabs

### Interaction
- `browser_click` - Click on elements
- `browser_fill` - Fill input fields
- `browser_wait_for` - Wait for elements or conditions

### Content & Data
- `browser_get_content` - Get full HTML content
- `browser_extract_data` - Extract data using CSS selectors
- `browser_execute` - Execute JavaScript in page

### Tab Management
- `browser_new_tab` - Open a new tab
- `browser_switch_tab` - Switch between tabs
- `browser_close_tab` - Close a tab

## Setup

1. Install dependencies:
   ```bash
   npm install
   npx playwright install chromium
   ```

2. Configure in Claude Code MCP settings:
   ```json
   {
     "chrome-browser": {
       "command": "node",
       "args": ["/path/to/mcp/chrome-browser/run.js"],
       "env": {
         "CHROME_HEADLESS": "false"
       }
     }
   }
   ```

3. Restart Claude Code/Cursor

## Usage Examples

```
User: "Navigate to opoch.com and take a screenshot"
Claude: [Uses browser_navigate and browser_screenshot]

User: "Fill out the contact form on the website"
Claude: [Uses browser_click, browser_fill to interact with form]

User: "Extract all the links from the page"
Claude: [Uses browser_extract_data with selector 'a' and attribute 'href']
```

## Environment Variables

- `CHROME_HEADLESS` - Set to "true" for headless mode (default: false)

## Development

Run in development mode:
```bash
npm run dev
```

Build TypeScript:
```bash
npm run build
```

## Architecture

- Uses Playwright for browser automation
- Implements MCP protocol for Claude Code integration
- Manages browser lifecycle and tab state
- Provides secure, isolated browsing sessions