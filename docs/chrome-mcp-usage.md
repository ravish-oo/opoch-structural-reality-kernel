# Using Chrome Browser MCP in Claude Code

Now that the Chrome Browser MCP server is configured, you can use browser automation capabilities directly in Claude Code.

## Quick Test

To test if it's working, restart Cursor/VS Code and try these commands:

### 1. Basic Navigation and Screenshot
```
"Navigate to opoch.com and take a screenshot"
```

### 2. Check Deployment
```
"Browse to https://opoch.com and verify our latest changes are live"
```

### 3. Form Testing
```
"Go to opoch.com, click the Apply button, and check if the form appears correctly"
```

## Available Commands

You can ask Claude to:

- **Navigate**: "Go to [URL]", "Open [website]"
- **Screenshot**: "Take a screenshot", "Capture the page"
- **Interact**: "Click on [element]", "Fill the form", "Submit"
- **Extract**: "Get all links", "Extract the text", "Find all images"
- **Execute JS**: "Run JavaScript to check console errors"
- **Multi-tab**: "Open a new tab", "Switch tabs", "Compare two websites"

## Example Workflows

### 1. Deployment Verification
```
"Navigate to opoch.com, take a screenshot, and check if the sign out button is visible"
```

### 2. Form Testing
```
"Go to opoch.com, click Apply, fill the form with test data, and verify it submits correctly"
```

### 3. Multi-site Comparison
```
"Open opoch.com in one tab and a competitor site in another, then compare their features"
```

## Troubleshooting

If the browser tools aren't working:

1. **Restart Cursor**: Required after MCP configuration changes
2. **Check logs**: Look for errors in Cursor's output
3. **Headless mode**: Set `CHROME_HEADLESS: "true"` for server environments
4. **Permissions**: Ensure Chrome can be launched by the process

## Advanced Usage

### Extract Structured Data
```
"Go to the blog page and extract all article titles and dates"
```

### Monitor Changes
```
"Navigate to the page and check if any content has changed since yesterday"
```

### Automated Testing
```
"Test the entire user flow: landing page → apply → fill form → submit"
```

## Security Notes

- The browser runs in an isolated profile
- No access to your personal Chrome data
- URLs are validated (no file:// access)
- Sessions are cleaned up on exit

Ready to browse! Just ask Claude to navigate to any website or perform browser automation tasks.