#!/bin/bash

# Load environment variables from .env.local
if [ -f "../.env.local" ]; then
  export $(cat ../.env.local | grep -v '^#' | xargs)
fi

# Check for GITHUB_TOKEN
if [ -z "$GITHUB_TOKEN" ]; then
  echo "Error: GITHUB_TOKEN is not set in .env.local"
  echo "Please add GITHUB_TOKEN=your_github_personal_access_token to .env.local"
  exit 1
fi

# Run the GitHub MCP server
cd /Users/dharamveerchouhan/Downloads/opoch/mcp
exec node dist/github-mcp.js