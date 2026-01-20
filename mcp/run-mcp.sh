#!/bin/bash
# Run the MCP server with proper environment
cd /Users/dharamveerchouhan/Downloads/opoch/mcp
export $(cat ../.env.local | grep -v '^#' | xargs)
node dist/supabase-mcp.js