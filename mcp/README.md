# Supabase MCP Server for Opoch

This MCP (Model Context Protocol) server allows Claude CLI to directly interact with your Supabase database.

## Setup Instructions

### 1. Install Dependencies

```bash
cd mcp
npm install
```

### 2. Build the MCP Server

```bash
npm run build
```

### 3. Configure Environment Variables

Create a `.env` file in the `mcp` directory:

```env
SUPABASE_URL=https://your-project.supabase.co
SUPABASE_SERVICE_ROLE_KEY=your-service-role-key
```

### 4. Configure Claude CLI

The MCP server is already configured for Claude CLI. The configuration is in `claude-cli-config.json`:

```json
{
  "mcpServers": {
    "supabase": {
      "command": "/Users/dharamveerchouhan/Downloads/opoch/mcp/run-mcp.sh"
    }
  }
}
```

The `run-mcp.sh` script automatically loads environment variables from `.env.local`.

**Note**: This setup uses the actual credentials from your `.env.local` file, so the MCP server has full access to your Supabase database with the service role key.

### 5. Create Audit Table (Optional but Recommended)

Run this in your Supabase SQL editor to enable operation logging:

```sql
CREATE TABLE IF NOT EXISTS public.agent_logs (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
  actor TEXT NOT NULL DEFAULT 'mcp_supabase',
  op TEXT NOT NULL,
  payload JSONB NOT NULL DEFAULT '{}'::jsonb
);

-- Enable RLS
ALTER TABLE public.agent_logs ENABLE ROW LEVEL SECURITY;

-- Allow service role to insert
CREATE POLICY "agent_logs_insert_service" ON public.agent_logs
  FOR INSERT
  WITH CHECK ((auth.jwt()->>'role') IN ('service_role','supabase_admin'));

-- Allow service role to read
CREATE POLICY "agent_logs_select_service" ON public.agent_logs
  FOR SELECT
  USING ((auth.jwt()->>'role') IN ('service_role','supabase_admin'));
```

### 6. Test the Connection

Run the test script to verify everything is working:

```bash
npm run test
```

This will check that all tables exist and the connection is working properly.

## Available Tools

Once configured, Claude will have access to these tools:

### `sb_select`
Select rows from a table:
```json
{
  "table": "profiles",
  "columns": "*",
  "match": { "id": "user-uuid" },
  "limit": 10,
  "order": { "column": "created_at", "ascending": false }
}
```

### `sb_insert`
Insert rows into a table:
```json
{
  "table": "queries",
  "rows": [
    {
      "user_id": "user-uuid",
      "text": "How do I implement caching?",
      "context": { "source": "mcp" }
    }
  ]
}
```

### `sb_update`
Update rows in a table:
```json
{
  "table": "profiles",
  "match": { "id": "user-uuid" },
  "patch": { "has_completed_profile": true }
}
```

### `sb_delete`
Delete rows from a table:
```json
{
  "table": "queries",
  "match": { "id": "query-uuid" }
}
```

### `sb_rpc`
Call a Postgres function:
```json
{
  "fn": "handle_new_user",
  "args": { "user_id": "user-uuid" }
}
```

### `sb_sql`
Execute raw SQL (use with caution):
```json
{
  "query": "SELECT COUNT(*) FROM profiles",
  "mode": "read"
}
```

## Security Notes

1. **Service Role Key**: Keep this key secure. It bypasses RLS.
2. **Whitelisted Tables**: Only the tables in `ALLOWED_TABLES` can be accessed.
3. **Audit Logs**: All operations are logged to `agent_logs` table.
4. **SQL Queries**: Raw SQL is restricted by default. Use with caution.

## Testing

To test the MCP server locally:

```bash
npm run dev
```

Then send JSON-RPC requests via stdin to test the tools.

## Troubleshooting

1. **Connection Issues**: Verify your Supabase URL and service role key
2. **Permission Errors**: Ensure the service role key is correct
3. **Tool Not Found**: Restart Claude Desktop after config changes
4. **Table Not Allowed**: Add the table to the `ALLOWED_TABLES` set in the server code