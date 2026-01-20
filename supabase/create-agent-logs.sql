-- ========================================
-- AGENT LOGS TABLE FOR MCP AUDIT TRAIL
-- ========================================

-- Create table for logging all MCP operations
CREATE TABLE IF NOT EXISTS public.agent_logs (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
  actor TEXT NOT NULL DEFAULT 'mcp_supabase',
  op TEXT NOT NULL, -- select, insert, update, delete, rpc, sql
  payload JSONB NOT NULL DEFAULT '{}'::jsonb,
  error TEXT,
  duration_ms INTEGER
);

-- Add indexes for performance
CREATE INDEX IF NOT EXISTS idx_agent_logs_created_at ON public.agent_logs(created_at DESC);
CREATE INDEX IF NOT EXISTS idx_agent_logs_actor ON public.agent_logs(actor);
CREATE INDEX IF NOT EXISTS idx_agent_logs_op ON public.agent_logs(op);

-- Enable RLS
ALTER TABLE public.agent_logs ENABLE ROW LEVEL SECURITY;

-- Drop existing policies if any
DROP POLICY IF EXISTS "agent_logs_insert_service" ON public.agent_logs;
DROP POLICY IF EXISTS "agent_logs_select_service" ON public.agent_logs;

-- Allow service role to insert
CREATE POLICY "agent_logs_insert_service" ON public.agent_logs
  FOR INSERT
  WITH CHECK ((auth.jwt()->>'role') IN ('service_role','supabase_admin'));

-- Allow service role to read
CREATE POLICY "agent_logs_select_service" ON public.agent_logs
  FOR SELECT
  USING ((auth.jwt()->>'role') IN ('service_role','supabase_admin'));

-- Grant permissions
GRANT USAGE ON SCHEMA public TO service_role;
GRANT ALL ON public.agent_logs TO service_role;

-- Add to whitelist in MCP server
-- The table 'agent_logs' is already included in the ALLOWED_TABLES set

-- Optional: Create a view for recent operations
CREATE OR REPLACE VIEW public.recent_agent_operations AS
SELECT 
  id,
  created_at,
  actor,
  op,
  payload->>'table' as table_name,
  CASE 
    WHEN op = 'select' THEN payload->>'params'
    WHEN op = 'insert' THEN 'rows: ' || (payload->>'rowCount')::text
    WHEN op = 'update' THEN 'match: ' || (payload->>'match')::text
    WHEN op = 'delete' THEN 'match: ' || (payload->>'match')::text
    WHEN op = 'rpc' THEN 'function: ' || (payload->>'function')::text
    WHEN op = 'sql' THEN 'mode: ' || (payload->>'mode')::text
  END as details,
  error,
  duration_ms
FROM public.agent_logs
ORDER BY created_at DESC
LIMIT 100;

-- Test insert
-- INSERT INTO public.agent_logs (actor, op, payload)
-- VALUES ('test', 'select', '{"table": "profiles", "params": {"limit": 10}}'::jsonb);