-- Create agent_logs table for MCP operation tracking
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

-- Create index for performance
CREATE INDEX IF NOT EXISTS agent_logs_created_at_idx ON public.agent_logs(created_at DESC);
CREATE INDEX IF NOT EXISTS agent_logs_op_idx ON public.agent_logs(op);